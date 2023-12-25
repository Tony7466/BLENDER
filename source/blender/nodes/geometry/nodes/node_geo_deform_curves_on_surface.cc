/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_editmesh.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_wrapper.hh"
#include "BKE_modifier.hh"

#include "BLI_math_matrix.hh"
#include "BLI_task.hh"

#include "GEO_reverse_uv_sampler.hh"

#include "DEG_depsgraph_query.hh"

#include "node_geometry_util.hh"

#include <fmt/format.h>

#include <map>

using blender::geometry::ReverseUVSampler;

struct NodeGeometryDeformCurvesOnSurfaceCacheEntry {
  blender::Array<ReverseUVSampler::Result> old_samples;
  blender::Array<ReverseUVSampler::Result> new_samples;
  blender::uint2 tri_counts = {0, 0};
  blender::uint2 uv_map_counts = {0, 0};
  std::mutex mutex;
};

struct NodeGeometryDeformCurvesOnSurfaceCache
    : public blender::Map<blender::uint2, NodeGeometryDeformCurvesOnSurfaceCacheEntry *> {
  ~NodeGeometryDeformCurvesOnSurfaceCache()
  {
    foreach_item([&]([[maybe_unused]] auto k, auto v) {
      MEM_delete<NodeGeometryDeformCurvesOnSurfaceCacheEntry>(v);
    });
  }
};

namespace blender::nodes::node_geo_deform_curves_on_surface_cc {

using bke::CurvesGeometry;
using bke::attribute_math::mix3;

NODE_STORAGE_FUNCS(NodeGeometryDeformCurvesOnSurface)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Curves").supported_type(GeometryComponent::Type::Curve);
  b.add_output<decl::Geometry>("Curves").propagate_all();
}

static void node_init(bNodeTree *, bNode *node)
{
  NodeGeometryDeformCurvesOnSurface *data = MEM_new<NodeGeometryDeformCurvesOnSurface>(__func__);
  data->cache = 0;
  node->storage = data;
}

/***
 * Surface_mesh_orig usually stays unchanged from call to call, but it may change.
 * We use tri_counts and uv_map_counts to detect changes, and use cached sampling results
 * as hints as long as counts stay the same. (We still need to run sampling, because we
 * don't keep full info about the previous mesh, so we can't detect all changes.)
 *
 * With regard to surface_mesh_eval, there are a few possible scenarios.
 * 1. surface_mesh_eval is topologically the same as surface_mesh_orig, except possibly
 *    deformed by armature / shape keys (identical corner lists and UV maps); there's no
 *    need to rerun ReverseUVSampler, because all curves stay attached to the same places.
 *
 * 2. The mesh is substantially different; commonly happens if there's a subdivide modifier
 *   on the mesh (because 'surface_mesh_orig' will be the version without subdivision).
 *
 *  -> 2.1. It is different from surface_mesh_orig, but it is (topologically) the same as
 *     in the previous call. Use cached result as hints.
 *
 *  -> 2.2. It is different from both surface_mesh_orig and from the previous call. Run
 *          ReverseUVSampler without hints.
 *
 * */
static void deform_curves(const CurvesGeometry &curves,
                          const Mesh &surface_mesh_old,
                          const Mesh &surface_mesh_new,
                          const Span<float2> curve_attachment_uvs,
                          const ReverseUVSampler *reverse_uv_sampler_old,
                          const ReverseUVSampler *reverse_uv_sampler_new,
                          const Span<float3> corner_normals_old,
                          const Span<float3> corner_normals_new,
                          const Span<float3> rest_positions,
                          const float4x4 &surface_to_curves,
                          const Span<float3> r_orig_positions,
                          MutableSpan<float3> r_positions,
                          MutableSpan<float3x3> r_rotations,
                          std::atomic<int> &r_invalid_uv_count,
                          uint2 uv_map_counts,
                          NodeGeometryDeformCurvesOnSurfaceCacheEntry *cache = nullptr)
{
  /* Find attachment points on old and new mesh. */
  const int curves_num = curves.curves_num();
  Array<ReverseUVSampler::Result> surface_samples_old(curves_num);
  Array<ReverseUVSampler::Result> surface_samples_new(curves_num);
  const uint old_mesh_size = surface_mesh_old.corner_tris().size();
  const uint new_mesh_size = surface_mesh_new.corner_tris().size();

  /* this function is not expected to be called from multiple threads for the same object,
     but we'll lock up just in case */
  if (cache)
    cache->mutex.lock();

  bool hints = false;
  if (cache && cache->old_samples.size() == curves_num && cache->tri_counts[0] == old_mesh_size &&
      cache->uv_map_counts[0] == uv_map_counts[0])
  {
    surface_samples_old = cache->old_samples;
    hints = true;
  }

  reverse_uv_sampler_old->sample_many(curve_attachment_uvs, surface_samples_old, hints);

  if (reverse_uv_sampler_new != nullptr) {
    /*
     * If we get here, it means that at least one entry in either corner_tris or uv_map is
     * different between 'orig' and 'eval'.
     */
    hints = false;
    if (cache && cache->new_samples.size() == curves_num &&
        cache->tri_counts[1] == new_mesh_size && cache->uv_map_counts[1] == uv_map_counts[1])
    {
      hints = true;
      surface_samples_new = cache->new_samples;
    }
    else if (surface_mesh_old.corner_tris().size() == new_mesh_size &&
             uv_map_counts[0] == uv_map_counts[1])
    {
      hints = true;
      surface_samples_new = surface_samples_old;
    }
    reverse_uv_sampler_new->sample_many(curve_attachment_uvs, surface_samples_new, hints);
  }
  else {
    surface_samples_new = surface_samples_old;
  }

  const float4x4 curves_to_surface = math::invert(surface_to_curves);

  const Span<float3> surface_positions_old = surface_mesh_old.vert_positions();
  const Span<int> surface_corner_verts_old = surface_mesh_old.corner_verts();
  const Span<int3> surface_corner_tris_old = surface_mesh_old.corner_tris();

  const Span<float3> surface_positions_new = surface_mesh_new.vert_positions();
  const Span<int> surface_corner_verts_new = surface_mesh_new.corner_verts();
  const Span<int3> surface_corner_tris_new = surface_mesh_new.corner_tris();

  const OffsetIndices points_by_curve = curves.points_by_curve();

  threading::parallel_for(curves.curves_range(), 256, [&](const IndexRange range) {
    int last_known_good_point = -1;
    for (const int curve_i : range) {
      bool invalid = false;
      const ReverseUVSampler::Result &surface_sample_old = surface_samples_old[curve_i];
      if (surface_sample_old.type != ReverseUVSampler::ResultType::Ok)
        invalid = true;

      const ReverseUVSampler::Result &surface_sample_new = surface_samples_new[curve_i];
      if (surface_sample_new.type == ReverseUVSampler::ResultType::Multiple) {
        invalid = true;
      } else if(surface_sample_new.type == ReverseUVSampler::ResultType::None) {
        /* UVs may end up outside the triangle during mapping from unsubdivided to 
         * subdivided mesh. */
        auto bary_weights = surface_sample_new.bary_weights;
        const float x_dist = std::max(-bary_weights.x, bary_weights.x - 1.0f);
        const float y_dist = std::max(-bary_weights.y, bary_weights.y - 1.0f);
        const float z_dist = std::max(-bary_weights.z, bary_weights.z - 1.0f);
        const float dist = std::max({x_dist, y_dist, z_dist});
        if(dist > 0.01f)
          invalid = true;
      }

      if (invalid) {
        r_invalid_uv_count++;
        const IndexRange points = points_by_curve[curve_i];
        /* There are no particularly good options here. Deleting invalid curves is 
         * potentially an very expensive operation. Leaving them at original locations
         * creates clumps of unattached hairs hanging in midair.
         * 
         * We set all points of an invalid curve to the same value, reducing it to 
         * a point (making it invisible in 'strand' mode and very small in 'strip' mode.)
         * To obscure it even further, we also move it to the location of the last known 
         * good root.
         * */
        float3 pos = last_known_good_point>=0 
                        ? r_positions[last_known_good_point]
                        : r_orig_positions[points[0]];
        for (const int point_i : points)
          r_positions[point_i] = pos;
        continue;
      }
      if(points_by_curve[curve_i].size() > 0)
        last_known_good_point = points_by_curve[curve_i][0];

      const int3 &tri_old = surface_corner_tris_old[surface_sample_old.tri_index];
      const int3 &tri_new = surface_corner_tris_new[surface_sample_new.tri_index];
      const float3 &bary_weights_old = surface_sample_old.bary_weights;
      const float3 &bary_weights_new = surface_sample_new.bary_weights;

      const int corner_0_old = tri_old[0];
      const int corner_1_old = tri_old[1];
      const int corner_2_old = tri_old[2];

      const int corner_0_new = tri_new[0];
      const int corner_1_new = tri_new[1];
      const int corner_2_new = tri_new[2];

      const int vert_0_old = surface_corner_verts_old[corner_0_old];
      const int vert_1_old = surface_corner_verts_old[corner_1_old];
      const int vert_2_old = surface_corner_verts_old[corner_2_old];

      const int vert_0_new = surface_corner_verts_new[corner_0_new];
      const int vert_1_new = surface_corner_verts_new[corner_1_new];
      const int vert_2_new = surface_corner_verts_new[corner_2_new];

      const float3 &normal_0_old = corner_normals_old[corner_0_old];
      const float3 &normal_1_old = corner_normals_old[corner_1_old];
      const float3 &normal_2_old = corner_normals_old[corner_2_old];
      const float3 normal_old = math::normalize(
          mix3(bary_weights_old, normal_0_old, normal_1_old, normal_2_old));

      const float3 &normal_0_new = corner_normals_new[corner_0_new];
      const float3 &normal_1_new = corner_normals_new[corner_1_new];
      const float3 &normal_2_new = corner_normals_new[corner_2_new];
      const float3 normal_new = math::normalize(
          mix3(bary_weights_new, normal_0_new, normal_1_new, normal_2_new));

      const float3 &pos_0_old = surface_positions_old[vert_0_old];
      const float3 &pos_1_old = surface_positions_old[vert_1_old];
      const float3 &pos_2_old = surface_positions_old[vert_2_old];
      const float3 pos_old = mix3(bary_weights_old, pos_0_old, pos_1_old, pos_2_old);

      const float3 &pos_0_new = surface_positions_new[vert_0_new];
      const float3 &pos_1_new = surface_positions_new[vert_1_new];
      const float3 &pos_2_new = surface_positions_new[vert_2_new];
      const float3 pos_new = mix3(bary_weights_new, pos_0_new, pos_1_new, pos_2_new);

      /* The translation is just the difference between the old and new position on the surface. */
      const float3 translation = pos_new - pos_old;

      const float3 &rest_pos_0 = rest_positions[vert_0_new];
      const float3 &rest_pos_1 = rest_positions[vert_1_new];

      /* The tangent reference direction is used to determine the rotation of the surface point
       * around its normal axis. It's important that the old and new tangent reference are computed
       * in a consistent way. If the surface has not been rotated, the old and new tangent
       * reference have to have the same direction. For that reason, the old tangent reference is
       * computed based on the rest position attribute instead of positions on the old mesh. This
       * way the old and new tangent reference use the same topology.
       *
       * TODO: Figure out if this can be smoothly interpolated across the surface as well.
       * Currently, this is a source of discontinuity in the deformation, because the vector
       * changes instantly from one triangle to the next. */
      const float3 tangent_reference_dir_old = rest_pos_1 - rest_pos_0;
      const float3 tangent_reference_dir_new = pos_1_new - pos_0_new;

      /* Compute first local tangent based on the (potentially smoothed) normal and the tangent
       * reference. */
      const float3 tangent_x_old = math::normalize(
          math::cross(normal_old, tangent_reference_dir_old));
      const float3 tangent_x_new = math::normalize(
          math::cross(normal_new, tangent_reference_dir_new));

      /* The second tangent defined by the normal and first tangent. */
      const float3 tangent_y_old = math::normalize(math::cross(normal_old, tangent_x_old));
      const float3 tangent_y_new = math::normalize(math::cross(normal_new, tangent_x_new));

      /* Construct rotation matrix that encodes the orientation of the old surface position. */
      float3x3 rotation_old(tangent_x_old, tangent_y_old, normal_old);

      /* Construct rotation matrix that encodes the orientation of the new surface position. */
      float3x3 rotation_new(tangent_x_new, tangent_y_new, normal_new);

      /* Can use transpose instead of inverse because the matrix is orthonormal. In the case of
       * zero-area triangles, the matrix would not be orthonormal, but in this case, none of this
       * works anyway. */
      const float3x3 rotation_old_inv = math::transpose(rotation_old);

      /* Compute a rotation matrix that rotates points from the old to the new surface
       * orientation. */
      const float3x3 rotation = rotation_new * rotation_old_inv;

      /* Construction transformation matrix for this surface position that includes rotation and
       * translation. */
      /* Subtract and add #pos_old, so that the rotation origin is the position on the surface. */
      float4x4 surface_transform = math::from_origin_transform<float4x4>(float4x4(rotation),
                                                                         pos_old);
      surface_transform.location() += translation;

      /* Change the basis of the transformation so to that it can be applied in the local space of
       * the curves. */
      const float4x4 curve_transform = surface_to_curves * surface_transform * curves_to_surface;

      /* Actually transform all points. */
      const IndexRange points = points_by_curve[curve_i];
      for (const int point_i : points) {
        const float3 old_point_pos = r_orig_positions[point_i];
        const float3 new_point_pos = math::transform_point(curve_transform, old_point_pos);
        r_positions[point_i] = new_point_pos;
      }

      if (!r_rotations.is_empty()) {
        for (const int point_i : points) {
          r_rotations[point_i] = rotation * r_rotations[point_i];
        }
      }
    }
  });

  if (cache) {
    cache->old_samples = std::move(surface_samples_old);
    cache->new_samples = std::move(surface_samples_new);
    cache->tri_counts = uint2{old_mesh_size, new_mesh_size};
    cache->uv_map_counts = uv_map_counts;
    cache->mutex.unlock();
  }
}

template<typename T> bool threaded_compare(const T &va, const T &vb)
{
  std::atomic<bool> difference(false);
  threading::parallel_for(IndexRange(va.size()), 1024, [&](const IndexRange range) {
    for (const int i : range) {
      if (va[i] != vb[i]) {
        difference = true;
        break;
      }
    }
  });
  return difference;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet curves_geometry = params.extract_input<GeometrySet>("Curves");

  Mesh *surface_mesh_orig = nullptr;
  bool free_suface_mesh_orig = false;
  BLI_SCOPED_DEFER([&]() {
    if (free_suface_mesh_orig) {
      BKE_id_free(nullptr, surface_mesh_orig);
    }
  });

  auto pass_through_input = [&]() { params.set_output("Curves", std::move(curves_geometry)); };

  const Object *self_ob_eval = params.self_object();
  if (self_ob_eval == nullptr || self_ob_eval->type != OB_CURVES) {
    pass_through_input();
    params.error_message_add(NodeWarningType::Error, TIP_("Node only works for curves objects"));
    return;
  }
  const Curves *self_curves_eval = static_cast<const Curves *>(self_ob_eval->data);
  if (self_curves_eval->surface_uv_map == nullptr || self_curves_eval->surface_uv_map[0] == '\0') {
    pass_through_input();
    params.error_message_add(NodeWarningType::Error, TIP_("Surface UV map not defined"));
    return;
  }
  /* Take surface information from self-object. */
  Object *surface_ob_eval = self_curves_eval->surface;
  const StringRefNull uv_map_name = self_curves_eval->surface_uv_map;
  const StringRefNull rest_position_name = "rest_position";

  if (!curves_geometry.has_curves()) {
    pass_through_input();
    return;
  }
  if (surface_ob_eval == nullptr || surface_ob_eval->type != OB_MESH) {
    pass_through_input();
    params.error_message_add(NodeWarningType::Error, TIP_("Curves not attached to a surface"));
    return;
  }
  Object *surface_ob_orig = DEG_get_original_object(surface_ob_eval);
  Mesh &surface_object_data = *static_cast<Mesh *>(surface_ob_orig->data);

  if (BMEditMesh *em = surface_object_data.runtime->edit_mesh.get()) {
    surface_mesh_orig = BKE_mesh_from_bmesh_for_eval_nomain(em->bm, nullptr, &surface_object_data);
    free_suface_mesh_orig = true;
  }
  else {
    surface_mesh_orig = &surface_object_data;
  }
  Mesh *surface_mesh_eval = BKE_modifier_get_evaluated_mesh_from_evaluated_object(surface_ob_eval);
  if (surface_mesh_eval == nullptr) {
    pass_through_input();
    params.error_message_add(NodeWarningType::Error, TIP_("Surface has no mesh"));
    return;
  }

  BKE_mesh_wrapper_ensure_mdata(surface_mesh_eval);

  const AttributeAccessor mesh_attributes_eval = surface_mesh_eval->attributes();
  const AttributeAccessor mesh_attributes_orig = surface_mesh_orig->attributes();

  Curves &curves_id = *curves_geometry.get_curves_for_write();
  CurvesGeometry &curves = curves_id.geometry.wrap();

  if (!mesh_attributes_eval.contains(uv_map_name)) {
    pass_through_input();
    const std::string message = fmt::format(TIP_("Evaluated surface missing UV map: \"{}\""),
                                            uv_map_name);
    params.error_message_add(NodeWarningType::Error, message);
    return;
  }
  if (!mesh_attributes_orig.contains(uv_map_name)) {
    pass_through_input();
    const std::string message = fmt::format(TIP_("Original surface missing UV map: \"{}\""),
                                            uv_map_name);
    params.error_message_add(NodeWarningType::Error, message);
    return;
  }
  if (!mesh_attributes_eval.contains(rest_position_name)) {
    pass_through_input();
    params.error_message_add(NodeWarningType::Error,
                             TIP_("Evaluated surface missing attribute: \"rest_position\""));
    return;
  }
  if (curves.surface_uv_coords().is_empty() && curves.curves_num() > 0) {
    pass_through_input();
    params.error_message_add(NodeWarningType::Error,
                             TIP_("Curves are not attached to any UV map"));
    return;
  }
  const VArraySpan uv_map_orig = *mesh_attributes_orig.lookup<float2>(uv_map_name,
                                                                      AttrDomain::Corner);
  const VArraySpan uv_map_eval = *mesh_attributes_eval.lookup<float2>(uv_map_name,
                                                                      AttrDomain::Corner);
  const VArraySpan rest_positions = *mesh_attributes_eval.lookup<float3>(rest_position_name,
                                                                         AttrDomain::Point);
  const VArraySpan surface_uv_coords = *curves.attributes().lookup_or_default<float2>(
      "surface_uv_coordinate", AttrDomain::Curve, float2(0));

  const Span<int3> corner_tris_orig = surface_mesh_orig->corner_tris();
  const Span<int3> corner_tris_eval = surface_mesh_eval->corner_tris();
  uint2 uv_map_counts{(uint)uv_map_orig.size(), (uint)uv_map_eval.size()};

  bool same_mesh = false;
  float2 uv_min = {0, 0}, uv_max = {0, 0};

  if (surface_uv_coords.size() > 0) {
    uv_min = uv_max = surface_uv_coords[0];
    for (auto x : surface_uv_coords) {
      uv_min = math::min(uv_min, x);
      uv_max = math::max(uv_max, x);
    }
  }

  same_mesh = (corner_tris_orig.size() == corner_tris_eval.size()) &&
              (uv_map_orig.size() == uv_map_eval.size());
  if (same_mesh)
    same_mesh = !threaded_compare(corner_tris_eval, corner_tris_orig);
  if (same_mesh)
    same_mesh = !threaded_compare(uv_map_eval, uv_map_orig);

  std::unique_ptr<ReverseUVSampler> reverse_uv_sampler_orig(
      new ReverseUVSampler(uv_map_orig, corner_tris_orig, uv_min, uv_max));
  std::unique_ptr<ReverseUVSampler> reverse_uv_sampler_eval;
  if (!same_mesh)
    reverse_uv_sampler_eval.reset(
        new ReverseUVSampler(uv_map_eval, corner_tris_eval, uv_min, uv_max));

  /* Retrieve face corner normals from each mesh. It's necessary to use face corner normals
   * because face normals or vertex normals may lose information (custom normals, auto smooth) in
   * some cases. */
  const Span<float3> corner_normals_orig = surface_mesh_orig->corner_normals();
  const Span<float3> corner_normals_eval = surface_mesh_eval->corner_normals();

  std::atomic<int> invalid_uv_count = 0;

  const bke::CurvesSurfaceTransforms transforms{*self_ob_eval, surface_ob_eval};

  bke::CurvesEditHints *edit_hints = curves_geometry.get_curve_edit_hints_for_write();
  MutableSpan<float3> edit_hint_positions;
  MutableSpan<float3x3> edit_hint_rotations;
  if (edit_hints != nullptr) {
    if (const std::optional<MutableSpan<float3>> positions = edit_hints->positions_for_write()) {
      edit_hint_positions = *positions;
    }
    if (!edit_hints->deform_mats.has_value()) {
      edit_hints->deform_mats.emplace(edit_hints->curves_id_orig.geometry.point_num,
                                      float3x3::identity());
      edit_hints->deform_mats->fill(float3x3::identity());
    }
    edit_hint_rotations = *edit_hints->deform_mats;
  }

  bNode &node = const_cast<bNode &>(params.node());
  NodeGeometryDeformCurvesOnSurface &storage = node_storage(node);
  NodeGeometryDeformCurvesOnSurfaceCache *cache = nullptr;
  NodeGeometryDeformCurvesOnSurfaceCacheEntry *cache_entry = nullptr;

  cache = reinterpret_cast<NodeGeometryDeformCurvesOnSurfaceCache *>(storage.cache);
  if (cache == 0)
    cache = MEM_new<NodeGeometryDeformCurvesOnSurfaceCache>(__func__);
  storage.cache = reinterpret_cast<uint64_t &>(cache);
  /* In case of multiple curves objects on the same mesh, a unique ID of the curves
   * object would work better, but I can't find one */
  uint2 key{(uint)surface_mesh_orig->id.session_uid, (uint)curves.curves_num()};
  if (!cache->contains(key))
    cache->add(key, MEM_new<NodeGeometryDeformCurvesOnSurfaceCacheEntry>(__func__));
  cache_entry = cache->lookup(key);

  auto orig_positions = curves.positions();
  /* This is a somewhat expensive call: it allocates memory for the output array.
     We pass 'preserve'='false' to indicate that we don't need the contents of the
     buffer we receive; otherwise, it would be filled twice. */
  MutableSpan<float3> new_positions = curves.positions_for_write(false);

  if (edit_hint_positions.is_empty()) {
    deform_curves(curves,
                  *surface_mesh_orig,
                  *surface_mesh_eval,
                  surface_uv_coords,
                  reverse_uv_sampler_orig.get(),
                  reverse_uv_sampler_eval.get(),
                  corner_normals_orig,
                  corner_normals_eval,
                  rest_positions,
                  transforms.surface_to_curves,
                  orig_positions,
                  new_positions,
                  edit_hint_rotations,
                  invalid_uv_count,
                  uv_map_counts,
                  cache_entry);
  }
  else {
    /* First deform the actual curves in the input geometry. */
    deform_curves(curves,
                  *surface_mesh_orig,
                  *surface_mesh_eval,
                  surface_uv_coords,
                  reverse_uv_sampler_orig.get(),
                  reverse_uv_sampler_eval.get(),
                  corner_normals_orig,
                  corner_normals_eval,
                  rest_positions,
                  transforms.surface_to_curves,
                  orig_positions,
                  new_positions,
                  {},
                  invalid_uv_count,
                  uv_map_counts,
                  cache_entry);

    /* Then also deform edit curve information for use in sculpt mode. */
    const CurvesGeometry &curves_orig = edit_hints->curves_id_orig.geometry.wrap();
    const VArraySpan<float2> surface_uv_coords_orig = *curves_orig.attributes().lookup_or_default(
        "surface_uv_coordinate", AttrDomain::Curve, float2(0));
    if (!surface_uv_coords_orig.is_empty()) {
      deform_curves(curves_orig,
                    *surface_mesh_orig,
                    *surface_mesh_eval,
                    surface_uv_coords_orig,
                    reverse_uv_sampler_orig.get(),
                    reverse_uv_sampler_eval.get(),
                    corner_normals_orig,
                    corner_normals_eval,
                    rest_positions,
                    transforms.surface_to_curves,
                    edit_hint_positions,
                    edit_hint_positions,
                    edit_hint_rotations,
                    invalid_uv_count,
                    uv_map_counts);
    }
  }
  curves.tag_positions_changed();

  if (invalid_uv_count) {
    const std::string message = fmt::format(TIP_("Invalid surface UVs on {} curves"),
                                            invalid_uv_count.load());
    params.error_message_add(NodeWarningType::Warning, message);
  }

  params.set_output("Curves", curves_geometry);
}

void node_free_CurvesOnSurface_storage(bNode *node);
void node_copy_CurvesOnSurface_storage(bNodeTree *, bNode *, const bNode *);

void node_free_CurvesOnSurface_storage(bNode *node)
{
  if (node->storage) {
    auto storage = (NodeGeometryDeformCurvesOnSurface *)node->storage;
    if (storage->cache) {
      NodeGeometryDeformCurvesOnSurfaceCache *cache =
          reinterpret_cast<NodeGeometryDeformCurvesOnSurfaceCache *>(storage->cache);
      MEM_delete<NodeGeometryDeformCurvesOnSurfaceCache>(cache);
    }
    MEM_delete<NodeGeometryDeformCurvesOnSurface>(storage);
  }
}

void node_copy_CurvesOnSurface_storage(bNodeTree * /*dest_ntree*/,
                                       bNode *dest_node,
                                       [[maybe_unused]] const bNode *src_node)
{
  auto storage = MEM_new<NodeGeometryDeformCurvesOnSurface>(__func__);
  storage->cache = 0;
  dest_node->storage = storage;
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_DEFORM_CURVES_ON_SURFACE, "Deform Curves on Surface", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  node_type_storage(&ntype,
                    "NodeGeometryDeformCurvesOnSurface",
                    node_free_CurvesOnSurface_storage,
                    node_copy_CurvesOnSurface_storage);
  ntype.initfunc = node_init;
  blender::bke::node_type_size(&ntype, 170, 120, 700);
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_deform_curves_on_surface_cc
