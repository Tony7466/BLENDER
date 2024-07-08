/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "editors/sculpt_paint/brushes/types.hh"

#include "DNA_brush_types.h"
#include "DNA_mesh_types.h"

#include "BLI_enumerable_thread_specific.hh"
#include "BLI_math_base.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"
#include "BLI_task.hh"

#include "editors/sculpt_paint/mesh_brush_common.hh"
#include "editors/sculpt_paint/sculpt_intern.hh"

namespace blender::ed::sculpt_paint {

inline namespace relax_face_sets_cc {

struct LocalData {
  Vector<float> distances;
  Vector<Vector<int>> vert_neighbors;
  Vector<float3> translations;
};

static std::array<float, 4> iteration_strengths(const float strength, const int stroke_iteration)
{
  if (stroke_iteration % 3 == 0) {
    return {strength, strength, strength, strength};
  }

  /* This operations needs a strength tweak as the relax deformation is too weak by default. */
  const float modified_strength = strength * 1.5f;
  return {modified_strength, modified_strength, strength, strength};
}

static void filter_factors_on_face_sets(const GroupedSpan<int> vert_to_face_map,
                                        const int *face_sets,
                                        const bool relax_face_sets,
                                        const Span<int> verts,
                                        const MutableSpan<float> factors)
{
  BLI_assert(verts.size() == factors.size());

  for (const int i : verts.index_range()) {
    if (relax_face_sets ==
        face_set::vert_has_unique_face_set_mesh(vert_to_face_map, face_sets, verts[i]))
    {
      factors[i] = 0.0f;
    }
  }
}

/* -------------------------------------------------------------------- */
/** \name Relax Vertex
 * \{ */

static bool get_normal_boundary(const GroupedSpan<int> vert_to_face_map,
                                const int *face_sets,
                                const BitSpan boundary_verts,
                                const float3 &current_position,
                                const Span<float3> vert_positions,
                                const bool filter_boundary_face_sets,
                                const Span<int> neighbors,
                                float3 &r_new_normal)
{
  int total = 0;
  float3 normal(0.0f, 0.0f, 0.0f);
  for (const int vert : neighbors) {
    /* If we are filtering face sets, then we only want to affect vertices that have more than one
     * face set, i.e. are on the boundary of a face set and another face set. */
    if (filter_boundary_face_sets &&
        face_set::vert_has_unique_face_set_mesh(vert_to_face_map, face_sets, vert))
    {
      continue;
    }

    /* When the vertex to relax is boundary, use only connected boundary vertices for the average
     * position. */
    if (!boundary_verts[vert]) {
      continue;
    }

    const float3 to_neighbor = vert_positions[vert] - current_position;
    normal += math::normalize(to_neighbor);
    total++;
  }

  /* If we are not dealing with a corner vertex, skip this step.*/
  if (total != 2) {
    return false;
  }

  r_new_normal = math::normalize(normal);

  return true;
}

static bool get_average_position_boundary(const GroupedSpan<int> vert_to_face_map,
                                          const int *face_sets,
                                          const BitSpan boundary_verts,
                                          const Span<float3> vert_positions,
                                          const bool filter_boundary_face_sets,
                                          const Span<int> neighbors,
                                          float3 &r_new_position)
{
  int total = 0;
  float3 average_position(0.0f, 0.0f, 0.0f);
  for (const int vert : neighbors) {
    /* If we are filtering face sets, then we only want to affect vertices that have more than one
     * face set, i.e. are on the boundary of a face set and another face set. */
    if (filter_boundary_face_sets &&
        face_set::vert_has_unique_face_set_mesh(vert_to_face_map, face_sets, vert))
    {
      continue;
    }

    /* When the vertex to relax is boundary, use only connected boundary vertices for the average
     * position. */
    if (!boundary_verts[vert]) {
      continue;
    }

    average_position += vert_positions[vert];
    total++;
  }

  if (total == 0) {
    return false;
  }

  average_position *= math::rcp(float(total));
  r_new_position = average_position;

  return true;
}

static bool get_average_position_interior(const GroupedSpan<int> vert_to_face_map,
                                          const int *face_sets,
                                          const Span<float3> vert_positions,
                                          const bool filter_boundary_face_sets,
                                          const Span<int> neighbors,
                                          float3 &r_new_position)
{
  int total = 0;
  float3 average_position(0.0f, 0.0f, 0.0f);
  for (const int vert : neighbors) {
    /* If we are filtering face sets, then we only want to affect vertices that have more than one
     * face set, i.e. are on the boundary of a face set and another face set. */
    if (filter_boundary_face_sets &&
        face_set::vert_has_unique_face_set_mesh(vert_to_face_map, face_sets, vert))
    {
      continue;
    }

    average_position += vert_positions[vert];
    total++;
  }

  if (total == 0) {
    return false;
  }

  average_position *= math::rcp(float(total));
  r_new_position = average_position;

  return true;
}

/** \} */

static void calc_factors_faces(const Brush &brush,
                               const Span<float3> positions_eval,
                               const Span<float3> vert_normals,
                               const PBVHNode &node,
                               const float strength,
                               const bool relax_face_sets,
                               Object &object,
                               LocalData &tls,
                               const MutableSpan<float> factors)
{
  SculptSession &ss = *object.sculpt;
  const StrokeCache &cache = *ss.cache;
  const Mesh &mesh = *static_cast<Mesh *>(object.data);

  const Span<int> verts = bke::pbvh::node_unique_verts(node);

  fill_factor_from_hide_and_mask(mesh, verts, factors);
  filter_region_clip_factors(ss, positions_eval, verts, factors);
  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, vert_normals, verts, factors);
  }

  tls.distances.reinitialize(verts.size());
  const MutableSpan<float> distances = tls.distances;
  calc_brush_distances(
      ss, positions_eval, verts, eBrushFalloffShape(brush.falloff_shape), distances);
  filter_distances_with_radius(cache.radius, distances, factors);
  apply_hardness_to_distances(cache, distances);
  calc_brush_strength_factors(cache, brush, distances, factors);

  if (cache.automasking) {
    auto_mask::calc_vert_factors(object, *cache.automasking, node, verts, factors);
  }

  scale_factors(factors, strength * strength);

  calc_brush_texture_factors(ss, brush, positions_eval, verts, factors);

  filter_factors_on_face_sets(ss.vert_to_face_map, ss.face_sets, relax_face_sets, verts, factors);
}

BLI_NOINLINE static void calc_relaxed_positions_faces(const OffsetIndices<int> faces,
                                                      const Span<int> corner_verts,
                                                      const int *face_sets,
                                                      const GroupedSpan<int> vert_to_face_map,
                                                      const BitSpan boundary_verts,
                                                      const Span<bool> hide_poly,
                                                      const Span<int> verts,
                                                      const Span<float3> positions,
                                                      const Span<float3> normals,
                                                      const bool relax_face_sets,
                                                      LocalData &tls,
                                                      const Span<float> factors,
                                                      const MutableSpan<float3> new_positions)
{
  BLI_assert(verts.size() == factors.size());
  BLI_assert(verts.size() == new_positions.size());

  tls.vert_neighbors.reinitialize(verts.size());
  calc_vert_neighbors_interior(
      faces, corner_verts, vert_to_face_map, boundary_verts, hide_poly, verts, tls.vert_neighbors);
  const Span<Vector<int>> vert_neighbors = tls.vert_neighbors;

  for (const int i : verts.index_range()) {
    /* Don't modify corner vertices */
    if (vert_neighbors[i].size() <= 2) {
      new_positions[i] = positions[verts[i]];
      continue;
    }

    /* Smoothed position calculation */
    float3 smoothed_position;
    bool has_new_position = false;
    if (boundary_verts[verts[i]]) {
      has_new_position = get_average_position_boundary(vert_to_face_map,
                                                       face_sets,
                                                       boundary_verts,
                                                       positions,
                                                       relax_face_sets,
                                                       vert_neighbors[i],
                                                       smoothed_position);
    }
    else {
      has_new_position = get_average_position_interior(vert_to_face_map,
                                                       face_sets,
                                                       positions,
                                                       relax_face_sets,
                                                       vert_neighbors[i],
                                                       smoothed_position);
    }

    if (!has_new_position) {
      new_positions[i] = positions[verts[i]];
      continue;
    }

    /* Normal Calculation */
    float3 normal;
    if (boundary_verts[verts[i]]) {
      bool has_boundary_normal = get_normal_boundary(vert_to_face_map,
                                                     face_sets,
                                                     boundary_verts,
                                                     positions[verts[i]],
                                                     positions,
                                                     relax_face_sets,
                                                     vert_neighbors[i],
                                                     normal);

      if (!has_boundary_normal) {
        normal = normals[verts[i]];
      }
    }
    else {
      normal = normals[verts[i]];
    }

    if (math::is_zero(normal)) {
      new_positions[i] = positions[verts[i]];
      continue;
    }

    float4 plane;
    plane_from_point_normal_v3(plane, positions[verts[i]], normal);

    float3 smooth_closest_plane;
    closest_to_plane_v3(smooth_closest_plane, plane, smoothed_position);

    float3 displacement = smooth_closest_plane - positions[verts[i]];
    new_positions[i] = positions[verts[i]] + displacement * factors[i];
  }
}

BLI_NOINLINE static void apply_positions_faces(const Sculpt &sd,
                                               const Span<float3> positions_eval,
                                               const PBVHNode &node,
                                               Object &object,
                                               LocalData &tls,
                                               const Span<float3> new_positions,
                                               const MutableSpan<float3> positions_orig)
{
  const Span<int> verts = bke::pbvh::node_unique_verts(node);

  tls.translations.reinitialize(verts.size());
  const MutableSpan<float3> translations = tls.translations;
  translations_from_new_positions(new_positions, verts, positions_eval, translations);

  write_translations(sd, object, positions_eval, verts, translations, positions_orig);
}

static void do_relax_face_sets_brush_mesh(const Sculpt &sd,
                                          const Brush &brush,
                                          Object &object,
                                          const Span<PBVHNode *> nodes,
                                          const float strength,
                                          const float relax_face_sets)
{
  const SculptSession &ss = *object.sculpt;
  Mesh &mesh = *static_cast<Mesh *>(object.data);
  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();
  const bke::AttributeAccessor attributes = mesh.attributes();
  const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);

  const PBVH &pbvh = *ss.pbvh;

  const Span<float3> positions_eval = BKE_pbvh_get_vert_positions(pbvh);
  const Span<float3> vert_normals = BKE_pbvh_get_vert_normals(pbvh);
  MutableSpan<float3> positions_orig = mesh.vert_positions_for_write();

  Array<int> node_offset_data;
  const OffsetIndices<int> node_vert_offsets = create_node_vert_offsets(nodes, node_offset_data);

  Array<float3> new_positions(node_vert_offsets.total_size());
  Array<float> factors(node_vert_offsets.total_size());

  threading::EnumerableThreadSpecific<LocalData> all_tls;
  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    LocalData &tls = all_tls.local();
    for (const int i : range) {
      calc_factors_faces(brush,
                         positions_eval,
                         vert_normals,
                         *nodes[i],
                         strength,
                         relax_face_sets,
                         object,
                         tls,
                         factors.as_mutable_span().slice(node_vert_offsets[i]));
    }
  });

  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    LocalData &tls = all_tls.local();
    for (const int i : range) {
      calc_relaxed_positions_faces(faces,
                                   corner_verts,
                                   ss.face_sets,
                                   ss.vert_to_face_map,
                                   ss.vertex_info.boundary,
                                   hide_poly,
                                   bke::pbvh::node_unique_verts(*nodes[i]),
                                   positions_eval,
                                   vert_normals,
                                   relax_face_sets,
                                   tls,
                                   factors.as_span().slice(node_vert_offsets[i]),
                                   new_positions.as_mutable_span().slice(node_vert_offsets[i]));
    }
  });

  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    LocalData &tls = all_tls.local();
    for (const int i : range) {
      apply_positions_faces(sd,
                            positions_eval,
                            *nodes[i],
                            object,
                            tls,
                            new_positions.as_span().slice(node_vert_offsets[i]),
                            positions_orig);
    }
  });
}

}  // namespace relax_face_sets_cc

void do_relax_face_sets_brush(const Sculpt &sd, Object &object, Span<PBVHNode *> nodes)
{
  const Brush &brush = *BKE_paint_brush_for_read(&sd.paint);

  SCULPT_boundary_info_ensure(object);

  const SculptSession &ss = *object.sculpt;
  const std::array<float, 4> strengths = iteration_strengths(ss.cache->bstrength,
                                                             ss.cache->iteration_count);

  /* On every third step of the stroke, behave more similarly to the Topology Relax brush */
  const bool relax_face_sets = !(ss.cache->iteration_count % 3 == 0);

  for (const float strength : strengths) {
    switch (BKE_pbvh_type(*ss.pbvh)) {
      case PBVH_FACES:
        do_relax_face_sets_brush_mesh(sd, brush, object, nodes, strength, relax_face_sets);
        break;
      case PBVH_GRIDS:
      case PBVH_BMESH:
        break;
    }
  }
}
}  // namespace blender::ed::sculpt_paint
