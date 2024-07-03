
/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "editors/sculpt_paint/brushes/types.hh"
#include "editors/sculpt_paint/mesh_brush_common.hh"

#include "DNA_brush_types.h"

#include "BKE_mesh.hh"
#include "BKE_pbvh.hh"
#include "BKE_subdiv_ccg.hh"

#include "BLI_array_utils.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_math_base.hh"
#include "BLI_task.h"
#include "BLI_task.hh"

#include "editors/sculpt_paint/sculpt_intern.hh"

namespace blender::ed::sculpt_paint {
inline namespace draw_face_sets_cc {

constexpr float FACE_SET_BRUSH_MIN_FADE = 0.05f;

struct MeshLocalData {
  Vector<int> face_indices;
  Vector<float3> positions;
  Vector<float3> normals;
  Vector<float> factors;
  Vector<float> distances;
};

static void calc_face_centers(Mesh &mesh,
                              const Span<float3> positions_eval,
                              const Span<int> face_indices,
                              const MutableSpan<float3> positions)
{
  BLI_assert(face_indices.size() == positions.size());

  const OffsetIndices<int> faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();

  for (const int i : face_indices.index_range()) {
    positions[i] = bke::mesh::face_center_calc(positions_eval,
                                               corner_verts.slice(faces[face_indices[i]]));
  }
}

static void calc_face_normals(Mesh &mesh,
                              const Span<float3> positions_eval,
                              const Span<int> face_indices,
                              const MutableSpan<float3> normals)
{
  BLI_assert(face_indices.size() == normals.size());

  const OffsetIndices<int> faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();

  for (const int i : face_indices.index_range()) {
    normals[i] = bke::mesh::face_normal_calc(positions_eval,
                                             corner_verts.slice(faces[face_indices[i]]));
  }
}

BLI_NOINLINE static void fill_factor_from_hide_and_mask(const Mesh &mesh,
                                                        const Span<int> face_indices,
                                                        const MutableSpan<float> r_factors)
{
  BLI_assert(face_indices.size() == r_factors.size());

  const OffsetIndices<int> faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();

  /* TODO: Avoid overhead of accessing attributes for every PBVH node. */
  const bke::AttributeAccessor attributes = mesh.attributes();
  if (const VArray mask = *attributes.lookup<float>(".sculpt_mask", bke::AttrDomain::Point)) {
    const VArraySpan span(mask);
    for (const int i : face_indices.index_range()) {
      const Span<int> face_verts = corner_verts.slice(faces[face_indices[i]]);
      const float inv_size = math::rcp(float(face_verts.size()));
      float sum = 0.0f;
      for (const int vert : face_verts) {
        sum += span[vert];
      }
      r_factors[i] = 1.0f - sum * inv_size;
    }
  }
  else {
    r_factors.fill(1.0f);
  }

  if (const VArray hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Point)) {
    const VArraySpan span(hide_poly);
    for (const int i : face_indices.index_range()) {
      if (span[face_indices[i]]) {
        r_factors[i] = 0.0f;
      }
    }
  }
}

BLI_NOINLINE static void apply_face_set(const int face_set_id,
                                        const Span<int> face_indices,
                                        const Span<float> factors,
                                        const MutableSpan<int> face_sets)
{
  BLI_assert(face_indices.size() == factors.size());

  for (const int i : face_indices.index_range()) {
    if (factors[i] > FACE_SET_BRUSH_MIN_FADE) {
      face_sets[face_indices[i]] = face_set_id;
    }
  }
}

static void calc_faces(Object &object,
                       const Brush &brush,
                       const float strength,
                       const int face_set_id,
                       const Span<float3> face_centers,
                       const Span<float3> face_normals,
                       const PBVHNode &node,
                       MeshLocalData &tls,
                       const MutableSpan<int> face_sets)
{
  SculptSession &ss = *object.sculpt;
  const StrokeCache &cache = *ss.cache;
  Mesh &mesh = *static_cast<Mesh *>(object.data);

  const Span<int> face_indices = tls.face_indices;

  tls.factors.reinitialize(face_indices.size());
  const MutableSpan<float> factors = tls.factors;

  fill_factor_from_hide_and_mask(mesh, face_indices, factors);

  filter_region_clip_factors(ss, face_centers, factors);
  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, face_normals, factors);
  }

  tls.distances.reinitialize(face_indices.size());
  const MutableSpan<float> distances = tls.distances;
  calc_distance_falloff(
      ss, face_centers, eBrushFalloffShape(brush.falloff_shape), distances, factors);
  apply_hardness_to_distances(cache, distances);
  calc_brush_strength_factors(cache, brush, distances, factors);

  if (cache.automasking) {
    const OffsetIndices<int> faces = mesh.faces();
    const Span<int> corner_verts = mesh.corner_verts();
    auto_mask::calc_face_factors(
        object, faces, corner_verts, *cache.automasking, node, face_indices, factors);
  }

  calc_brush_texture_factors(ss, brush, face_centers, factors);
  scale_factors(factors, strength);

  apply_face_set(face_set_id, face_indices, factors, face_sets);
}

static void do_draw_face_sets_brush_mesh(Object &object,
                                         const Brush &brush,
                                         const Span<PBVHNode *> nodes)
{
  const SculptSession &ss = *object.sculpt;
  const PBVH &pbvh = *ss.pbvh;
  const Span<float3> positions_eval = BKE_pbvh_get_vert_positions(pbvh);

  Mesh &mesh = *static_cast<Mesh *>(object.data);
  const Span<int> corner_tris = mesh.corner_tri_faces();
  threading::EnumerableThreadSpecific<MeshLocalData> all_tls;

  bke::SpanAttributeWriter<int> attribute = face_set::ensure_face_sets_mesh(object);
  MutableSpan<int> face_sets = attribute.span;

  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    MeshLocalData &tls = all_tls.local();
    for (const int i : range) {
      const Span<int> face_indices = bke::pbvh::node_face_indices_calc_mesh(
          corner_tris, *nodes[i], tls.face_indices);

      tls.positions.reinitialize(face_indices.size());
      calc_face_centers(mesh, positions_eval, face_indices, tls.positions);

      tls.normals.reinitialize(face_indices.size());
      calc_face_normals(mesh, positions_eval, face_indices, tls.normals);

      undo::push_node(object, nodes[i], undo::Type::FaceSet);

      calc_faces(object,
                 brush,
                 ss.cache->bstrength,
                 ss.cache->paint_face_set,
                 tls.positions,
                 tls.normals,
                 *nodes[i],
                 tls,
                 face_sets);
    }
  });

  attribute.finish();
}

struct GridLocalData {
  Vector<int> face_indices;
  Vector<float3> positions;
  Vector<float> factors;
  Vector<float> distances;
};

BLI_NOINLINE static void calc_face_indices_grids(const SubdivCCG &subdiv_ccg,
                                                 const Span<int> grids,
                                                 const MutableSpan<int> face_indices)
{
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);
  BLI_assert(grids.size() * key.grid_area == face_indices.size());

  for (const int i : grids.index_range()) {
    const int start = i * key.grid_area;
    for (const int offset : IndexRange(key.grid_area)) {
      face_indices[start + offset] = BKE_subdiv_ccg_grid_to_face_index(subdiv_ccg, grids[i]);
    }
  }
}

static void calc_grids(Object &object,
                       const Brush &brush,
                       const float strength,
                       const int face_set_id,
                       const PBVHNode &node,
                       GridLocalData &tls,
                       const MutableSpan<int> face_sets)
{
  SculptSession &ss = *object.sculpt;
  const StrokeCache &cache = *ss.cache;
  SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const CCGKey key = BKE_subdiv_ccg_key_top_level(subdiv_ccg);

  const Span<int> grids = bke::pbvh::node_grid_indices(node);
  const int grid_verts_num = grids.size() * key.grid_area;

  tls.positions.reinitialize(grid_verts_num);
  MutableSpan<float3> positions = tls.positions;
  gather_grids_positions(subdiv_ccg, grids, positions);

  tls.factors.reinitialize(grid_verts_num);
  const MutableSpan<float> factors = tls.factors;
  blender::ed::sculpt_paint::fill_factor_from_hide_and_mask(subdiv_ccg, grids, factors);
  filter_region_clip_factors(ss, positions, factors);
  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, subdiv_ccg, grids, factors);
  }

  tls.distances.reinitialize(grid_verts_num);
  const MutableSpan<float> distances = tls.distances;
  calc_distance_falloff(
      ss, positions, eBrushFalloffShape(brush.falloff_shape), distances, factors);
  apply_hardness_to_distances(cache, distances);
  calc_brush_strength_factors(cache, brush, distances, factors);

  if (cache.automasking) {
    auto_mask::calc_grids_factors(object, *cache.automasking, node, grids, factors);
  }

  calc_brush_texture_factors(ss, brush, positions, factors);
  scale_factors(factors, strength);

  tls.face_indices.reinitialize(grid_verts_num);
  MutableSpan<int> face_indices = tls.face_indices;

  calc_face_indices_grids(subdiv_ccg, grids, face_indices);
  apply_face_set(face_set_id, face_indices, factors, face_sets);
}

static void do_draw_face_sets_brush_grids(Object &object,
                                          const Brush &brush,
                                          const Span<PBVHNode *> nodes)
{
  SculptSession &ss = *object.sculpt;

  bke::SpanAttributeWriter<int> attribute = face_set::ensure_face_sets_mesh(object);
  MutableSpan<int> face_sets = attribute.span;
  threading::EnumerableThreadSpecific<GridLocalData> all_tls;

  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    GridLocalData &tls = all_tls.local();
    for (PBVHNode *node : nodes.slice(range)) {
      for (const int i : range) {
        undo::push_node(object, node, undo::Type::FaceSet);

        calc_grids(object,
                   brush,
                   ss.cache->bstrength,
                   ss.cache->paint_face_set,
                   *nodes[i],
                   tls,
                   face_sets);
      }
    }
  });
  attribute.finish();
}
struct BMeshLocalData {
  Vector<float3> positions;
  Vector<float> masks;
  Vector<float> factors;
  Vector<float> distances;
};

BLI_NOINLINE static void fill_factor_from_hide_and_mask(const Set<BMFace *, 0L> &faces,
                                                        const Span<float> masks,
                                                        const MutableSpan<float> r_factors)
{
  BLI_assert(faces.size() == masks.size());
  BLI_assert(faces.size() == r_factors.size());

  for (const int i : masks.index_range()) {
    r_factors[i] = 1.0f - masks[i];
  }

  int i = 0;
  for (const BMFace *face : faces) {
    if (BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      r_factors[i] = 0.0f;
    }
    i++;
  }
}

static void generate_face_data_bmesh(const BMesh &bm,
                                     const Set<BMFace *, 0L> &node_faces,
                                     const MutableSpan<float3> positions,
                                     const MutableSpan<float> masks)
{
  BLI_assert(node_faces.size() == positions.size());
  BLI_assert(node_faces.size() == masks.size());

  int i = 0;
  for (const BMFace *f : node_faces) {
    float3 face_center;
    BM_face_calc_center_median(f, face_center);

    positions[i] = face_center;
    i++;
  }

  /* TODO: Avoid overhead of accessing attributes for every PBVH node. */
  const int mask_offset = CustomData_get_offset_named(&bm.vdata, CD_PROP_FLOAT, ".sculpt_mask");
  if (mask_offset == -1) {
    masks.fill(1.0f);
  }
  else {
    i = 0;
    for (BMFace *f : node_faces) {
      const BMLoop *l_iter = f->l_first = BM_FACE_FIRST_LOOP(f);
      int total_verts = 0;
      float sum;
      do {
        BMVert *vert = l_iter->v;
        sum += BM_ELEM_CD_GET_FLOAT(vert, mask_offset);
        total_verts++;
      } while ((l_iter = l_iter->next) != f->l_first);
      masks[i] = sum * math::rcp(float(total_verts));
      i++;
    }
  }
}

BLI_NOINLINE static void apply_face_set(const int face_set_id,
                                        const Set<BMFace *, 0> &faces,
                                        const MutableSpan<float> factors,
                                        const int cd_offset)
{
  int i = 0;
  for (BMFace *face : faces) {
    if (factors[i] > FACE_SET_BRUSH_MIN_FADE) {
      int &fset = *static_cast<int *>(POINTER_OFFSET(face->head.data, cd_offset));
      fset = face_set_id;
    }
    i++;
  }
}

static void calc_bmesh(Object &object,
                       const Brush &brush,
                       const float strength,
                       const int face_set_id,
                       PBVHNode &node,
                       BMeshLocalData &tls,
                       const int cd_offset)
{
  SculptSession &ss = *object.sculpt;
  const StrokeCache &cache = *ss.cache;

  const Set<BMFace *, 0> &faces = BKE_pbvh_bmesh_node_faces(&node);

  const Span<float3> positions = tls.positions;
  tls.factors.reinitialize(faces.size());
  const MutableSpan<float> factors = tls.factors;
  fill_factor_from_hide_and_mask(faces, tls.masks, factors);
  filter_region_clip_factors(ss, positions, factors);
  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, faces, factors);
  }

  tls.distances.reinitialize(faces.size());
  const MutableSpan<float> distances = tls.distances;
  calc_distance_falloff(
      ss, positions, eBrushFalloffShape(brush.falloff_shape), distances, factors);
  apply_hardness_to_distances(cache, distances);
  calc_brush_strength_factors(cache, brush, distances, factors);

  /* Disable auto-masking code path which rely on an undo step to access original data.
   *
   * This is because the dynamic topology uses BMesh Log based undo system, which creates a
   * single node for the undo step, and its type could be different for the needs of the
   * brush undo and the original data access.
   *
   * For the brushes like Draw the ss.cache->automasking is set to nullptr at the first step
   * of the brush, as there is an explicit check there for the brushes which support dynamic
   * topology. Do it locally here for the Draw Face Set brush here, to mimic the behavior of
   * the other brushes but without marking the brush as supporting dynamic topology. */
  auto_mask::node_begin(object, nullptr, node);

  calc_brush_texture_factors(ss, brush, positions, factors);
  scale_factors(factors, strength);

  apply_face_set(face_set_id, faces, factors, cd_offset);
}

static void do_draw_face_sets_brush_bmesh(Object &object,
                                          const Brush &brush,
                                          const Span<PBVHNode *> nodes)
{
  SculptSession &ss = *object.sculpt;
  const int cd_offset = face_set::ensure_face_sets_bmesh(object);
  threading::EnumerableThreadSpecific<BMeshLocalData> all_tls;
  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    BMeshLocalData &tls = all_tls.local();
    for (const int i : range) {
      Set<BMFace *, 0L> node_faces = BKE_pbvh_bmesh_node_faces(nodes[i]);
      tls.positions.reinitialize(node_faces.size());
      tls.masks.reinitialize(node_faces.size());

      undo::push_node(object, nodes[i], undo::Type::FaceSet);

      generate_face_data_bmesh(*ss.bm, node_faces, tls.positions, tls.masks);

      calc_bmesh(
          object, brush, ss.cache->bstrength, ss.cache->paint_face_set, *nodes[i], tls, cd_offset);
    }
  });
}

}  // namespace draw_face_sets_cc

void do_draw_face_sets_brush(const Sculpt &sd, Object &object, Span<PBVHNode *> nodes)
{
  SculptSession &ss = *object.sculpt;
  const Brush &brush = *BKE_paint_brush_for_read(&sd.paint);

  switch (BKE_pbvh_type(*ss.pbvh)) {
    case PBVH_FACES:
      do_draw_face_sets_brush_mesh(object, brush, nodes);
      break;
    case PBVH_GRIDS:
      do_draw_face_sets_brush_grids(object, brush, nodes);
      break;
    case PBVH_BMESH:
      do_draw_face_sets_brush_bmesh(object, brush, nodes);
      break;
  }
}
}  // namespace blender::ed::sculpt_paint
