/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "editors/sculpt_paint/brushes/types.hh"

#include "DNA_brush_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_colortools.hh"
#include "BKE_key.hh"
#include "BKE_mesh.hh"
#include "BKE_paint.hh"
#include "BKE_pbvh.hh"

#include "BLI_array.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_task.h"
#include "BLI_task.hh"
#include "BLI_virtual_array.hh"

#include "editors/sculpt_paint/sculpt_intern.hh"
#include "mesh_brush_common.hh"

namespace blender::ed::sculpt_paint {

inline namespace smooth_cc {

struct LocalData {
  Vector<float> factors;
  Vector<float> distances;
  // Vector<int> neighbor_offsets;
  // Vector<int> neighbor_indices;
  Vector<Vector<int>> vert_neighbors;
  Vector<float3> translations;
};

struct MeshTopologyData {
  OffsetIndices<int> faces;
  Span<int> corner_verts;
  GroupedSpan<int> vert_to_face;
};

// template<typename FilterFn>
// void foreach_vert_neighbor_by_face(const OffsetIndices<int> faces,
//                                    const Span<int> corner_verts,
//                                    const GroupedSpan<int> vert_to_face,
//                                    const Span<bool> hide_poly,
//                                    const FilterFn &filter_fn,
//                                    const int vert)
// {
//   for (const int face : vert_to_face[vert]) {
//     if (!hide_poly.is_empty() && hide_poly[face]) {
//       /* Skip connectivity from hidden faces. */
//       continue;
//     }
//     const int2 verts = bke::mesh::face_find_adjacent_verts(faces[face], corner_verts, vert);
//     if (filter_fn[verts[0]]) {
//     }
//     if (!is_boundary || boundary_verts[verts[0]]) {
//       vert_neighbors[i].append(verts[0]);
//     }
//     if (!is_boundary || boundary_verts[verts[1]]) {
//       vert_neighbors[i].append(verts[1]);
//     }
//   }
// }

/* For boundary vertices, only include other boundary vertices. */
static void calc_vert_neighbors(const OffsetIndices<int> faces,
                                const Span<int> corner_verts,
                                const GroupedSpan<int> vert_to_face,
                                const BitSpan boundary_verts,
                                const Span<bool> hide_poly,
                                const Span<int> verts,
                                MutableSpan<Vector<int>> neighbors)
{
  for (Vector<int> &vector : neighbors) {
    vector.clear();
  }

  for (const int i : verts.index_range()) {
    const int vert = verts[i];
    const bool is_boundary = boundary_verts[vert];
    for (const int face : vert_to_face[vert]) {
      if (!hide_poly.is_empty() && hide_poly[face]) {
        /* Skip connectivity from hidden faces. */
        continue;
      }
      const int2 verts = bke::mesh::face_find_adjacent_verts(faces[face], corner_verts, vert);
      if (!is_boundary || boundary_verts[verts[0]]) {
        neighbors[i].append_non_duplicates(verts[0]);
      }
      if (!is_boundary || boundary_verts[verts[1]]) {
        neighbors[i].append_non_duplicates(verts[1]);
      }
    }
  }
}

static float3 average_positions(const Span<float3> positions, const Span<int> indices)
{
  const float factor = math::rcp(float(indices.size()));
  float3 result(0);
  for (const int i : indices) {
    result += positions[i] * factor;
  }
  return result;
}

static void calc_smooth_positions_faces(const OffsetIndices<int> faces,
                                        const Span<int> corner_verts,
                                        const GroupedSpan<int> vert_to_face_map,
                                        const BitSpan boundary_verts,
                                        const Span<bool> hide_poly,
                                        const Span<int> verts,
                                        LocalData &tls,
                                        const Span<float3> positions,
                                        const MutableSpan<float3> new_positions)
{
  tls.vert_neighbors.reinitialize(verts.size());
  calc_vert_neighbors(
      faces, corner_verts, vert_to_face_map, boundary_verts, hide_poly, verts, tls.vert_neighbors);
  const Span<Vector<int>> vert_neighbors = tls.vert_neighbors;

  for (const int i : verts.index_range()) {
    new_positions[i] = average_positions(positions, vert_neighbors[i]);
  }
}

static void apply_positions_faces(const Sculpt &sd,
                                  const Brush &brush,
                                  const PBVHNode &node,
                                  Object &object,
                                  LocalData &tls,
                                  const Span<float3> new_positions)
{
  SculptSession &ss = *object.sculpt;
  const StrokeCache &cache = *ss.cache;
  const Mesh &mesh = *static_cast<Mesh *>(object.data);

  const PBVH &pbvh = *ss.pbvh;

  tls.factors.reinitialize(verts.size());
  const MutableSpan<float> factors = tls.factors;
  calc_mesh_hide_and_mask(mesh, verts, factors);

  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, vert_normals, verts, factors);
  }

  tls.distances.reinitialize(verts.size());
  const MutableSpan<float> distances = tls.distances;
  calc_distance_falloff(
      ss, positions_eval, verts, eBrushFalloffShape(brush.falloff_shape), distances, factors);
  calc_brush_strength_factors(ss, brush, verts, distances, factors);

  if (ss.cache->automasking) {
    auto_mask::calc_vert_factors(object, *ss.cache->automasking, node, verts, factors);
  }

  calc_brush_texture_factors(ss, brush, positions_eval, verts, factors);

  // TODO: clip_and_lock_translations
  // TODO: apply_crazyspace_to_translations
  // TODO: apply_translations
  // TODO: flush_positions_to_shape_keys
}

static void do_smooth_brush_mesh(const Sculpt &sd,
                                 const Brush &brush,
                                 Object &object,
                                 Span<PBVHNode *> nodes)
{
  const SculptSession &ss = *object.sculpt;
  Mesh &mesh = *static_cast<Mesh *>(object.data);
  const bke::AttributeAccessor attributes = mesh.attributes();
  const VArraySpan hide_poly = *attributes.lookup<bool>(".hide_poly", bke::AttrDomain::Face);

  const PBVH &pbvh = *ss.pbvh;

  const Span<float3> positions_eval = BKE_pbvh_get_vert_positions(pbvh);
  const Span<float3> vert_normals = BKE_pbvh_get_vert_normals(pbvh);

  Array<int> node_vert_offset_data(nodes.size() + 1);
  for (const int i : nodes.index_range()) {
    node_vert_offset_data[i] = bke::pbvh::node_unique_verts(*nodes[i]).size();
  }
  const OffsetIndices<int> node_vert_offsets = offset_indices::accumulate_counts_to_offsets(
      node_vert_offset_data);

  Array<float3> new_positions(node_vert_offsets.total_size());

  threading::EnumerableThreadSpecific<LocalData> all_tls;
  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    LocalData &tls = all_tls.local();
    for (const int i : range) {
      calc_smooth_positions_faces(mesh.faces(),
                                  mesh.corner_verts(),
                                  ss.vert_to_face_map,
                                  ss.vertex_info.boundary,
                                  hide_poly,
                                  bke::pbvh::node_unique_verts(*nodes[i]),
                                  tls,
                                  positions_eval,
                                  new_positions.as_mutable_span().slice(node_vert_offsets[i]));
    }
  });

  MutableSpan<float3> positions_sculpt = mesh_brush_positions_for_write(*object.sculpt, mesh);
  MutableSpan<float3> positions_mesh = mesh.vert_positions_for_write();
  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    LocalData &tls = all_tls.local();
    for (const int i : range) {
      apply_positions_faces(sd,
                            brush,
                            *nodes[i],
                            object,
                            tls,
                            new_positions.as_mutable_span().slice(node_vert_offsets[i]));
    }
  });
}

static void calc_grids(Object &object, const Brush &brush, PBVHNode &node)
{
  SculptSession &ss = *object.sculpt;
  PBVHVertexIter vd;
  const MutableSpan<float3> proxy = BKE_pbvh_node_add_proxy(*ss.pbvh, node).co;

  SculptBrushTest test;
  SculptBrushTestFn sculpt_brush_test_sq_fn = SCULPT_brush_test_init_with_falloff_shape(
      &ss, &test, brush.falloff_shape);
  const int thread_id = BLI_task_parallel_thread_id(nullptr);

  auto_mask::NodeData automask_data = auto_mask::node_begin(
      object, ss.cache->automasking.get(), node);

  BKE_pbvh_vertex_iter_begin (*ss.pbvh, &node, vd, PBVH_ITER_UNIQUE) {
    if (!sculpt_brush_test_sq_fn(&test, vd.co)) {
      continue;
    }

    const float fade = bstrength * SCULPT_brush_strength_factor(&ss,
                                                                &brush,
                                                                vd.co,
                                                                sqrtf(test.dist),
                                                                vd.no,
                                                                vd.fno,
                                                                vd.mask,
                                                                vd.vertex,
                                                                thread_id,
                                                                &automask_data);

    float3 avg;
    smooth::neighbor_coords_average_interior(&ss, avg, vd.vertex);
    float3 final = float3(vd.co) + (avg - float3(vd.co)) * fade;
    SCULPT_clip(sd, &ss, vd.co, val);
  }
  BKE_pbvh_vertex_iter_end;
}

static void calc_bmesh(Object &object, const Brush &brush, PBVHNode &node)
{
  SculptSession &ss = *object.sculpt;
  PBVHVertexIter vd;
  const MutableSpan<float3> proxy = BKE_pbvh_node_add_proxy(*ss.pbvh, node).co;

  SculptBrushTest test;
  SculptBrushTestFn sculpt_brush_test_sq_fn = SCULPT_brush_test_init_with_falloff_shape(
      &ss, &test, brush.falloff_shape);
  const int thread_id = BLI_task_parallel_thread_id(nullptr);

  auto_mask::NodeData automask_data = auto_mask::node_begin(
      object, ss.cache->automasking.get(), node);

  BKE_pbvh_vertex_iter_begin (*ss.pbvh, &node, vd, PBVH_ITER_UNIQUE) {
    if (!sculpt_brush_test_sq_fn(&test, vd.co)) {
      continue;
    }

    const float fade = bstrength * SCULPT_brush_strength_factor(&ss,
                                                                &brush,
                                                                vd.co,
                                                                sqrtf(test.dist),
                                                                vd.no,
                                                                vd.fno,
                                                                vd.mask,
                                                                vd.vertex,
                                                                thread_id,
                                                                &automask_data);

    float3 avg;
    smooth::neighbor_coords_average_interior(&ss, avg, vd.vertex);
    float3 final = float3(vd.co) + (avg - float3(vd.co)) * fade;
    SCULPT_clip(sd, &ss, vd.co, final);
  }
  BKE_pbvh_vertex_iter_end;
}

}  // namespace smooth_cc

void do_smooth_brush(const Sculpt &sd, Object &object, const Span<PBVHNode *> nodes)
{
  SculptSession &ss = *object.sculpt;
  const Brush &brush = *BKE_paint_brush_for_read(&sd.paint);
  const float bstrength = ss.cache->bstrength;

  const int max_iterations = 4;
  const float fract = 1.0f / max_iterations;

  CLAMP(bstrength, 0.0f, 1.0f);

  const int count = int(bstrength * max_iterations);
  const float last = max_iterations * (bstrength - count * fract);

  SCULPT_boundary_info_ensure(&object);

  switch (BKE_pbvh_type(*object.sculpt->pbvh)) {
    case PBVH_FACES: {
      do_smooth_brush_mesh(sd, brush, object, nodes);
      break;
    }
    case PBVH_GRIDS:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          calc_grids(object, brush, *nodes[i]);
        }
      });
      break;
    case PBVH_BMESH:
      BM_mesh_elem_index_ensure(ss.bm, BM_VERT);
      BM_mesh_elem_table_ensure(ss.bm, BM_VERT);
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          calc_bmesh(object, brush, *nodes[i]);
        }
      });
      break;
  }
}

}  // namespace blender::ed::sculpt_paint
