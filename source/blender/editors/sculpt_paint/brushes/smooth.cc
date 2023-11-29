/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "editors/sculpt_paint/brushes/types.hh"

#include "DNA_brush_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_colortools.h"
#include "BKE_key.h"
#include "BKE_mesh.hh"
#include "BKE_paint.hh"
#include "BKE_pbvh.hh"

#include "BLI_array.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_task.h"
#include "BLI_task.hh"

#include "editors/sculpt_paint/sculpt_intern.hh"

namespace blender::ed::sculpt_paint {

inline namespace draw_cc {

namespace pbvh = bke::pbvh;

struct TLS {
  Vector<float> factors;
  Vector<float> distances;
  Vector<int> neighbor_offsets;
  Vector<int> neighbor_indices;
  Vector<float3> translations;
};

static Array<Vector<int>> calc_vert_neighbors(const OffsetIndices<int> faces,
                                              const Span<int> corner_verts,
                                              const GroupedSpan<int> vert_to_face,
                                              const BitSpan boundary_verts,
                                              const bool *hide_poly,
                                              const Span<int> verts)
{
  Array<Vector<int>> vert_neighbors(verts.size());
  for (const int i : verts.index_range()) {
    const int vert = verts[i];
    const bool is_boundary = boundary_verts[vert];
    if (boundary_verts[vert]) {
    }
    else {
    }
    int count = 0;
    for (const int face : vert_to_face[vert]) {
      if (hide_poly && hide_poly[face]) {
        /* Skip connectivity from hidden faces. */
        continue;
      }
      const int2 verts = bke::mesh::face_find_adjecent_verts(faces[face], corner_verts, vert);
      if (!is_boundary || boundary_verts[verts[0]]) {
        vert_neighbors[i].append(verts[0]);
      }
      if (!is_boundary || boundary_verts[verts[1]]) {
        vert_neighbors[i].append(verts[1]);
      }
    }
  }
  return vert_neighbors;
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

static void calc_smooth_positions_faces(const Sculpt &sd,
                                        const Brush &brush,
                                        Object &object,
                                        pbvh::mesh::Node &node,
                                        TLS &tls,
                                        const MutableSpan<float3> new_positions)
{
  SculptSession &ss = *object.sculpt;
  StrokeCache &cache = *ss.cache;
  Mesh &mesh = *static_cast<Mesh *>(object.data);

  pbvh::Tree pbvh_tree(*ss.pbvh);

  const Span<float3> positions_eval = pbvh_tree.vert_positions();
  const Span<float3> vert_normals = pbvh_tree.vert_normals();
  const Span<int> verts = node.unique_vert_indices();

  tls.factors.reinitialize(verts.size());
  const MutableSpan<float> factors = tls.factors;
  calc_mesh_hide_and_mask(mesh, verts, factors);

  tls.distances.reinitialize(verts.size());
  const MutableSpan<float> distances = tls.distances;
  calc_distance_falloff(
      ss, positions_eval, verts, eBrushFalloffShape(brush.falloff_shape), distances, factors);
  calc_brush_strength_factors(ss, brush, verts, distances, factors);

  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, vert_normals, verts, factors);
  }

  if (ss.cache->automasking) {
    calc_mesh_automask(object, *ss.cache->automasking, node, verts, factors);
  }

  calc_brush_texture_factors(ss, brush, positions_eval, verts, factors);

  const GroupedSpan<int> vert_neighbors = calc_vert_neighbors(ss.faces,
                                                              ss.corner_verts,
                                                              ss.pmap,
                                                              ss.vertex_info.boundary,
                                                              ss.hide_poly,
                                                              verts,
                                                              tls.neighbor_offsets,
                                                              tls.neighbor_indices);

  tls.translations.reinitialize(verts.size());
  const MutableSpan<float3> translations = tls.translations;

  for (const int i : verts.index_range()) {
    new_positions[i] = average_positions(positions_eval, vert_neighbors[i]);
  }

  // XXX: Maybe try not to tag verts with factor == 0.0f
  BKE_pbvh_vert_tag_update_normals(*ss.pbvh, verts);
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

  AutomaskingNodeData automask_data;
  SCULPT_automasking_node_begin(&object, ss.cache->automasking, &automask_data, &node);

  BKE_pbvh_vertex_iter_begin (ss.pbvh, &node, vd, PBVH_ITER_UNIQUE) {
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

    float avg[3], val[3];
    SCULPT_neighbor_coords_average_interior(&ss, avg, vd.vertex);
    sub_v3_v3v3(val, avg, vd.co);
    madd_v3_v3v3fl(val, vd.co, val, fade);
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

  AutomaskingNodeData automask_data;
  SCULPT_automasking_node_begin(&object, ss.cache->automasking, &automask_data, &node);

  BKE_pbvh_vertex_iter_begin (ss.pbvh, &node, vd, PBVH_ITER_UNIQUE) {
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

    float avg[3], val[3];
    SCULPT_neighbor_coords_average_interior(&ss, avg, vd.vertex);
    sub_v3_v3v3(val, avg, vd.co);
    madd_v3_v3v3fl(val, vd.co, val, fade);
    SCULPT_clip(sd, &ss, vd.co, val);
  }
  BKE_pbvh_vertex_iter_end;
}

}  // namespace draw_cc

void do_smooth_brush(const Sculpt &sd, Object &object, Span<PBVHNode *> nodes)
{
  SculptSession &ss = *object.sculpt;
  const Brush &brush = *BKE_paint_brush_for_read(&sd.paint);
  const float bstrength = ss.cache->bstrength;

  /* Offset with as much as possible factored in already. */
  float3 effective_normal;
  SCULPT_tilt_effective_normal_get(&ss, &brush, effective_normal);

  /* XXX: this shouldn't be necessary, but sculpting crashes in blender2.8 otherwise
   * initialize before threads so they can do curve mapping. */
  BKE_curvemapping_init(brush.curve);

  switch (BKE_pbvh_type(object.sculpt->pbvh)) {
    case PBVH_FACES: {
      Array<int> node_vert_offset_data(nodes.size() + 1);
      int offset = 0;
      for (const int i : nodes.index_range()) {
        node_vert_offset_data[i] = offset;
        offset += BKE_pbvh_node_get_unique_vert_indices(nodes[i]).size();
      }
      node_vert_offset_data.last() = offset;
      const OffsetIndices<int> node_vert_offsets(node_vert_offset_data);

      Array<float3> new_positions(node_vert_offsets.total_size());

      threading::EnumerableThreadSpecific<TLS> all_tls;
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        TLS &tls = all_tls.local();
        for (const int i : range) {
          pbvh::mesh::Node face_node(*nodes[i]);
          calc_smooth_positions_faces(sd,
                                      brush,
                                      object,
                                      face_node,
                                      tls,
                                      new_positions.as_mutable_span().slice(node_vert_offsets[i]));
        }
      });

      Mesh &mesh = *static_cast<Mesh *>(object.data);
      MutableSpan<float3> positions_sculpt = mesh_brush_positions_for_write(*object.sculpt, mesh);
      MutableSpan<float3> positions_mesh = mesh.vert_positions_for_write();
      // TODO: clip_and_lock_translations
      // TODO: apply_crazyspace_to_translations
      // TODO: apply_translations
      // TODO: flush_positions_to_shape_keys
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        TLS &tls = all_tls.local();
        for (const int i : range) {
          pbvh::mesh::Node face_node(*nodes[i]);
          calc_faces(sd,
                     brush,
                     object,
                     face_node,
                     tls,
                     new_positions.as_mutable_span().slice(node_vert_offsets[i]));
        }
      });

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
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          calc_bmesh(object, brush, *nodes[i]);
        }
      });
      break;
  }
}

}  // namespace blender::ed::sculpt_paint
