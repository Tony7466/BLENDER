/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "editors/sculpt_paint/brushes/draw_vector_displacement.hh"

#include "DNA_brush_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_colortools.h"
#include "BKE_paint.hh"
#include "BKE_pbvh.h"

#include "BLI_array.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_math_vector.hh"
#include "BLI_task.h"
#include "BLI_task.hh"

#include "editors/sculpt_paint/sculpt_intern.hh"

namespace blender::ed::sculpt_paint {

/* The important part is to place the TLS into its own namespace. Otherwise its definition
 * silently conflicts with defintions from other brushes. Without this TLS from other modules might
 * actually be the wrong type. */
namespace {

namespace pbvh = bke::pbvh;

struct TLS {
  Vector<float> factors;
  Vector<float> distances;
  Vector<float4> colors;
};

static void calc_faces(Object &object, const Brush &brush, pbvh::mesh::Node &node, TLS &tls)
{
  SculptSession &ss = *object.sculpt;
  StrokeCache &cache = *ss.cache;
  const Mesh &mesh = *static_cast<const Mesh *>(object.data);

  pbvh::Tree pbvh_tree(*ss.pbvh);

  const Span<float3> vert_positions = pbvh_tree.vert_positions();
  const Span<float3> vert_normals = pbvh_tree.vert_normals();
  const Span<int> vert_indices = node.unique_vert_indices();

  /* XXX: Possibly use full-masking and hide status and distance to build a smaller array of vert
   * indices for later steps in the algorithm, to avoid the need to check for fade[i] == 0.0f
   * later. But this might not be worth it if the nodes are small enough. In that case calculating
   * a value for every vertex in a node would be fine, as long as we can filter out unnecessary
   * nodes at a higher level. */

  tls.factors.reinitialize(vert_indices.size());
  const MutableSpan<float> factors = tls.factors;
  calc_mesh_hide_and_mask(mesh, vert_indices, factors);

  tls.distances.reinitialize(vert_indices.size());
  const MutableSpan<float> distances = tls.distances;
  calc_distance_falloff(ss,
                        vert_positions,
                        vert_indices,
                        eBrushFalloffShape(brush.falloff_shape),
                        distances,
                        factors);
  calc_brush_strength_factors(ss, brush, vert_indices, distances, factors);

  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, vert_normals, vert_indices, factors);
  }

  if (ss.cache->automasking) {
    calc_mesh_automask(object, *ss.cache->automasking, node, vert_indices, factors);
  }

  tls.colors.reinitialize(vert_indices.size());
  const MutableSpan<float4> colors = tls.colors;
  calc_brush_texture_colors(ss, brush, vert_positions, vert_indices, factors, colors);

  const MutableSpan<float3> proxy = node.add_proxy(pbvh_tree);
  for (const int i : vert_indices.index_range()) {
    SCULPT_calc_vertex_displacement(&ss, &brush, colors[i], proxy[i]);
    BKE_pbvh_vert_tag_update_normal(*ss.pbvh, vert_indices[i]);
  }
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

    SCULPT_automasking_node_update(&automask_data, &vd);

    float r_rgba[4];
    SCULPT_brush_strength_color(&ss,
                                &brush,
                                vd.co,
                                sqrtf(test.dist),
                                vd.no,
                                vd.fno,
                                vd.mask,
                                vd.vertex,
                                thread_id,
                                &automask_data,
                                r_rgba);
    SCULPT_calc_vertex_displacement(&ss, &brush, r_rgba, proxy[vd.i]);
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

    SCULPT_automasking_node_update(&automask_data, &vd);

    float r_rgba[4];
    SCULPT_brush_strength_color(&ss,
                                &brush,
                                vd.co,
                                sqrtf(test.dist),
                                vd.no,
                                vd.fno,
                                vd.mask,
                                vd.vertex,
                                thread_id,
                                &automask_data,
                                r_rgba);
    SCULPT_calc_vertex_displacement(&ss, &brush, r_rgba, proxy[vd.i]);
  }
  BKE_pbvh_vertex_iter_end;
}

}  // namespace

void do_draw_vector_displacement_brush(Sculpt &sd, Object &object, Span<PBVHNode *> nodes)
{
  Brush &brush = *BKE_paint_brush(&sd.paint);

  /* XXX: this shouldn't be necessary, but sculpting crashes in blender2.8 otherwise
   * initialize before threads so they can do curve mapping. */
  BKE_curvemapping_init(brush.curve);

  threading::EnumerableThreadSpecific<TLS> all_tls;
  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    TLS &tls = all_tls.local();
    for (const int i : range) {
      switch (BKE_pbvh_type(object.sculpt->pbvh)) {
        case PBVH_FACES: {
          pbvh::mesh::Node face_node(*nodes[i]);
          calc_faces(object, brush, face_node, tls);
          break;
        }
        case PBVH_GRIDS:
          calc_grids(object, brush, *nodes[i]);
          break;
        case PBVH_BMESH:
          calc_bmesh(object, brush, *nodes[i]);
          break;
      }
    }
  });
}

}  // namespace blender::ed::sculpt_paint
