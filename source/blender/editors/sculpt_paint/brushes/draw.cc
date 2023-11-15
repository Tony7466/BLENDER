/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "editors/sculpt_paint/brushes/types.hh"

#include "DNA_brush_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_colortools.h"
#include "BKE_mesh.hh"
#include "BKE_paint.hh"
#include "BKE_pbvh.h"

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
  Vector<float3> translations;
};

static void calc_faces(const Sculpt &sd,
                       const Brush &brush,
                       const float3 &offset,
                       Object &object,
                       pbvh::mesh::Node &node,
                       TLS &tls,
                       MutableSpan<float3> positions_orig)
{
  SculptSession &ss = *object.sculpt;
  StrokeCache &cache = *ss.cache;
  Mesh &mesh = *static_cast<Mesh *>(object.data);

  pbvh::Tree pbvh_tree(*ss.pbvh);

  const Span<float3> positions = pbvh_tree.vert_positions();
  const Span<float3> vert_normals = pbvh_tree.vert_normals();
  const Span<int> verts = node.unique_vert_indices();

  tls.factors.reinitialize(verts.size());
  const MutableSpan<float> factors = tls.factors;
  calc_mesh_hide_and_mask(mesh, verts, factors);

  tls.distances.reinitialize(verts.size());
  const MutableSpan<float> distances = tls.distances;
  calc_distance_falloff(
      ss, positions, verts, eBrushFalloffShape(brush.falloff_shape), distances, factors);
  calc_brush_strength_factors(ss, brush, verts, distances, factors);

  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, vert_normals, verts, factors);
  }

  if (ss.cache->automasking) {
    calc_mesh_automask(object, *ss.cache->automasking, node, verts, factors);
  }

  calc_brush_texture_factors(ss, brush, positions, verts, factors);

  tls.translations.reinitialize(verts.size());
  const MutableSpan<float3> translations = tls.translations;
  for (const int i : verts.index_range()) {
    translations[i] = offset * factors[i];
  }

  clip_and_lock_translations(sd, ss, positions, verts, translations);

  if (ss.deform_modifiers_active) {
    apply_crazyspace_translations(
        translations,
        {reinterpret_cast<const float3x3 *>(ss.deform_imats), positions.size()},
        verts,
        positions_orig);
  }
  else {
    apply_translations(translations, verts, positions_orig);
  }

  // XXX: Maybe try not to tag verts with factor == 0.0f
  BKE_pbvh_vert_tag_update_normals(*ss.pbvh, verts);
}

static void calc_grids(Object &object, const Brush &brush, const float3 &offset, PBVHNode &node)
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

    const float fade = SCULPT_brush_strength_factor(&ss,
                                                    &brush,
                                                    vd.co,
                                                    math::sqrt(test.dist),
                                                    vd.no,
                                                    vd.fno,
                                                    vd.mask,
                                                    vd.vertex,
                                                    thread_id,
                                                    &automask_data);
    proxy[vd.i] = offset * fade;
  }
  BKE_pbvh_vertex_iter_end;
}

static void calc_bmesh(Object &object, const Brush &brush, const float3 &offset, PBVHNode &node)
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

    /* Offset vertex. */
    const float fade = SCULPT_brush_strength_factor(&ss,
                                                    &brush,
                                                    vd.co,
                                                    math::sqrt(test.dist),
                                                    vd.no,
                                                    vd.fno,
                                                    vd.mask,
                                                    vd.vertex,
                                                    thread_id,
                                                    &automask_data);
    proxy[vd.i] = offset * fade;
  }
  BKE_pbvh_vertex_iter_end;
}

}  // namespace draw_cc

void do_draw_brush(const Sculpt &sd, Object &object, Span<PBVHNode *> nodes)
{
  SculptSession &ss = *object.sculpt;
  const Brush &brush = *BKE_paint_brush_for_read(&sd.paint);
  const float bstrength = ss.cache->bstrength;

  /* Offset with as much as possible factored in already. */
  float3 effective_normal;
  SCULPT_tilt_effective_normal_get(&ss, &brush, effective_normal);

  const float3 offset = effective_normal * ss.cache->radius * ss.cache->scale * bstrength;

  /* XXX: this shouldn't be necessary, but sculpting crashes in blender2.8 otherwise
   * initialize before threads so they can do curve mapping. */
  BKE_curvemapping_init(brush.curve);

  switch (BKE_pbvh_type(object.sculpt->pbvh)) {
    case PBVH_FACES: {
      threading::EnumerableThreadSpecific<TLS> all_tls;
      Mesh &mesh = *static_cast<Mesh *>(object.data);
      MutableSpan<float3> positions_orig = mesh.vert_positions_for_write();
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        TLS &tls = all_tls.local();
        for (const int i : range) {
          pbvh::mesh::Node face_node(*nodes[i]);
          calc_faces(sd, brush, offset, object, face_node, tls, positions_orig);
        }
      });
      break;
    }
    case PBVH_GRIDS:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          calc_grids(object, brush, offset, *nodes[i]);
        }
      });
      break;
    case PBVH_BMESH:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          calc_bmesh(object, brush, offset, *nodes[i]);
        }
      });
      break;
  }
}

}  // namespace blender::ed::sculpt_paint
