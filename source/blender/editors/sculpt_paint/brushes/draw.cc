/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "editors/sculpt_paint/brushes/draw.hh"

#include "DNA_brush_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_attribute.hh"
#include "BKE_colortools.h"
#include "BKE_customdata.h"
#include "BKE_paint.hh"
#include "BKE_pbvh.h"

#include "BLI_array.hh"
#include "BLI_math_vector.hh"
#include "BLI_task.h"
#include "BLI_task.hh"

#include "editors/sculpt_paint/sculpt_intern.hh"

namespace blender::ed::sculpt_paint {

namespace pbvh = bke::pbvh;

static void calc_faces_old(Object &object,
                           const Brush &brush,
                           const float3 &offset,
                           PBVHNode &node)
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
    if (ss.cache->brush->flag2 & BRUSH_USE_COLOR_AS_DISPLACEMENT &&
        (brush.mtex.brush_map_mode == MTEX_MAP_MODE_AREA))
    {
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
    else {
      float fade = SCULPT_brush_strength_factor(&ss,
                                                &brush,
                                                vd.co,
                                                sqrtf(test.dist),
                                                vd.no,
                                                vd.fno,
                                                vd.mask,
                                                vd.vertex,
                                                thread_id,
                                                &automask_data);
      proxy[vd.i] = offset * fade;
    }

    if (vd.is_mesh) {
      BKE_pbvh_vert_tag_update_normal(ss.pbvh, vd.vertex);
    }
  }
  BKE_pbvh_vertex_iter_end;
}

static void init_fade_from_hide_and_mask(const Mesh &mesh,
                                         const Span<int> vert_indices,
                                         const MutableSpan<float> fade)
{
  const float *mask = static_cast<const float *>(
      CustomData_get_layer(&mesh.vert_data, CD_PAINT_MASK));

  bke::AttributeAccessor attributes = mesh.attributes();
  const VArray<bool> hide_vert = *attributes.lookup_or_default<bool>(
      ".hide_vert", ATTR_DOMAIN_POINT, false);

  for (const int i : vert_indices.index_range()) {
    const int vert_index = vert_indices[i];
    fade[i] = hide_vert[vert_index] ? 0.0f : 1.0f;
    if (mask) {
      fade[i] *= (1.0f - mask[vert_index]);
    }
  }
}

static void calc_faces_new(Object &object,
                           const Brush &brush,
                           const float3 &offset,
                           pbvh::FaceNode &node)
{
  SculptSession &ss = *object.sculpt;
  const Mesh &mesh = *static_cast<const Mesh *>(object.data);

  pbvh::Tree pbvh_tree(*ss.pbvh);

  const Span<float3> vert_positions = pbvh_tree.get_vert_positions();
  const Span<float3> vert_normals = pbvh_tree.get_vert_normals();
  const Span<int> vert_indices = node.get_unique_vert_indices();

  Array<float> fade(vert_indices.size());
  init_fade_from_hide_and_mask(mesh, vert_indices, fade);

  Array<float> brush_distance(vert_indices.size());
  calc_brush_falloff_and_distance(
      ss, vert_positions, vert_indices, brush.falloff_shape, fade, brush_distance);
  calc_brush_strength_factor(ss, brush, vert_positions, vert_indices, brush_distance, fade);
  calc_brush_front_face(ss, brush, vert_normals, vert_indices, fade);

  if (ss.cache->automasking) {
    calc_mesh_automask(object, *ss.cache->automasking, node, vert_indices, fade);
  }

  const MutableSpan<float3> proxy = node.add_proxy(pbvh_tree);
  threading::parallel_for(vert_indices.index_range(), 1024, [&](const IndexRange range) {
    for (const int i : range) {
      proxy[i] = offset * fade[i];
      BKE_pbvh_vert_tag_update_normal(*ss.pbvh, vert_indices[i]);
    }
  });
}

static void calc_faces(Object &object, const Brush &brush, const float3 &offset, PBVHNode &node)
{
  if (true) {
    pbvh::FaceNode face_node(node);
    calc_faces_new(object, brush, offset, face_node);
  }
  calc_faces_old(object, brush, offset, node);
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

    /* Offset vertex. */
    if (ss.cache->brush->flag2 & BRUSH_USE_COLOR_AS_DISPLACEMENT &&
        (brush.mtex.brush_map_mode == MTEX_MAP_MODE_AREA))
    {
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
    else {
      float fade = SCULPT_brush_strength_factor(&ss,
                                                &brush,
                                                vd.co,
                                                sqrtf(test.dist),
                                                vd.no,
                                                vd.fno,
                                                vd.mask,
                                                vd.vertex,
                                                thread_id,
                                                &automask_data);
      proxy[vd.i] = offset * fade;
    }

    if (vd.is_mesh) {
      BKE_pbvh_vert_tag_update_normal(ss.pbvh, vd.vertex);
    }
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
    if (ss.cache->brush->flag2 & BRUSH_USE_COLOR_AS_DISPLACEMENT &&
        (brush.mtex.brush_map_mode == MTEX_MAP_MODE_AREA))
    {
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
    else {
      float fade = SCULPT_brush_strength_factor(&ss,
                                                &brush,
                                                vd.co,
                                                sqrtf(test.dist),
                                                vd.no,
                                                vd.fno,
                                                vd.mask,
                                                vd.vertex,
                                                thread_id,
                                                &automask_data);
      proxy[vd.i] = offset * fade;
    }

    if (vd.is_mesh) {
      BKE_pbvh_vert_tag_update_normal(ss.pbvh, vd.vertex);
    }
  }
  BKE_pbvh_vertex_iter_end;
}

void do_draw_brush(Sculpt &sd, Object &object, Span<PBVHNode *> nodes)
{
  SculptSession &ss = *object.sculpt;
  Brush &brush = *BKE_paint_brush(&sd.paint);
  const float bstrength = ss.cache->bstrength;

  /* Offset with as much as possible factored in already. */
  float3 effective_normal;
  SCULPT_tilt_effective_normal_get(&ss, &brush, effective_normal);

  const float3 offset = effective_normal * ss.cache->radius * ss.cache->scale * bstrength;

  /* XXX: this shouldn't be necessary, but sculpting crashes in blender2.8 otherwise
   * initialize before threads so they can do curve mapping. */
  BKE_curvemapping_init(brush.curve);

  threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
    for (const int i : range) {
      switch (BKE_pbvh_type(object.sculpt->pbvh)) {
        case PBVH_FACES:
          calc_faces(object, brush, offset, *nodes[i]);
          break;
        case PBVH_GRIDS:
          calc_grids(object, brush, offset, *nodes[i]);
          break;
        case PBVH_BMESH:
          calc_bmesh(object, brush, offset, *nodes[i]);
          break;
      };
    }
  });
}

}  // namespace blender::ed::sculpt_paint
