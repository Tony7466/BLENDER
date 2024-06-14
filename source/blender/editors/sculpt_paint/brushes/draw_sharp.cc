/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "editors/sculpt_paint/brushes/types.hh"

#include "DNA_brush_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_key.hh"
#include "BKE_mesh.hh"
#include "BKE_paint.hh"
#include "BKE_pbvh.hh"
#include "BKE_subdiv_ccg.hh"

#include "BLI_array.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_task.h"
#include "BLI_task.hh"

#include "editors/sculpt_paint/mesh_brush_common.hh"
#include "editors/sculpt_paint/sculpt_intern.hh"

namespace blender::ed::sculpt_paint {

inline namespace draw_sharp_cc {

struct LocalData {
  Vector<float> factors;
  Vector<float> distances;
  Vector<float3> translations;
};

static void calc_faces_sharp(const Sculpt &sd,
                             const Brush &brush,
                             const float3 &offset,
                             const PBVHNode &node,
                             Object &object,
                             LocalData &tls,
                             const MutableSpan<float3> positions_orig)
{
  SculptSession &ss = *object.sculpt;
  StrokeCache &cache = *ss.cache;
  Mesh &mesh = *static_cast<Mesh *>(object.data);

  SculptOrigVertData orig_data;
  SCULPT_orig_vert_data_init(orig_data, object, node, undo::Type::Position);

  const Span<int> verts = bke::pbvh::node_unique_verts(node);

  for (const int i : verts.index_range()) {
    SCULPT_orig_vert_data_update(orig_data, i);
  }

  tls.factors.reinitialize(verts.size());
  const MutableSpan<float> factors = tls.factors;
  fill_factor_from_hide_and_mask(mesh, verts, factors);

  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, orig_data, verts, factors);
  }

  tls.distances.reinitialize(verts.size());
  const MutableSpan<float> distances = tls.distances;
  calc_distance_falloff(
      ss, orig_data, verts, eBrushFalloffShape(brush.falloff_shape), distances, factors);
  calc_brush_strength_factors(ss, brush, verts, distances, factors);

  if (ss.cache->automasking) {
    auto_mask::calc_vert_factors(object, *ss.cache->automasking, node, verts, factors);
  }

  calc_brush_texture_factors(ss, brush, orig_data, verts, factors);

  tls.translations.reinitialize(verts.size());
  const MutableSpan<float3> translations = tls.translations;
  for (const int i : verts.index_range()) {
    SCULPT_orig_vert_data_update(orig_data, verts[i]);
    translations[i] = offset * factors[i];
  }

  clip_and_lock_translations(sd, ss, orig_data, verts, translations);

  if (!ss.deform_imats.is_empty()) {
    apply_crazyspace_to_translations(ss.deform_imats, verts, translations);
  }

  apply_translations(translations, verts, positions_orig);
  apply_translations_to_shape_keys(object, verts, translations, positions_orig);
  apply_translations_to_pbvh(*ss.pbvh, verts, translations);
}

static void calc_grids_sharp(Object &object,
                             const Brush &brush,
                             const float3 &offset,
                             PBVHNode &node)
{
  SculptSession &ss = *object.sculpt;

  SculptBrushTest test;
  SculptBrushTestFn sculpt_brush_test_sq_fn = SCULPT_brush_test_init_with_falloff_shape(
      ss, test, brush.falloff_shape);
  const int thread_id = BLI_task_parallel_thread_id(nullptr);
  auto_mask::NodeData automask_data = auto_mask::node_begin(
      object, ss.cache->automasking.get(), node);

  SubdivCCG &subdiv_ccg = *ss.subdiv_ccg;
  const CCGKey key = *BKE_pbvh_get_grid_key(*ss.pbvh);
  const Span<CCGElem *> grids = subdiv_ccg.grids;
  const BitGroupVector<> &grid_hidden = subdiv_ccg.grid_hidden;

  /* TODO: Remove usage of proxies. */
  const MutableSpan<float3> proxy = BKE_pbvh_node_add_proxy(*ss.pbvh, node).co;
  int i = 0;
  for (const int grid : bke::pbvh::node_grid_indices(node)) {
    const int grid_verts_start = grid * key.grid_area;
    CCGElem *elem = grids[grid];
    for (const int j : IndexRange(key.grid_area)) {
      if (!grid_hidden.is_empty() && grid_hidden[grid][j]) {
        i++;
        continue;
      }
      // SCULPT_orig_vert_data_update_ss(subdiv_ccg, key, elem, j);
      if (!sculpt_brush_test_sq_fn(test, CCG_elem_offset_co(key, elem, j))) {
        i++;
        continue;
      }
      auto_mask::node_update(automask_data, i);
      const float fade = SCULPT_brush_strength_factor(
          ss,
          brush,
          CCG_elem_offset_co(key, elem, j),
          math::sqrt(test.dist),
          CCG_elem_offset_no(key, elem, j),
          nullptr,
          key.has_mask ? CCG_elem_offset_mask(key, elem, j) : 0.0f,
          BKE_pbvh_make_vref(grid_verts_start + j),
          thread_id,
          &automask_data);
      proxy[i] = offset * fade;
      i++;
    }
  }
}

static void calc_bmesh_sharp(Object &object,
                             const Brush &brush,
                             const float3 &offset,
                             PBVHNode &node)
{
  SculptSession &ss = *object.sculpt;

  SculptBrushTest test;
  SculptBrushTestFn sculpt_brush_test_sq_fn = SCULPT_brush_test_init_with_falloff_shape(
      ss, test, brush.falloff_shape);
  const int thread_id = BLI_task_parallel_thread_id(nullptr);
  auto_mask::NodeData automask_data = auto_mask::node_begin(
      object, ss.cache->automasking.get(), node);

  const int mask_offset = CustomData_get_offset_named(
      &ss.bm->vdata, CD_PROP_FLOAT, ".sculpt_mask");

  SculptOrigVertData orig_data;
  SCULPT_orig_vert_data_init(orig_data, object, node, undo::Type::Position);

  /* TODO: Remove usage of proxies. */
  const MutableSpan<float3> proxy = BKE_pbvh_node_add_proxy(*ss.pbvh, node).co;
  int i = 0;
  for (BMVert *vert : BKE_pbvh_bmesh_node_unique_verts(&node)) {
    if (BM_elem_flag_test(vert, BM_ELEM_HIDDEN)) {
      i++;
      continue;
    }

    SCULPT_orig_vert_data_update(orig_data, *vert); // WIP <--- Fix this null data
    if (!sculpt_brush_test_sq_fn(test, orig_data.co)) {
      i++;
      continue;
    }
    auto_mask::node_update(automask_data, *vert);
    const float mask = mask_offset == -1 ? 0.0f : BM_ELEM_CD_GET_FLOAT(vert, mask_offset);
    const float fade = SCULPT_brush_strength_factor(ss,
                                                    brush,
                                                    orig_data.co,
                                                    math::sqrt(test.dist),
                                                    orig_data.no,
                                                    nullptr,
                                                    mask,
                                                    BKE_pbvh_make_vref(intptr_t(vert)),
                                                    thread_id,
                                                    &automask_data);
    proxy[i] = offset * fade;
    i++;
  }
}

}  // namespace draw_sharp_cc

void do_draw_sharp_brush(const Sculpt &sd, Object &object, Span<PBVHNode *> nodes)
{
  const SculptSession &ss = *object.sculpt;
  const Brush &brush = *BKE_paint_brush_for_read(&sd.paint);
  const float bstrength = ss.cache->bstrength;

  /* Offset with as much as possible factored in already. */
  float3 effective_normal;
  SCULPT_tilt_effective_normal_get(ss, brush, effective_normal);

  const float3 offset = effective_normal * ss.cache->radius * ss.cache->scale * bstrength;

  switch (BKE_pbvh_type(*object.sculpt->pbvh)) {
    case PBVH_FACES: {
      threading::EnumerableThreadSpecific<LocalData> all_tls;
      Mesh &mesh = *static_cast<Mesh *>(object.data);
      MutableSpan<float3> positions_orig = mesh.vert_positions_for_write();
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        LocalData &tls = all_tls.local();
        for (const int i : range) {
          calc_faces_sharp(sd, brush, offset, *nodes[i], object, tls, positions_orig);
          BKE_pbvh_node_mark_positions_update(nodes[i]);
        }
      });
      break;
    }
    case PBVH_GRIDS:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          calc_grids_sharp(object, brush, offset, *nodes[i]);
        }
      });
      break;
    case PBVH_BMESH:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          calc_bmesh_sharp(object, brush, offset, *nodes[i]);
        }
      });
      break;
  }
}

}  // namespace blender::ed::sculpt_paint
