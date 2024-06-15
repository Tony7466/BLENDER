/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "editors/sculpt_paint/brushes/types.hh"

#include "DNA_brush_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_brush.hh"
#include "BKE_key.hh"
#include "BKE_mesh.hh"
#include "BKE_paint.hh"
#include "BKE_pbvh.hh"
#include "BKE_subdiv_ccg.hh"

#include "BLI_array.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_task.h"
#include "BLI_task.hh"

#include "editors/sculpt_paint/mesh_brush_common.hh"
#include "editors/sculpt_paint/sculpt_intern.hh"

namespace blender::ed::sculpt_paint {

inline namespace crease_cc {

struct LocalData {
  Vector<float> factors;
  Vector<float> distances;
  Vector<float3> translations;
};

static void write_translations(const Sculpt &sd,
                               Object &object,
                               const Span<float3> positions_eval,
                               const Span<int> verts,
                               const MutableSpan<float3> translations,
                               const MutableSpan<float3> positions_orig)
{
  SculptSession &ss = *object.sculpt;

  clip_and_lock_translations(sd, ss, positions_eval, verts, translations);

  apply_translations_to_pbvh(*ss.pbvh, verts, translations);

  if (!ss.deform_imats.is_empty()) {
    apply_crazyspace_to_translations(ss.deform_imats, verts, translations);
  }

  apply_translations(translations, verts, positions_orig);
  apply_translations_to_shape_keys(object, verts, translations, positions_orig);
}

static void calc_faces(const Sculpt &sd,
                       const Brush &brush,
                       const float3 &offset,
                       const float strength,
                       const Span<float3> positions_eval,
                       const Span<float3> vert_normals,
                       const PBVHNode &node,
                       Object &object,
                       LocalData &tls,
                       const MutableSpan<float3> positions_orig)
{
  SculptSession &ss = *object.sculpt;
  const StrokeCache &cache = *ss.cache;
  const float3 &location = cache.location;
  Mesh &mesh = *static_cast<Mesh *>(object.data);

  const Span<int> verts = bke::pbvh::node_unique_verts(node);

  tls.factors.reinitialize(verts.size());
  const MutableSpan<float> factors = tls.factors;
  fill_factor_from_hide_and_mask(mesh, verts, factors);

  if (brush.flag & BRUSH_FRONTFACE) {
    calc_front_face(cache.view_normal, vert_normals, verts, factors);
  }

  tls.distances.reinitialize(verts.size());
  const MutableSpan<float> distances = tls.distances;
  calc_distance_falloff(
      ss, positions_eval, verts, eBrushFalloffShape(brush.falloff_shape), distances, factors);
  calc_brush_strength_factors(cache, brush, distances, factors);

  if (cache.automasking) {
    auto_mask::calc_vert_factors(object, *cache.automasking, node, verts, factors);
  }

  calc_brush_texture_factors(ss, brush, positions_eval, verts, factors);

  scale_factors(factors, strength);

  tls.translations.reinitialize(verts.size());
  const MutableSpan<float3> translations = tls.translations;

  for (const int i : verts.index_range()) {
    translations[i] = location - positions_eval[verts[i]];
  }

  if (brush.falloff_shape == PAINT_FALLOFF_SHAPE_TUBE) {
    for (const int i : verts.index_range()) {
      project_plane_v3_v3v3(translations[i], translations[i], cache.view_normal);
    }
  }

  scale_translations(translations, factors);
  for (const int i : translations.index_range()) {
    translations[i] *= strength;
  }

  /* Use surface normal for 'spvc', so the vertices are pinched towards a line instead of a
   * single point. Without this we get a 'flat' surface surrounding the pinch. */
  for (const int i : verts.index_range()) {
    project_plane_v3_v3v3(translations[i], translations[i], cache.sculpt_normal_symm);
  }

  for (const int i : verts.index_range()) {
    translations[i] += offset * factors[i];
  }

  write_translations(sd, object, positions_eval, verts, translations, positions_orig);
}

static void calc_grids(
    Object &object, const Brush &brush, const float3 &offset, const float strength, PBVHNode &node)
{
  SculptSession &ss = *object.sculpt;
  const StrokeCache &cache = *ss.cache;
  const float3 &location = cache.location;

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
      const float3 &co = CCG_elem_offset_co(key, elem, j);
      if (!sculpt_brush_test_sq_fn(test, co)) {
        i++;
        continue;
      }

      float3 translation = location - co;
      if (brush.falloff_shape == PAINT_FALLOFF_SHAPE_TUBE) {
        project_plane_v3_v3v3(translation, translation, cache.view_normal);
      }

      auto_mask::node_update(automask_data, i);
      const float fade = SCULPT_brush_strength_factor(
          ss,
          brush,
          co,
          math::sqrt(test.dist),
          CCG_elem_offset_no(key, elem, j),
          nullptr,
          key.has_mask ? CCG_elem_offset_mask(key, elem, j) : 0.0f,
          BKE_pbvh_make_vref(grid_verts_start + j),
          thread_id,
          &automask_data);

      translation *= fade;
      translation *= strength;

      /* Use surface normal for 'spvc', so the vertices are pinched towards a line instead of a
       * single point. Without this we get a 'flat' surface surrounding the pinch. */
      project_plane_v3_v3v3(translation, translation, cache.sculpt_normal_symm);

      translation += offset * fade;

      proxy[i] = translation;
      i++;
    }
  }
}

static void calc_bmesh(
    Object &object, const Brush &brush, const float3 &offset, const float strength, PBVHNode &node)
{
  SculptSession &ss = *object.sculpt;
  const StrokeCache &cache = *ss.cache;
  const float3 &location = cache.location;

  SculptBrushTest test;
  SculptBrushTestFn sculpt_brush_test_sq_fn = SCULPT_brush_test_init_with_falloff_shape(
      ss, test, brush.falloff_shape);
  const int thread_id = BLI_task_parallel_thread_id(nullptr);
  auto_mask::NodeData automask_data = auto_mask::node_begin(
      object, ss.cache->automasking.get(), node);

  const int mask_offset = CustomData_get_offset_named(
      &ss.bm->vdata, CD_PROP_FLOAT, ".sculpt_mask");

  /* TODO: Remove usage of proxies. */
  const MutableSpan<float3> proxy = BKE_pbvh_node_add_proxy(*ss.pbvh, node).co;
  int i = 0;
  for (BMVert *vert : BKE_pbvh_bmesh_node_unique_verts(&node)) {
    if (BM_elem_flag_test(vert, BM_ELEM_HIDDEN)) {
      i++;
      continue;
    }
    if (!sculpt_brush_test_sq_fn(test, vert->co)) {
      i++;
      continue;
    }

    float3 translation = location - float3(vert->co);
    if (brush.falloff_shape == PAINT_FALLOFF_SHAPE_TUBE) {
      project_plane_v3_v3v3(translation, translation, cache.view_normal);
    }

    auto_mask::node_update(automask_data, i);
    const float mask = mask_offset == -1 ? 0.0f : BM_ELEM_CD_GET_FLOAT(vert, mask_offset);
    const float fade = SCULPT_brush_strength_factor(ss,
                                                    brush,
                                                    vert->co,
                                                    math::sqrt(test.dist),
                                                    vert->no,
                                                    nullptr,
                                                    mask,
                                                    BKE_pbvh_make_vref(intptr_t(vert)),
                                                    thread_id,
                                                    &automask_data);

    translation *= fade;
    translation *= strength;

    /* Use surface normal for 'spvc', so the vertices are pinched towards a line instead of a
     * single point. Without this we get a 'flat' surface surrounding the pinch. */
    project_plane_v3_v3v3(translation, translation, cache.sculpt_normal_symm);

    translation += offset * fade;

    proxy[i] = translation;
    i++;
  }
}

}  // namespace crease_cc

void do_crease_brush(const Scene &scene, const Sculpt &sd, Object &object, Span<PBVHNode *> nodes)
{
  const SculptSession &ss = *object.sculpt;
  const Brush &brush = *BKE_paint_brush_for_read(&sd.paint);

  /* Offset with as much as possible factored in already. */
  const float3 offset = ss.cache->sculpt_normal_symm * ss.cache->scale * ss.cache->radius *
                        ss.cache->bstrength;

  /* We divide out the squared alpha and multiply by the squared crease
   * to give us the pinch strength. */
  float crease_correction = brush.crease_pinch_factor * brush.crease_pinch_factor;
  float brush_alpha = BKE_brush_alpha_get(&scene, &brush);
  if (brush_alpha > 0.0f) {
    crease_correction /= brush_alpha * brush_alpha;
  }

  /* We always want crease to pinch or blob to relax even when draw is negative. */
  float strength = std::abs(ss.cache->bstrength);

  if (brush.sculpt_tool == SCULPT_TOOL_BLOB) {
    strength *= -1.0f;
  }

  /* Use surface normal for 'spvc', so the vertices are pinched towards a line instead of a single
   * point. Without this we get a 'flat' surface surrounding the pinch. */
  //   sculpt_project_v3_cache_init(&spvc, ss.cache->sculpt_normal_symm);

  switch (BKE_pbvh_type(*object.sculpt->pbvh)) {
    case PBVH_FACES: {
      threading::EnumerableThreadSpecific<LocalData> all_tls;
      Mesh &mesh = *static_cast<Mesh *>(object.data);
      const PBVH &pbvh = *ss.pbvh;
      const Span<float3> positions_eval = BKE_pbvh_get_vert_positions(pbvh);
      const Span<float3> vert_normals = BKE_pbvh_get_vert_normals(pbvh);
      MutableSpan<float3> positions_orig = mesh.vert_positions_for_write();
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        LocalData &tls = all_tls.local();
        for (const int i : range) {
          calc_faces(sd,
                     brush,
                     offset,
                     strength,
                     positions_eval,
                     vert_normals,
                     *nodes[i],
                     object,
                     tls,
                     positions_orig);
          BKE_pbvh_node_mark_positions_update(nodes[i]);
        }
      });
      break;
    }
    case PBVH_GRIDS:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          calc_grids(object, brush, offset, strength, *nodes[i]);
        }
      });
      break;
    case PBVH_BMESH:
      threading::parallel_for(nodes.index_range(), 1, [&](const IndexRange range) {
        for (const int i : range) {
          calc_bmesh(object, brush, offset, strength, *nodes[i]);
        }
      });
      break;
  }
}

}  // namespace blender::ed::sculpt_paint
