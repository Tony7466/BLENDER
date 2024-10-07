/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "MEM_guardedalloc.h"

#include "BLI_array_utils.h"
#include "BLI_blenlib.h"
#include "BLI_ghash.h"
#include "BLI_hash.h"
#include "BLI_heap.h"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_polyfill_2d.h"
#include "BLI_span.hh"
#include "BLI_string_utils.hh"

#include "DNA_gpencil_legacy_types.h"
#include "DNA_gpencil_modifier_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_scene_types.h"

#include "BKE_attribute.hh"
#include "BKE_deform.hh"
#include "BKE_gpencil_curve_legacy.h"
#include "BKE_gpencil_geom_legacy.h"
#include "BKE_gpencil_legacy.h"
#include "BKE_material.h"
#include "BKE_object.hh"
#include "BKE_object_types.hh"

#include "DEG_depsgraph_query.hh"

using blender::float3;
using blender::Span;

/* -------------------------------------------------------------------- */
/** \name Grease Pencil Object: Bound-box Support
 * \{ */

bool BKE_gpencil_stroke_minmax(const bGPDstroke *gps,
                               const bool use_select,
                               float r_min[3],
                               float r_max[3])
{
  if (gps == nullptr) {
    return false;
  }

  bool changed = false;
  if (use_select) {
    for (const bGPDspoint &pt : Span(gps->points, gps->totpoints)) {
      if (pt.flag & GP_SPOINT_SELECT) {
        minmax_v3v3_v3(r_min, r_max, &pt.x);
        changed = true;
      }
    }
  }
  else {
    for (const bGPDspoint &pt : Span(gps->points, gps->totpoints)) {
      minmax_v3v3_v3(r_min, r_max, &pt.x);
      changed = true;
    }
  }

  return changed;
}

std::optional<blender::Bounds<blender::float3>> BKE_gpencil_data_minmax(const bGPdata *gpd)
{
  bool changed = false;

  float3 min;
  float3 max;
  INIT_MINMAX(min, max);
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    bGPDframe *gpf = gpl->actframe;

    if (gpf != nullptr) {
      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
        changed |= BKE_gpencil_stroke_minmax(gps, false, min, max);
      }
    }
  }

  if (!changed) {
    return std::nullopt;
  }

  return blender::Bounds<blender::float3>{min, max};
}

void BKE_gpencil_centroid_3d(bGPdata *gpd, float r_centroid[3])
{
  using namespace blender;
  const Bounds<float3> bounds = BKE_gpencil_data_minmax(gpd).value_or(Bounds(float3(0)));
  copy_v3_v3(r_centroid, math::midpoint(bounds.min, bounds.max));
}

void BKE_gpencil_stroke_boundingbox_calc(bGPDstroke *gps)
{
  INIT_MINMAX(gps->boundbox_min, gps->boundbox_max);
  BKE_gpencil_stroke_minmax(gps, false, gps->boundbox_min, gps->boundbox_max);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Stroke Sample
 * \{ */

static int stroke_march_next_point(const bGPDstroke *gps,
                                   const int index_next_pt,
                                   const float *current,
                                   const float dist,
                                   float *result,
                                   float *pressure,
                                   float *strength,
                                   float *vert_color,
                                   float *uv_fac,
                                   float *uv_fill,
                                   float *uv_rot,
                                   float *ratio_result,
                                   int *index_from,
                                   int *index_to)
{
  float remaining_till_next = 0.0f;
  float remaining_march = dist;
  float step_start[3];
  float point[3];
  int next_point_index = index_next_pt;
  bGPDspoint *pt = nullptr;

  if (next_point_index == gps->totpoints) {
    next_point_index = 0;
  }

  copy_v3_v3(step_start, current);
  pt = &gps->points[next_point_index];
  copy_v3_v3(point, &pt->x);
  remaining_till_next = len_v3v3(point, step_start);

  while (remaining_till_next < remaining_march && next_point_index) {
    remaining_march -= remaining_till_next;
    pt = &gps->points[next_point_index];
    if (pt->flag & GP_SPOINT_TEMP_TAG) {
      pt = &gps->points[next_point_index];
      copy_v3_v3(result, &pt->x);
      *pressure = gps->points[next_point_index].pressure;
      *strength = gps->points[next_point_index].strength;
      memcpy(vert_color, gps->points[next_point_index].vert_color, sizeof(float[4]));

      *index_from = next_point_index == 0 ? (gps->totpoints - 1) : (next_point_index - 1);
      *index_to = next_point_index;
      *ratio_result = 1.0f;
      next_point_index++;
      return next_point_index == 0 ? gps->totpoints : next_point_index;
    }
    next_point_index++;
    copy_v3_v3(point, &pt->x);
    copy_v3_v3(step_start, point);
    if (!(next_point_index < gps->totpoints)) {
      if (gps->flag & GP_STROKE_CYCLIC) {
        next_point_index = 0;
      }
      else {
        next_point_index = gps->totpoints - 1;
        remaining_till_next = 0;
        break;
      }
    }
    pt = &gps->points[next_point_index];
    copy_v3_v3(point, &pt->x);
    remaining_till_next = len_v3v3(point, step_start);
  }
  if (remaining_till_next < remaining_march) {
    pt = &gps->points[next_point_index];
    copy_v3_v3(result, &pt->x);
    *pressure = gps->points[next_point_index].pressure;
    *strength = gps->points[next_point_index].strength;
    memcpy(vert_color, gps->points[next_point_index].vert_color, sizeof(float[4]));

    *index_from = next_point_index == 0 ? (gps->totpoints - 1) : (next_point_index - 1);
    *index_to = next_point_index;
    *ratio_result = 1.0f;

    return 0;
  }

  *index_from = next_point_index == 0 ? (gps->totpoints - 1) : (next_point_index - 1);
  *index_to = next_point_index;

  float ratio = remaining_march / remaining_till_next;
  interp_v3_v3v3(result, step_start, point, ratio);
  *ratio_result = ratio;
  float d1 = len_v3v3(result, &gps->points[*index_from].x);
  float d2 = len_v3v3(result, &gps->points[next_point_index].x);
  float vratio = d1 / (d1 + d2);

  *pressure = interpf(
      gps->points[next_point_index].pressure, gps->points[*index_from].pressure, vratio);
  *strength = interpf(
      gps->points[next_point_index].strength, gps->points[*index_from].strength, vratio);
  *uv_fac = interpf(gps->points[next_point_index].uv_fac, gps->points[*index_from].uv_fac, vratio);
  *uv_rot = interpf(gps->points[next_point_index].uv_rot, gps->points[*index_from].uv_rot, vratio);
  interp_v2_v2v2(
      uv_fill, gps->points[*index_from].uv_fill, gps->points[next_point_index].uv_fill, vratio);
  interp_v4_v4v4(vert_color,
                 gps->points[*index_from].vert_color,
                 gps->points[next_point_index].vert_color,
                 vratio);

  return next_point_index == 0 ? gps->totpoints : next_point_index;
}

static int stroke_march_next_point_no_interp(const bGPDstroke *gps,
                                             const int index_next_pt,
                                             const float *current,
                                             const float dist,
                                             const float sharp_threshold,
                                             float *result)
{
  float remaining_till_next = 0.0f;
  float remaining_march = dist;
  float step_start[3];
  float point[3];
  int next_point_index = index_next_pt;
  bGPDspoint *pt = nullptr;

  if (next_point_index == gps->totpoints) {
    next_point_index = 0;
  }

  copy_v3_v3(step_start, current);
  pt = &gps->points[next_point_index];
  copy_v3_v3(point, &pt->x);
  remaining_till_next = len_v3v3(point, step_start);

  while (remaining_till_next < remaining_march && next_point_index) {
    remaining_march -= remaining_till_next;
    pt = &gps->points[next_point_index];
    if (next_point_index < gps->totpoints - 1 &&
        angle_v3v3v3(&gps->points[next_point_index - 1].x,
                     &gps->points[next_point_index].x,
                     &gps->points[next_point_index + 1].x) < sharp_threshold)
    {
      copy_v3_v3(result, &pt->x);
      pt->flag |= GP_SPOINT_TEMP_TAG;
      next_point_index++;
      return next_point_index == 0 ? gps->totpoints : next_point_index;
    }
    next_point_index++;
    copy_v3_v3(point, &pt->x);
    copy_v3_v3(step_start, point);
    if (!(next_point_index < gps->totpoints)) {
      if (gps->flag & GP_STROKE_CYCLIC) {
        next_point_index = 0;
      }
      else {
        next_point_index = gps->totpoints - 1;
        remaining_till_next = 0;
        break;
      }
    }
    pt = &gps->points[next_point_index];
    copy_v3_v3(point, &pt->x);
    remaining_till_next = len_v3v3(point, step_start);
  }
  if (remaining_till_next < remaining_march) {
    pt = &gps->points[next_point_index];
    copy_v3_v3(result, &pt->x);
    /* Stroke marching only terminates here. */
    return 0;
  }

  float ratio = remaining_march / remaining_till_next;
  interp_v3_v3v3(result, step_start, point, ratio);
  return next_point_index == 0 ? gps->totpoints : next_point_index;
}

static int stroke_march_count(const bGPDstroke *gps, const float dist, const float sharp_threshold)
{
  int point_count = 0;
  float point[3];
  int next_point_index = 1;
  bGPDspoint *pt = nullptr;

  pt = &gps->points[0];
  copy_v3_v3(point, &pt->x);
  point_count++;

  /* Sharp points will be tagged by the stroke_march_next_point_no_interp() call below. */
  for (int i = 0; i < gps->totpoints; i++) {
    gps->points[i].flag &= (~GP_SPOINT_TEMP_TAG);
  }

  while ((next_point_index = stroke_march_next_point_no_interp(
              gps, next_point_index, point, dist, sharp_threshold, point)) > -1)
  {
    point_count++;
    if (next_point_index == 0) {
      break; /* last point finished */
    }
  }
  return point_count;
}

static void stroke_defvert_create_nr_list(MDeformVert *dv_list,
                                          int count,
                                          ListBase *result,
                                          int *totweight)
{
  LinkData *ld;
  MDeformVert *dv;
  MDeformWeight *dw;
  int i, j;
  int tw = 0;
  for (i = 0; i < count; i++) {
    dv = &dv_list[i];

    /* find def_nr in list, if not exist, then create one */
    for (j = 0; j < dv->totweight; j++) {
      bool found = false;
      dw = &dv->dw[j];
      LISTBASE_FOREACH (LinkData *, ld, result) {
        if (ld->data == POINTER_FROM_INT(dw->def_nr)) {
          found = true;
          break;
        }
      }
      if (!found) {
        ld = MEM_cnew<LinkData>("def_nr_item");
        ld->data = POINTER_FROM_INT(dw->def_nr);
        BLI_addtail(result, ld);
        tw++;
      }
    }
  }

  *totweight = tw;
}

static MDeformVert *stroke_defvert_new_count(int count, int totweight, ListBase *def_nr_list)
{
  int i, j;
  MDeformVert *dst = (MDeformVert *)MEM_mallocN(count * sizeof(MDeformVert), "new_deformVert");

  for (i = 0; i < count; i++) {
    dst[i].dw = (MDeformWeight *)MEM_mallocN(sizeof(MDeformWeight) * totweight,
                                             "new_deformWeight");
    dst[i].totweight = totweight;
    j = 0;
    /* re-assign deform groups */
    LISTBASE_FOREACH (LinkData *, ld, def_nr_list) {
      dst[i].dw[j].def_nr = POINTER_AS_INT(ld->data);
      j++;
    }
  }

  return dst;
}

static void stroke_interpolate_deform_weights(
    bGPDstroke *gps, int index_from, int index_to, float ratio, MDeformVert *vert)
{
  const MDeformVert *vl = &gps->dvert[index_from];
  const MDeformVert *vr = &gps->dvert[index_to];

  for (int i = 0; i < vert->totweight; i++) {
    float wl = BKE_defvert_find_weight(vl, vert->dw[i].def_nr);
    float wr = BKE_defvert_find_weight(vr, vert->dw[i].def_nr);
    vert->dw[i].weight = interpf(wr, wl, ratio);
  }
}

bool BKE_gpencil_stroke_sample(bGPdata *gpd,
                               bGPDstroke *gps,
                               const float dist,
                               const bool select,
                               const float sharp_threshold)
{
  bGPDspoint *pt = gps->points;
  bGPDspoint *pt1 = nullptr;
  bGPDspoint *pt2 = nullptr;
  ListBase def_nr_list = {nullptr};

  if (gps->totpoints < 2 || dist < FLT_EPSILON) {
    return false;
  }
  /* TODO: Implement feature point preservation. */
  int count = stroke_march_count(gps, dist, sharp_threshold);
  const bool is_cyclic = (gps->flag & GP_STROKE_CYCLIC) != 0;
  if (is_cyclic) {
    count--;
  }

  bGPDspoint *new_pt = (bGPDspoint *)MEM_callocN(sizeof(bGPDspoint) * count,
                                                 "gp_stroke_points_sampled");
  MDeformVert *new_dv = nullptr;

  int result_totweight;

  if (gps->dvert != nullptr) {
    stroke_defvert_create_nr_list(gps->dvert, gps->totpoints, &def_nr_list, &result_totweight);
    new_dv = stroke_defvert_new_count(count, result_totweight, &def_nr_list);
  }

  int next_point_index = 1;
  int i = 0;
  float pressure, strength, ratio_result;
  float uv_fac, uv_rot, uv_fill[2];
  float vert_color[4];
  int index_from, index_to;
  float last_coord[3];

  /*  1st point is always at the start */
  pt1 = &gps->points[0];
  copy_v3_v3(last_coord, &pt1->x);
  pt2 = &new_pt[i];
  copy_v3_v3(&pt2->x, last_coord);
  new_pt[i].pressure = pt[0].pressure;
  new_pt[i].strength = pt[0].strength;
  copy_v3_v3(&pt2->x, last_coord);
  new_pt[i].pressure = pt[0].pressure;
  new_pt[i].strength = pt[0].strength;
  new_pt[i].uv_fac = pt[0].uv_fac;
  new_pt[i].uv_rot = pt[0].uv_rot;
  copy_v2_v2(new_pt[i].uv_fill, pt[0].uv_fill);
  copy_v4_v4(new_pt[i].vert_color, pt[0].vert_color);
  if (select) {
    new_pt[i].flag |= GP_SPOINT_SELECT;
  }
  i++;

  if (new_dv) {
    stroke_interpolate_deform_weights(gps, 0, 0, 0, &new_dv[0]);
  }

  /* The rest. */
  while ((next_point_index = stroke_march_next_point(gps,
                                                     next_point_index,
                                                     last_coord,
                                                     dist,
                                                     last_coord,
                                                     &pressure,
                                                     &strength,
                                                     vert_color,
                                                     &uv_fac,
                                                     uv_fill,
                                                     &uv_rot,
                                                     &ratio_result,
                                                     &index_from,
                                                     &index_to)) > -1)
  {
    if (is_cyclic && next_point_index == 0) {
      break; /* last point finished */
    }
    pt2 = &new_pt[i];
    copy_v3_v3(&pt2->x, last_coord);
    new_pt[i].pressure = pressure;
    new_pt[i].strength = strength;
    new_pt[i].uv_fac = uv_fac;
    new_pt[i].uv_rot = uv_rot;
    copy_v2_v2(new_pt[i].uv_fill, uv_fill);

    memcpy(new_pt[i].vert_color, vert_color, sizeof(float[4]));
    if (select) {
      new_pt[i].flag |= GP_SPOINT_SELECT;
    }

    if (new_dv) {
      stroke_interpolate_deform_weights(gps, index_from, index_to, ratio_result, &new_dv[i]);
    }

    i++;
    if (next_point_index == 0) {
      break; /* last point finished */
    }
  }

  gps->points = new_pt;
  /* Free original vertex list. */
  MEM_freeN(pt);

  if (new_dv) {
    /* Free original weight data. */
    BKE_gpencil_free_stroke_weights(gps);
    MEM_freeN(gps->dvert);
    while (LinkData *ld = (LinkData *)BLI_pophead(&def_nr_list)) {
      MEM_freeN(ld);
    }

    gps->dvert = new_dv;
  }

  BLI_assert(i == count);
  gps->totpoints = i;

  /* Calc geometry data. */
  BKE_gpencil_stroke_geometry_update(gpd, gps);

  return true;
}

/**
 * Give extra stroke points before and after the original tip points.
 * \param gps: Target stroke
 * \param count_before: how many extra points to be added before a stroke
 * \param count_after: how many extra points to be added after a stroke
 */
static bool BKE_gpencil_stroke_extra_points(bGPDstroke *gps,
                                            const int count_before,
                                            const int count_after)
{
  bGPDspoint *pts = gps->points;

  BLI_assert(count_before >= 0);
  BLI_assert(count_after >= 0);
  if (!count_before && !count_after) {
    return false;
  }

  const int new_count = count_before + count_after + gps->totpoints;

  bGPDspoint *new_pts = (bGPDspoint *)MEM_mallocN(sizeof(bGPDspoint) * new_count, __func__);

  for (int i = 0; i < count_before; i++) {
    new_pts[i] = blender::dna::shallow_copy(pts[0]);
  }
  memcpy(static_cast<void *>(&new_pts[count_before]), pts, sizeof(bGPDspoint) * gps->totpoints);
  for (int i = new_count - count_after; i < new_count; i++) {
    new_pts[i] = blender::dna::shallow_copy(pts[gps->totpoints - 1]);
  }

  if (gps->dvert) {
    MDeformVert *new_dv = (MDeformVert *)MEM_mallocN(sizeof(MDeformVert) * new_count, __func__);

    for (int i = 0; i < new_count; i++) {
      MDeformVert *dv = &gps->dvert[std::clamp(i - count_before, 0, gps->totpoints - 1)];
      int inew = i;
      new_dv[inew].flag = dv->flag;
      new_dv[inew].totweight = dv->totweight;
      new_dv[inew].dw = (MDeformWeight *)MEM_mallocN(sizeof(MDeformWeight) * dv->totweight,
                                                     __func__);
      memcpy(new_dv[inew].dw, dv->dw, sizeof(MDeformWeight) * dv->totweight);
    }
    BKE_gpencil_free_stroke_weights(gps);
    MEM_freeN(gps->dvert);
    gps->dvert = new_dv;
  }

  MEM_freeN(gps->points);
  gps->points = new_pts;
  gps->totpoints = new_count;

  return true;
}

bool BKE_gpencil_stroke_stretch(bGPDstroke *gps,
                                const float dist,
                                const float overshoot_fac,
                                const short mode,
                                const bool follow_curvature,
                                const int extra_point_count,
                                const float segment_influence,
                                const float max_angle,
                                const bool invert_curvature)
{
#define BOTH 0
#define START 1
#define END 2

  const bool do_start = ELEM(mode, BOTH, START);
  const bool do_end = ELEM(mode, BOTH, END);
  float used_percent_length = overshoot_fac;
  CLAMP(used_percent_length, 1e-4f, 1.0f);
  if (!isfinite(used_percent_length)) {
    /* #used_percent_length must always be finite, otherwise a segfault occurs.
     * Since this function should never segfault, set #used_percent_length to a safe fallback. */
    /* NOTE: This fallback is used if `gps->totpoints == 2`, see `MOD_gpencil_legacy_length.cc`. */
    used_percent_length = 0.1f;
  }

  if (gps->totpoints <= 1 || dist < FLT_EPSILON || extra_point_count <= 0) {
    return false;
  }

  /* NOTE: When it's just a straight line, we don't need to do the curvature stuff. */
  if (!follow_curvature || gps->totpoints <= 2) {
    /* Not following curvature, just straight line. */
    /* NOTE: #overshoot_point_param can not be zero. */
    float overshoot_point_param = used_percent_length * (gps->totpoints - 1);
    float result[3];

    if (do_start) {
      int index1 = floor(overshoot_point_param);
      int index2 = ceil(overshoot_point_param);
      interp_v3_v3v3(result,
                     &gps->points[index1].x,
                     &gps->points[index2].x,
                     fmodf(overshoot_point_param, 1.0f));
      sub_v3_v3(result, &gps->points[0].x);
      if (UNLIKELY(is_zero_v3(result))) {
        sub_v3_v3v3(result, &gps->points[1].x, &gps->points[0].x);
      }
      madd_v3_v3fl(&gps->points[0].x, result, -dist / len_v3(result));
    }

    if (do_end) {
      int index1 = gps->totpoints - 1 - floor(overshoot_point_param);
      int index2 = gps->totpoints - 1 - ceil(overshoot_point_param);
      interp_v3_v3v3(result,
                     &gps->points[index1].x,
                     &gps->points[index2].x,
                     fmodf(overshoot_point_param, 1.0f));
      sub_v3_v3(result, &gps->points[gps->totpoints - 1].x);
      if (UNLIKELY(is_zero_v3(result))) {
        sub_v3_v3v3(
            result, &gps->points[gps->totpoints - 2].x, &gps->points[gps->totpoints - 1].x);
      }
      madd_v3_v3fl(&gps->points[gps->totpoints - 1].x, result, -dist / len_v3(result));
    }
    return true;
  }

  /* Curvature calculation. */

  /* First allocate the new stroke size. */
  const int first_old_index = do_start ? extra_point_count : 0;
  const int last_old_index = gps->totpoints - 1 + first_old_index;
  const int orig_totpoints = gps->totpoints;
  BKE_gpencil_stroke_extra_points(gps, first_old_index, do_end ? extra_point_count : 0);

  /* The fractional amount of points to query when calculating the average curvature of the
   * strokes. */
  const float overshoot_parameter = used_percent_length * (orig_totpoints - 2);
  int overshoot_pointcount = ceil(overshoot_parameter);
  CLAMP(overshoot_pointcount, 1, orig_totpoints - 2);

  /* Do for both sides without code duplication. */
  float no[3], vec1[3], vec2[3], total_angle[3];
  for (int k = 0; k < 2; k++) {
    if ((k == 0 && !do_start) || (k == 1 && !do_end)) {
      continue;
    }

    const int start_i = k == 0 ? first_old_index :
                                 last_old_index;  // first_old_index, last_old_index
    const int dir_i = 1 - k * 2;                  // 1, -1

    sub_v3_v3v3(vec1, &gps->points[start_i + dir_i].x, &gps->points[start_i].x);
    zero_v3(total_angle);
    float segment_length = normalize_v3(vec1);
    float overshoot_length = 0.0f;

    /* Accumulate rotation angle and length. */
    int j = 0;
    for (int i = start_i; j < overshoot_pointcount; i += dir_i, j++) {
      /* Don't fully add last segment to get continuity in overshoot_fac. */
      float fac = fmin(overshoot_parameter - j, 1.0f);

      /* Read segments. */
      copy_v3_v3(vec2, vec1);
      sub_v3_v3v3(vec1, &gps->points[i + dir_i * 2].x, &gps->points[i + dir_i].x);
      const float len = normalize_v3(vec1);
      float angle = angle_normalized_v3v3(vec1, vec2) * fac;

      /* Add half of both adjacent legs of the current angle. */
      const float added_len = (segment_length + len) * 0.5f * fac;
      overshoot_length += added_len;
      segment_length = len;

      if (angle > max_angle) {
        continue;
      }
      if (angle > M_PI * 0.995f) {
        continue;
      }

      angle *= powf(added_len, segment_influence);

      cross_v3_v3v3(no, vec1, vec2);
      normalize_v3_length(no, angle);
      add_v3_v3(total_angle, no);
    }

    if (UNLIKELY(overshoot_length == 0.0f)) {
      /* Don't do a proper extension if the used points are all in the same position. */
      continue;
    }

    sub_v3_v3v3(vec1, &gps->points[start_i].x, &gps->points[start_i + dir_i].x);
    /* In general curvature = 1/radius. For the case without the
     * weights introduced by #segment_influence, the calculation is:
     * `curvature = delta angle/delta arclength = len_v3(total_angle) / overshoot_length` */
    float curvature = normalize_v3(total_angle) / overshoot_length;
    /* Compensate for the weights powf(added_len, segment_influence). */
    curvature /= powf(overshoot_length / fminf(overshoot_parameter, float(j)), segment_influence);
    if (invert_curvature) {
      curvature = -curvature;
    }
    const float angle_step = curvature * dist / extra_point_count;
    float step_length = dist / extra_point_count;
    if (fabsf(angle_step) > FLT_EPSILON) {
      /* Make a direct step length from the assigned arc step length. */
      step_length *= sin(angle_step * 0.5f) / (angle_step * 0.5f);
    }
    else {
      zero_v3(total_angle);
    }
    const float prev_length = normalize_v3_length(vec1, step_length);

    /* Build rotation matrix here to get best performance. */
    float rot[3][3];
    float q[4];
    axis_angle_to_quat(q, total_angle, angle_step);
    quat_to_mat3(rot, q);

    /* Rotate the starting direction to account for change in edge lengths. */
    axis_angle_to_quat(q,
                       total_angle,
                       fmaxf(0.0f, 1.0f - fabs(segment_influence)) *
                           (curvature * prev_length - angle_step) / 2.0f);
    mul_qt_v3(q, vec1);

    /* Now iteratively accumulate the segments with a rotating added direction. */
    for (int i = start_i - dir_i, j = 0; j < extra_point_count; i -= dir_i, j++) {
      mul_v3_m3v3(vec1, rot, vec1);
      add_v3_v3v3(&gps->points[i].x, vec1, &gps->points[i + dir_i].x);
    }
  }
  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Stroke Trim
 * \{ */

bool BKE_gpencil_stroke_trim_points(bGPDstroke *gps,
                                    const int index_from,
                                    const int index_to,
                                    const bool keep_point)
{
  bGPDspoint *pt = gps->points, *new_pt;
  MDeformVert *dv, *new_dv;

  const int new_count = index_to - index_from + 1;

  if (new_count >= gps->totpoints) {
    return false;
  }

  if ((!keep_point) && (new_count == 1)) {
    if (gps->dvert) {
      BKE_gpencil_free_stroke_weights(gps);
      MEM_freeN(gps->dvert);
    }
    MEM_freeN(gps->points);
    gps->points = nullptr;
    gps->dvert = nullptr;
    gps->totpoints = 0;
    return false;
  }

  new_pt = (bGPDspoint *)MEM_mallocN(sizeof(bGPDspoint) * new_count, "gp_stroke_points_trimmed");
  memcpy(static_cast<void *>(new_pt), &pt[index_from], sizeof(bGPDspoint) * new_count);

  if (gps->dvert) {
    new_dv = (MDeformVert *)MEM_mallocN(sizeof(MDeformVert) * new_count,
                                        "gp_stroke_dverts_trimmed");
    for (int i = 0; i < new_count; i++) {
      dv = &gps->dvert[i + index_from];
      new_dv[i].flag = dv->flag;
      new_dv[i].totweight = dv->totweight;
      new_dv[i].dw = (MDeformWeight *)MEM_mallocN(sizeof(MDeformWeight) * dv->totweight,
                                                  "gp_stroke_dverts_dw_trimmed");
      for (int j = 0; j < dv->totweight; j++) {
        new_dv[i].dw[j].weight = dv->dw[j].weight;
        new_dv[i].dw[j].def_nr = dv->dw[j].def_nr;
      }
    }
    BKE_gpencil_free_stroke_weights(gps);
    MEM_freeN(gps->dvert);
    gps->dvert = new_dv;
  }

  MEM_freeN(gps->points);
  gps->points = new_pt;
  gps->totpoints = new_count;

  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Stroke Split
 * \{ */

bool BKE_gpencil_stroke_split(bGPdata *gpd,
                              bGPDframe *gpf,
                              bGPDstroke *gps,
                              const int before_index,
                              bGPDstroke **remaining_gps)
{
  bGPDstroke *new_gps;
  bGPDspoint *pt = gps->points, *new_pt;
  MDeformVert *dv, *new_dv;

  if (before_index >= gps->totpoints || before_index == 0) {
    return false;
  }

  const int new_count = gps->totpoints - before_index;
  const int old_count = before_index;

  /* Handle remaining segments first. */

  new_gps = BKE_gpencil_stroke_add_existing_style(
      gpf, gps, gps->mat_nr, new_count, gps->thickness);

  new_pt = new_gps->points; /* Allocated from above. */
  memcpy(static_cast<void *>(new_pt), &pt[before_index], sizeof(bGPDspoint) * new_count);

  if (gps->dvert) {
    new_dv = (MDeformVert *)MEM_mallocN(sizeof(MDeformVert) * new_count,
                                        "gp_stroke_dverts_remaining(MDeformVert)");
    for (int i = 0; i < new_count; i++) {
      dv = &gps->dvert[i + before_index];
      new_dv[i].flag = dv->flag;
      new_dv[i].totweight = dv->totweight;
      new_dv[i].dw = (MDeformWeight *)MEM_mallocN(sizeof(MDeformWeight) * dv->totweight,
                                                  "gp_stroke_dverts_dw_remaining(MDeformWeight)");
      for (int j = 0; j < dv->totweight; j++) {
        new_dv[i].dw[j].weight = dv->dw[j].weight;
        new_dv[i].dw[j].def_nr = dv->dw[j].def_nr;
      }
    }
    new_gps->dvert = new_dv;
  }

  (*remaining_gps) = new_gps;

  /* Trim the original stroke into a shorter one.
   * Keep the end point. */

  BKE_gpencil_stroke_trim_points(gps, 0, old_count, false);
  BKE_gpencil_stroke_geometry_update(gpd, gps);
  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Stroke Shrink
 * \{ */

bool BKE_gpencil_stroke_shrink(bGPDstroke *gps, const float dist, const short mode)
{
#define START 1
#define END 2

  bGPDspoint *pt = gps->points, *second_last;
  int i;

  if (gps->totpoints < 2) {
    if (gps->totpoints == 1) {
      second_last = &pt[1];
      if (len_v3v3(&second_last->x, &pt->x) < dist) {
        BKE_gpencil_stroke_trim_points(gps, 0, 0, false);
        return true;
      }
    }

    return false;
  }

  second_last = &pt[gps->totpoints - 2];

  float len;
  float len1, cut_len1;
  float len2, cut_len2;
  len1 = len2 = cut_len1 = cut_len2 = 0.0f;

  int index_start = 0;
  int index_end = 0;
  if (mode == START) {
    i = 0;
    index_end = gps->totpoints - 1;
    while (len1 < dist && gps->totpoints > i + 1) {
      len = len_v3v3(&pt[i].x, &pt[i + 1].x);
      len1 += len;
      cut_len1 = len1 - dist;
      i++;
    }
    index_start = i - 1;
    interp_v3_v3v3(&pt[index_start].x, &pt[index_start + 1].x, &pt[index_start].x, cut_len1 / len);
  }

  if (mode == END) {
    index_start = 0;
    i = 2;
    while (len2 < dist && gps->totpoints >= i) {
      second_last = &pt[gps->totpoints - i];
      len = len_v3v3(&second_last[1].x, &second_last->x);
      len2 += len;
      cut_len2 = len2 - dist;
      i++;
    }
    index_end = gps->totpoints - i + 2;
    interp_v3_v3v3(&pt[index_end].x, &pt[index_end - 1].x, &pt[index_end].x, cut_len2 / len);
  }

  if (index_end <= index_start) {
    index_start = index_end = 0; /* empty stroke */
  }

  if ((index_end == index_start + 1) && (cut_len1 + cut_len2 < 0)) {
    index_start = index_end = 0; /* no length left to cut */
  }

  BKE_gpencil_stroke_trim_points(gps, index_start, index_end, false);

  if (gps->totpoints == 0) {
    return false;
  }

  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Stroke Smooth Positions
 * \{ */

bool BKE_gpencil_stroke_smooth_point(bGPDstroke *gps,
                                     int point_index,
                                     float influence,
                                     int iterations,
                                     const bool smooth_caps,
                                     const bool keep_shape,
                                     bGPDstroke *r_gps)
{
  /* If nothing to do, return early */
  if (gps->totpoints <= 2 || iterations <= 0) {
    return false;
  }

  /* - Overview of the algorithm here and in the following smooth functions:
   *
   *   The smooth functions return the new attribute in question for a single point.
   *   The result is stored in r_gps->points[point_index], while the data is read from gps.
   *   To get a correct result, duplicate the stroke point data and read from the copy,
   *   while writing to the real stroke. Not doing that will result in acceptable, but
   *   asymmetric results.
   *
   * This algorithm works as long as all points are being smoothed. If there is
   * points that should not get smoothed, use the old repeat smooth pattern with
   * the parameter "iterations" set to 1 or 2. (2 matches the old algorithm).
   */

  const bGPDspoint *pt = &gps->points[point_index];
  const bool is_cyclic = (gps->flag & GP_STROKE_CYCLIC) != 0;
  /* If smooth_caps is false, the caps will not be translated by smoothing. */
  if (!smooth_caps && !is_cyclic && ELEM(point_index, 0, gps->totpoints - 1)) {
    copy_v3_v3(&r_gps->points[point_index].x, &pt->x);
    return true;
  }

  /* This function uses a binomial kernel, which is the discrete version of gaussian blur.
   * The weight for a vertex at the relative index point_index is
   * `w = nCr(n, j + n/2) / 2^n = (n/1 * (n-1)/2 * ... * (n-j-n/2)/(j+n/2)) / 2^n`
   * All weights together sum up to 1
   * This is equivalent to doing multiple iterations of averaging neighbors,
   * where n = iterations * 2 and -n/2 <= j <= n/2
   *
   * Now the problem is that `nCr(n, j + n/2)` is very hard to compute for `n > 500`, since even
   * double precision isn't sufficient. A very good robust approximation for n > 20 is
   * `nCr(n, j + n/2) / 2^n = sqrt(2/(pi*n)) * exp(-2*j*j/n)`
   *
   * There is one more problem left: The old smooth algorithm was doing a more aggressive
   * smooth. To solve that problem, choose a different n/2, which does not match the range and
   * normalize the weights on finish. This may cause some artifacts at low values.
   *
   * keep_shape is a new option to stop the stroke from severely deforming.
   * It uses different partially negative weights.
   * w = `2 * (nCr(n, j + n/2) / 2^n) - (nCr(3*n, j + n) / 2^(3*n))`
   *   ~ `2 * sqrt(2/(pi*n)) * exp(-2*j*j/n) - sqrt(2/(pi*3*n)) * exp(-2*j*j/(3*n))`
   * All weights still sum up to 1.
   * Note these weights only work because the averaging is done in relative coordinates.
   */
  float sco[3] = {0.0f, 0.0f, 0.0f};
  float tmp[3];
  const int n_half = keep_shape ? (iterations * iterations) / 8 + iterations :
                                  (iterations * iterations) / 4 + 2 * iterations + 12;
  double w = keep_shape ? 2.0 : 1.0;
  double w2 = keep_shape ?
                  (1.0 / M_SQRT3) * exp((2 * iterations * iterations) / double(n_half * 3)) :
                  0.0;
  double total_w = 0.0;
  for (int step = iterations; step > 0; step--) {
    int before = point_index - step;
    int after = point_index + step;
    float w_before = float(w - w2);
    float w_after = float(w - w2);

    if (is_cyclic) {
      before = (before % gps->totpoints + gps->totpoints) % gps->totpoints;
      after = after % gps->totpoints;
    }
    else {
      if (before < 0) {
        if (!smooth_caps) {
          w_before *= -before / float(point_index);
        }
        before = 0;
      }
      if (after > gps->totpoints - 1) {
        if (!smooth_caps) {
          w_after *= (after - (gps->totpoints - 1)) / float(gps->totpoints - 1 - point_index);
        }
        after = gps->totpoints - 1;
      }
    }

    /* Add both these points in relative coordinates to the weighted average sum. */
    sub_v3_v3v3(tmp, &gps->points[before].x, &pt->x);
    madd_v3_v3fl(sco, tmp, w_before);
    sub_v3_v3v3(tmp, &gps->points[after].x, &pt->x);
    madd_v3_v3fl(sco, tmp, w_after);

    total_w += w_before;
    total_w += w_after;

    w *= (n_half + step) / double(n_half + 1 - step);
    w2 *= (n_half * 3 + step) / double(n_half * 3 + 1 - step);
  }
  total_w += w - w2;
  /* The accumulated weight total_w should be
   * `~sqrt(M_PI * n_half) * exp((iterations * iterations) / n_half) < 100`
   * here, but sometimes not quite. */
  mul_v3_fl(sco, float(1.0 / total_w));
  /* Shift back to global coordinates. */
  add_v3_v3(sco, &pt->x);

  /* Based on influence factor, blend between original and optimal smoothed coordinate. */
  interp_v3_v3v3(&r_gps->points[point_index].x, &pt->x, sco, influence);

  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Stroke Smooth Strength
 * \{ */

bool BKE_gpencil_stroke_smooth_strength(
    bGPDstroke *gps, int point_index, float influence, int iterations, bGPDstroke *r_gps)
{
  /* If nothing to do, return early */
  if (gps->totpoints <= 2 || iterations <= 0) {
    return false;
  }

  /* See BKE_gpencil_stroke_smooth_point for details on the algorithm. */

  const bGPDspoint *pt = &gps->points[point_index];
  const bool is_cyclic = (gps->flag & GP_STROKE_CYCLIC) != 0;
  float strength = 0.0f;
  const int n_half = (iterations * iterations) / 4 + iterations;
  double w = 1.0;
  double total_w = 0.0;
  for (int step = iterations; step > 0; step--) {
    int before = point_index - step;
    int after = point_index + step;
    float w_before = float(w);
    float w_after = float(w);

    if (is_cyclic) {
      before = (before % gps->totpoints + gps->totpoints) % gps->totpoints;
      after = after % gps->totpoints;
    }
    else {
      CLAMP_MIN(before, 0);
      CLAMP_MAX(after, gps->totpoints - 1);
    }

    /* Add both these points in relative coordinates to the weighted average sum. */
    strength += w_before * (gps->points[before].strength - pt->strength);
    strength += w_after * (gps->points[after].strength - pt->strength);

    total_w += w_before;
    total_w += w_after;

    w *= (n_half + step) / double(n_half + 1 - step);
  }
  total_w += w;
  /* The accumulated weight total_w should be
   * ~sqrt(M_PI * n_half) * exp((iterations * iterations) / n_half) < 100
   * here, but sometimes not quite. */
  strength /= total_w;

  /* Based on influence factor, blend between original and optimal smoothed value. */
  r_gps->points[point_index].strength = pt->strength + strength * influence;

  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Stroke Smooth Thickness
 * \{ */

bool BKE_gpencil_stroke_smooth_thickness(
    bGPDstroke *gps, int point_index, float influence, int iterations, bGPDstroke *r_gps)
{
  /* If nothing to do, return early */
  if (gps->totpoints <= 2 || iterations <= 0) {
    return false;
  }

  /* See BKE_gpencil_stroke_smooth_point for details on the algorithm. */

  const bGPDspoint *pt = &gps->points[point_index];
  const bool is_cyclic = (gps->flag & GP_STROKE_CYCLIC) != 0;
  float pressure = 0.0f;
  const int n_half = (iterations * iterations) / 4 + iterations;
  double w = 1.0;
  double total_w = 0.0;
  for (int step = iterations; step > 0; step--) {
    int before = point_index - step;
    int after = point_index + step;
    float w_before = float(w);
    float w_after = float(w);

    if (is_cyclic) {
      before = (before % gps->totpoints + gps->totpoints) % gps->totpoints;
      after = after % gps->totpoints;
    }
    else {
      CLAMP_MIN(before, 0);
      CLAMP_MAX(after, gps->totpoints - 1);
    }

    /* Add both these points in relative coordinates to the weighted average sum. */
    pressure += w_before * (gps->points[before].pressure - pt->pressure);
    pressure += w_after * (gps->points[after].pressure - pt->pressure);

    total_w += w_before;
    total_w += w_after;

    w *= (n_half + step) / double(n_half + 1 - step);
  }
  total_w += w;
  /* The accumulated weight total_w should be
   * ~sqrt(M_PI * n_half) * exp((iterations * iterations) / n_half) < 100
   * here, but sometimes not quite. */
  pressure /= total_w;

  /* Based on influence factor, blend between original and optimal smoothed value. */
  r_gps->points[point_index].pressure = pt->pressure + pressure * influence;

  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Stroke Smooth UV
 * \{ */

bool BKE_gpencil_stroke_smooth_uv(
    bGPDstroke *gps, int point_index, float influence, int iterations, bGPDstroke *r_gps)
{
  /* If nothing to do, return early */
  if (gps->totpoints <= 2 || iterations <= 0) {
    return false;
  }

  /* See BKE_gpencil_stroke_smooth_point for details on the algorithm. */

  const bGPDspoint *pt = &gps->points[point_index];
  const bool is_cyclic = (gps->flag & GP_STROKE_CYCLIC) != 0;

  /* If don't change the caps. */
  if (!is_cyclic && ELEM(point_index, 0, gps->totpoints - 1)) {
    r_gps->points[point_index].uv_rot = pt->uv_rot;
    r_gps->points[point_index].uv_fac = pt->uv_fac;
    return true;
  }

  float uv_rot = 0.0f;
  float uv_fac = 0.0f;
  const int n_half = iterations * iterations + iterations;
  double w = 1.0;
  double total_w = 0.0;
  for (int step = iterations; step > 0; step--) {
    int before = point_index - step;
    int after = point_index + step;
    float w_before = float(w);
    float w_after = float(w);

    if (is_cyclic) {
      before = (before % gps->totpoints + gps->totpoints) % gps->totpoints;
      after = after % gps->totpoints;
    }
    else {
      if (before < 0) {
        w_before *= -before / float(point_index);
        before = 0;
      }
      if (after > gps->totpoints - 1) {
        w_after *= (after - (gps->totpoints - 1)) / float(gps->totpoints - 1 - point_index);
        after = gps->totpoints - 1;
      }
    }

    /* Add both these points in relative coordinates to the weighted average sum. */
    uv_rot += w_before * (gps->points[before].uv_rot - pt->uv_rot);
    uv_rot += w_after * (gps->points[after].uv_rot - pt->uv_rot);
    uv_fac += w_before * (gps->points[before].uv_fac - pt->uv_fac);
    uv_fac += w_after * (gps->points[after].uv_fac - pt->uv_fac);

    total_w += w_before;
    total_w += w_after;

    w *= (n_half + step) / double(n_half + 1 - step);
  }
  total_w += w;
  /* The accumulated weight total_w should be
   * ~sqrt(M_PI * n_half) * exp((iterations * iterations) / n_half) < 100
   * here, but sometimes not quite. */
  uv_rot /= total_w;
  uv_fac /= total_w;

  /* Based on influence factor, blend between original and optimal smoothed value. */
  r_gps->points[point_index].uv_rot = pt->uv_rot + uv_rot * influence;
  r_gps->points[point_index].uv_fac = pt->uv_fac + uv_fac * influence;

  return true;
}

void BKE_gpencil_stroke_smooth(bGPDstroke *gps,
                               const float influence,
                               const int iterations,
                               const bool smooth_position,
                               const bool smooth_strength,
                               const bool smooth_thickness,
                               const bool smooth_uv,
                               const bool keep_shape,
                               const float *weights)
{
  if (influence <= 0 || iterations <= 0) {
    return;
  }

  /* Make a copy of the point data to avoid directionality of the smooth operation. */
  bGPDstroke gps_old = blender::dna::shallow_copy(*gps);
  gps_old.points = (bGPDspoint *)MEM_dupallocN(gps->points);

  /* Smooth stroke. */
  for (int i = 0; i < gps->totpoints; i++) {
    float val = influence;
    if (weights != nullptr) {
      val *= weights[i];
      if (val <= 0.0f) {
        continue;
      }
    }

    /* TODO: Currently the weights only control the influence, but is would be much better if they
     * would control the distribution used in smooth, similar to how the ends are handled. */

    /* Perform smoothing. */
    if (smooth_position) {
      BKE_gpencil_stroke_smooth_point(&gps_old, i, val, iterations, false, keep_shape, gps);
    }
    if (smooth_strength) {
      BKE_gpencil_stroke_smooth_strength(&gps_old, i, val, iterations, gps);
    }
    if (smooth_thickness) {
      BKE_gpencil_stroke_smooth_thickness(&gps_old, i, val, iterations, gps);
    }
    if (smooth_uv) {
      BKE_gpencil_stroke_smooth_uv(&gps_old, i, val, iterations, gps);
    }
  }

  /* Free the copied points array. */
  MEM_freeN(gps_old.points);
}

void BKE_gpencil_stroke_2d_flat(const bGPDspoint *points,
                                int totpoints,
                                float (*points2d)[2],
                                int *r_direction)
{
  BLI_assert(totpoints >= 2);

  const bGPDspoint *pt0 = &points[0];
  const bGPDspoint *pt1 = &points[1];
  const bGPDspoint *pt3 = &points[int(totpoints * 0.75)];

  float locx[3];
  float locy[3];
  float loc3[3];
  float normal[3];

  /* local X axis (p0 -> p1) */
  sub_v3_v3v3(locx, &pt1->x, &pt0->x);

  /* point vector at 3/4 */
  float v3[3];
  if (totpoints == 2) {
    mul_v3_v3fl(v3, &pt3->x, 0.001f);
  }
  else {
    copy_v3_v3(v3, &pt3->x);
  }

  sub_v3_v3v3(loc3, v3, &pt0->x);

  /* vector orthogonal to polygon plane */
  cross_v3_v3v3(normal, locx, loc3);

  /* local Y axis (cross to normal/x axis) */
  cross_v3_v3v3(locy, normal, locx);

  /* Normalize vectors */
  normalize_v3(locx);
  normalize_v3(locy);

  /* Calculate last point first. */
  const bGPDspoint *pt_last = &points[totpoints - 1];
  float tmp[3];
  sub_v3_v3v3(tmp, &pt_last->x, &pt0->x);

  points2d[totpoints - 1][0] = dot_v3v3(tmp, locx);
  points2d[totpoints - 1][1] = dot_v3v3(tmp, locy);

  /* Calculate the scalar cross product of the 2d points. */
  float cross = 0.0f;
  float *co_curr;
  float *co_prev = (float *)&points2d[totpoints - 1];

  /* Get all points in local space */
  for (int i = 0; i < totpoints - 1; i++) {
    const bGPDspoint *pt = &points[i];
    float loc[3];

    /* Get local space using first point as origin */
    sub_v3_v3v3(loc, &pt->x, &pt0->x);

    points2d[i][0] = dot_v3v3(loc, locx);
    points2d[i][1] = dot_v3v3(loc, locy);

    /* Calculate cross product. */
    co_curr = (float *)&points2d[i][0];
    cross += (co_curr[0] - co_prev[0]) * (co_curr[1] + co_prev[1]);
    co_prev = (float *)&points2d[i][0];
  }

  /* Concave (-1), Convex (1) */
  *r_direction = (cross >= 0.0f) ? 1 : -1;
}

void BKE_gpencil_stroke_2d_flat_ref(const bGPDspoint *ref_points,
                                    int ref_totpoints,
                                    const bGPDspoint *points,
                                    int totpoints,
                                    float (*points2d)[2],
                                    const float scale,
                                    int *r_direction)
{
  BLI_assert(totpoints >= 2);

  const bGPDspoint *pt0 = &ref_points[0];
  const bGPDspoint *pt1 = &ref_points[1];
  const bGPDspoint *pt3 = &ref_points[int(ref_totpoints * 0.75)];

  float locx[3];
  float locy[3];
  float loc3[3];
  float normal[3];

  /* local X axis (p0 -> p1) */
  sub_v3_v3v3(locx, &pt1->x, &pt0->x);

  /* point vector at 3/4 */
  float v3[3];
  if (totpoints == 2) {
    mul_v3_v3fl(v3, &pt3->x, 0.001f);
  }
  else {
    copy_v3_v3(v3, &pt3->x);
  }

  sub_v3_v3v3(loc3, v3, &pt0->x);

  /* vector orthogonal to polygon plane */
  cross_v3_v3v3(normal, locx, loc3);

  /* local Y axis (cross to normal/x axis) */
  cross_v3_v3v3(locy, normal, locx);

  /* Normalize vectors */
  normalize_v3(locx);
  normalize_v3(locy);

  /* Get all points in local space */
  for (int i = 0; i < totpoints; i++) {
    const bGPDspoint *pt = &points[i];
    float loc[3];
    float v1[3];
    float vn[3] = {0.0f, 0.0f, 0.0f};

    /* apply scale to extremes of the stroke to get better collision detection
     * the scale is divided to get more control in the UI parameter
     */
    /* first point */
    if (i == 0) {
      const bGPDspoint *pt_next = &points[i + 1];
      sub_v3_v3v3(vn, &pt->x, &pt_next->x);
      normalize_v3(vn);
      mul_v3_fl(vn, scale / 10.0f);
      add_v3_v3v3(v1, &pt->x, vn);
    }
    /* last point */
    else if (i == totpoints - 1) {
      const bGPDspoint *pt_prev = &points[i - 1];
      sub_v3_v3v3(vn, &pt->x, &pt_prev->x);
      normalize_v3(vn);
      mul_v3_fl(vn, scale / 10.0f);
      add_v3_v3v3(v1, &pt->x, vn);
    }
    else {
      copy_v3_v3(v1, &pt->x);
    }

    /* Get local space using first point as origin (ref stroke) */
    sub_v3_v3v3(loc, v1, &pt0->x);

    points2d[i][0] = dot_v3v3(loc, locx);
    points2d[i][1] = dot_v3v3(loc, locy);
  }

  /* Concave (-1), Convex (1), or Auto-detect (0)? */
  *r_direction = int(locy[2]);
}

/* Calc texture coordinates using flat projected points. */
static void gpencil_calc_stroke_fill_uv(const float (*points2d)[2],
                                        bGPDstroke *gps,
                                        const float minv[2],
                                        const float maxv[2],
                                        float (*r_uv)[2])
{
  const float s = sin(gps->uv_rotation);
  const float c = cos(gps->uv_rotation);

  /* Calc center for rotation. */
  float center[2] = {0.5f, 0.5f};
  float d[2];
  d[0] = maxv[0] - minv[0];
  d[1] = maxv[1] - minv[1];
  for (int i = 0; i < gps->totpoints; i++) {
    r_uv[i][0] = (points2d[i][0] - minv[0]) / d[0];
    r_uv[i][1] = (points2d[i][1] - minv[1]) / d[1];

    /* Apply translation. */
    add_v2_v2(r_uv[i], gps->uv_translation);

    /* Apply Rotation. */
    r_uv[i][0] -= center[0];
    r_uv[i][1] -= center[1];

    float x = r_uv[i][0] * c - r_uv[i][1] * s;
    float y = r_uv[i][0] * s + r_uv[i][1] * c;

    r_uv[i][0] = x + center[0];
    r_uv[i][1] = y + center[1];

    /* Apply scale. */
    if (gps->uv_scale != 0.0f) {
      mul_v2_fl(r_uv[i], 1.0f / gps->uv_scale);
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Stroke Fill Triangulate
 * \{ */

void BKE_gpencil_stroke_fill_triangulate(bGPDstroke *gps)
{
  BLI_assert(gps->totpoints >= 3);

  /* allocate memory for temporary areas */
  gps->tot_triangles = gps->totpoints - 2;
  uint(*tmp_triangles)[3] = (uint(*)[3])MEM_mallocN(sizeof(*tmp_triangles) * gps->tot_triangles,
                                                    "GP Stroke temp triangulation");
  float(*points2d)[2] = (float(*)[2])MEM_mallocN(sizeof(*points2d) * gps->totpoints,
                                                 "GP Stroke temp 2d points");
  float(*uv)[2] = (float(*)[2])MEM_mallocN(sizeof(*uv) * gps->totpoints,
                                           "GP Stroke temp 2d uv data");

  int direction = 0;

  /* convert to 2d and triangulate */
  BKE_gpencil_stroke_2d_flat(gps->points, gps->totpoints, points2d, &direction);
  BLI_polyfill_calc(points2d, uint(gps->totpoints), direction, tmp_triangles);

  /* calc texture coordinates automatically */
  float minv[2];
  float maxv[2];
  /* first needs bounding box data */
  ARRAY_SET_ITEMS(minv, -1.0f, -1.0f);
  ARRAY_SET_ITEMS(maxv, 1.0f, 1.0f);

  /* calc uv data */
  gpencil_calc_stroke_fill_uv(points2d, gps, minv, maxv, uv);

  /* Save triangulation data. */
  if (gps->tot_triangles > 0) {
    MEM_SAFE_FREE(gps->triangles);
    gps->triangles = (bGPDtriangle *)MEM_callocN(sizeof(*gps->triangles) * gps->tot_triangles,
                                                 "GP Stroke triangulation");

    for (int i = 0; i < gps->tot_triangles; i++) {
      memcpy(gps->triangles[i].verts, tmp_triangles[i], sizeof(uint[3]));
    }

    /* Copy UVs to bGPDspoint. */
    for (int i = 0; i < gps->totpoints; i++) {
      copy_v2_v2(gps->points[i].uv_fill, uv[i]);
    }
  }
  else {
    /* No triangles needed - Free anything allocated previously */
    if (gps->triangles) {
      MEM_freeN(gps->triangles);
    }

    gps->triangles = nullptr;
  }

  /* clear memory */
  MEM_SAFE_FREE(tmp_triangles);
  MEM_SAFE_FREE(points2d);
  MEM_SAFE_FREE(uv);
}

void BKE_gpencil_stroke_uv_update(bGPDstroke *gps)
{
  if (gps == nullptr || gps->totpoints == 0) {
    return;
  }

  bGPDspoint *pt = gps->points;
  float totlen = 0.0f;
  pt[0].uv_fac = totlen;
  for (int i = 1; i < gps->totpoints; i++) {
    totlen += len_v3v3(&pt[i - 1].x, &pt[i].x);
    pt[i].uv_fac = totlen;
  }
}

void BKE_gpencil_stroke_geometry_update(bGPdata */*gpd*/, bGPDstroke *gps)
{
  if (gps == nullptr) {
    return;
  }

  if (gps->totpoints > 2) {
    BKE_gpencil_stroke_fill_triangulate(gps);
  }
  else {
    gps->tot_triangles = 0;
    MEM_SAFE_FREE(gps->triangles);
  }

  /* calc uv data along the stroke */
  BKE_gpencil_stroke_uv_update(gps);

  /* Calc stroke bounding box. */
  BKE_gpencil_stroke_boundingbox_calc(gps);
}

float BKE_gpencil_stroke_length(const bGPDstroke *gps, bool use_3d)
{
  if (!gps->points || gps->totpoints < 2) {
    return 0.0f;
  }
  float *last_pt = &gps->points[0].x;
  float total_length = 0.0f;
  for (int i = 1; i < gps->totpoints; i++) {
    bGPDspoint *pt = &gps->points[i];
    if (use_3d) {
      total_length += len_v3v3(&pt->x, last_pt);
    }
    else {
      total_length += len_v2v2(&pt->x, last_pt);
    }
    last_pt = &pt->x;
  }
  return total_length;
}

float BKE_gpencil_stroke_segment_length(const bGPDstroke *gps,
                                        const int start_index,
                                        const int end_index,
                                        bool use_3d)
{
  if (!gps->points || gps->totpoints < 2 || end_index <= start_index) {
    return 0.0f;
  }

  int index = std::max(start_index, 0) + 1;
  int last_index = std::min(end_index, gps->totpoints - 1) + 1;

  float *last_pt = &gps->points[index - 1].x;
  float total_length = 0.0f;
  for (int i = index; i < last_index; i++) {
    bGPDspoint *pt = &gps->points[i];
    if (use_3d) {
      total_length += len_v3v3(&pt->x, last_pt);
    }
    else {
      total_length += len_v2v2(&pt->x, last_pt);
    }
    last_pt = &pt->x;
  }
  return total_length;
}

bool BKE_gpencil_stroke_trim(bGPdata *gpd, bGPDstroke *gps)
{
  if (gps->totpoints < 4) {
    return false;
  }
  bool intersect = false;
  int start = 0;
  int end = 0;
  float point[3];
  /* loop segments from start until we have an intersection */
  for (int i = 0; i < gps->totpoints - 2; i++) {
    start = i;
    bGPDspoint *a = &gps->points[start];
    bGPDspoint *b = &gps->points[start + 1];
    for (int j = start + 2; j < gps->totpoints - 1; j++) {
      end = j + 1;
      bGPDspoint *c = &gps->points[j];
      bGPDspoint *d = &gps->points[end];
      float pointb[3];
      /* get intersection */
      if (isect_line_line_v3(&a->x, &b->x, &c->x, &d->x, point, pointb)) {
        if (len_v3(point) > 0.0f) {
          float closest[3];
          /* check intersection is on both lines */
          float lambda = closest_to_line_v3(closest, point, &a->x, &b->x);
          if ((lambda <= 0.0f) || (lambda >= 1.0f)) {
            continue;
          }
          lambda = closest_to_line_v3(closest, point, &c->x, &d->x);
          if ((lambda <= 0.0f) || (lambda >= 1.0f)) {
            continue;
          }

          intersect = true;
          break;
        }
      }
    }
    if (intersect) {
      break;
    }
  }

  /* trim unwanted points */
  if (intersect) {

    /* save points */
    bGPDspoint *old_points = (bGPDspoint *)MEM_dupallocN(gps->points);
    MDeformVert *old_dvert = nullptr;
    MDeformVert *dvert_src = nullptr;

    if (gps->dvert != nullptr) {
      old_dvert = (MDeformVert *)MEM_dupallocN(gps->dvert);
    }

    /* resize gps */
    int newtot = end - start + 1;

    gps->points = (bGPDspoint *)MEM_recallocN(gps->points, sizeof(*gps->points) * newtot);
    if (gps->dvert != nullptr) {
      gps->dvert = (MDeformVert *)MEM_recallocN(gps->dvert, sizeof(*gps->dvert) * newtot);
    }

    for (int i = 0; i < newtot; i++) {
      int idx = start + i;
      bGPDspoint *pt_src = &old_points[idx];
      bGPDspoint *pt_new = &gps->points[i];
      *pt_new = blender::dna::shallow_copy(*pt_src);
      if (gps->dvert != nullptr) {
        dvert_src = &old_dvert[idx];
        MDeformVert *dvert = &gps->dvert[i];
        memcpy(dvert, dvert_src, sizeof(MDeformVert));
        if (dvert_src->dw) {
          memcpy(dvert->dw, dvert_src->dw, sizeof(MDeformWeight));
        }
      }
      if (ELEM(idx, start, end)) {
        copy_v3_v3(&pt_new->x, point);
      }
    }

    gps->totpoints = newtot;

    MEM_SAFE_FREE(old_points);
    MEM_SAFE_FREE(old_dvert);
  }

  BKE_gpencil_stroke_geometry_update(gpd, gps);

  return intersect;
}

bool BKE_gpencil_stroke_close(bGPDstroke *gps)
{
  bGPDspoint *pt1 = nullptr;
  bGPDspoint *pt2 = nullptr;

  /* Only can close a stroke with 3 points or more. */
  if (gps->totpoints < 3) {
    return false;
  }

  /* Calc average distance between points to get same level of sampling. */
  float dist_tot = 0.0f;
  for (int i = 0; i < gps->totpoints - 1; i++) {
    pt1 = &gps->points[i];
    pt2 = &gps->points[i + 1];
    dist_tot += len_v3v3(&pt1->x, &pt2->x);
  }
  /* Calc the average distance. */
  float dist_avg = dist_tot / (gps->totpoints - 1);

  /* Calc distance between last and first point. */
  pt1 = &gps->points[gps->totpoints - 1];
  pt2 = &gps->points[0];
  float dist_close = len_v3v3(&pt1->x, &pt2->x);

  /* if the distance to close is very small, don't need add points and just enable cyclic. */
  if (dist_close <= dist_avg) {
    gps->flag |= GP_STROKE_CYCLIC;
    return true;
  }

  /* Calc number of points required using the average distance. */
  int tot_newpoints = std::max<int>(dist_close / dist_avg, 1);

  /* Resize stroke array. */
  int old_tot = gps->totpoints;
  gps->totpoints += tot_newpoints;
  gps->points = (bGPDspoint *)MEM_recallocN(gps->points, sizeof(*gps->points) * gps->totpoints);
  if (gps->dvert != nullptr) {
    gps->dvert = (MDeformVert *)MEM_recallocN(gps->dvert, sizeof(*gps->dvert) * gps->totpoints);
  }

  /* Generate new points */
  pt1 = &gps->points[old_tot - 1];
  pt2 = &gps->points[0];
  bGPDspoint *pt = &gps->points[old_tot];
  for (int i = 1; i < tot_newpoints + 1; i++, pt++) {
    float step = (tot_newpoints > 1) ? (float(i) / float(tot_newpoints)) : 0.99f;
    /* Clamp last point to be near, but not on top of first point. */
    if ((tot_newpoints > 1) && (i == tot_newpoints)) {
      step *= 0.99f;
    }

    /* Average point. */
    interp_v3_v3v3(&pt->x, &pt1->x, &pt2->x, step);
    pt->pressure = interpf(pt2->pressure, pt1->pressure, step);
    pt->strength = interpf(pt2->strength, pt1->strength, step);
    pt->flag = 0;
    interp_v4_v4v4(pt->vert_color, pt1->vert_color, pt2->vert_color, step);
    /* Set point as selected. */
    if (gps->flag & GP_STROKE_SELECT) {
      pt->flag |= GP_SPOINT_SELECT;
    }

    /* Set weights. */
    if (gps->dvert != nullptr) {
      MDeformVert *dvert1 = &gps->dvert[old_tot - 1];
      MDeformWeight *dw1 = BKE_defvert_ensure_index(dvert1, 0);
      float weight_1 = dw1 ? dw1->weight : 0.0f;

      MDeformVert *dvert2 = &gps->dvert[0];
      MDeformWeight *dw2 = BKE_defvert_ensure_index(dvert2, 0);
      float weight_2 = dw2 ? dw2->weight : 0.0f;

      MDeformVert *dvert_final = &gps->dvert[old_tot + i - 1];
      dvert_final->totweight = 0;
      MDeformWeight *dw = BKE_defvert_ensure_index(dvert_final, 0);
      if (dvert_final->dw) {
        dw->weight = interpf(weight_2, weight_1, step);
      }
    }
  }

  /* Enable cyclic flag. */
  gps->flag |= GP_STROKE_CYCLIC;

  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Dissolve Points
 * \{ */

void BKE_gpencil_dissolve_points(bGPdata *gpd, bGPDframe *gpf, bGPDstroke *gps, const short tag)
{
  bGPDspoint *pt;
  MDeformVert *dvert = nullptr;
  int i;

  int tot = gps->totpoints; /* number of points in new buffer */
  /* first pass: count points to remove */
  /* Count how many points are selected (i.e. how many to remove) */
  for (i = 0, pt = gps->points; i < gps->totpoints; i++, pt++) {
    if (pt->flag & tag) {
      /* selected point - one of the points to remove */
      tot--;
    }
  }

  /* if no points are left, we simply delete the entire stroke */
  if (tot <= 0) {
    /* remove the entire stroke */
    if (gps->points) {
      MEM_freeN(gps->points);
    }
    if (gps->dvert) {
      BKE_gpencil_free_stroke_weights(gps);
      MEM_freeN(gps->dvert);
    }
    if (gps->triangles) {
      MEM_freeN(gps->triangles);
    }
    BLI_freelinkN(&gpf->strokes, gps);
  }
  else {
    /* just copy all points to keep into a smaller buffer */
    bGPDspoint *new_points = (bGPDspoint *)MEM_callocN(sizeof(bGPDspoint) * tot,
                                                       "new gp stroke points copy");
    bGPDspoint *npt = new_points;

    MDeformVert *new_dvert = nullptr;
    MDeformVert *ndvert = nullptr;

    if (gps->dvert != nullptr) {
      new_dvert = (MDeformVert *)MEM_callocN(sizeof(MDeformVert) * tot,
                                             "new gp stroke weights copy");
      ndvert = new_dvert;
    }

    (gps->dvert != nullptr) ? dvert = gps->dvert : nullptr;
    for (i = 0, pt = gps->points; i < gps->totpoints; i++, pt++) {
      if ((pt->flag & tag) == 0) {
        *npt = blender::dna::shallow_copy(*pt);
        npt++;

        if (gps->dvert != nullptr) {
          *ndvert = *dvert;
          ndvert->dw = (MDeformWeight *)MEM_dupallocN(dvert->dw);
          ndvert++;
        }
      }
      if (gps->dvert != nullptr) {
        dvert++;
      }
    }

    /* free the old buffer */
    if (gps->points) {
      MEM_freeN(gps->points);
    }
    if (gps->dvert) {
      BKE_gpencil_free_stroke_weights(gps);
      MEM_freeN(gps->dvert);
    }

    /* save the new buffer */
    gps->points = new_points;
    gps->dvert = new_dvert;
    gps->totpoints = tot;

    /* triangles cache needs to be recalculated */
    BKE_gpencil_stroke_geometry_update(gpd, gps);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Normal Calculation
 * \{ */

void BKE_gpencil_stroke_normal(const bGPDstroke *gps, float r_normal[3])
{
  if (gps->totpoints < 3) {
    zero_v3(r_normal);
    return;
  }

  bGPDspoint *points = gps->points;
  int totpoints = gps->totpoints;

  const bGPDspoint *pt0 = &points[0];
  const bGPDspoint *pt1 = &points[1];
  const bGPDspoint *pt3 = &points[int(totpoints * 0.75)];

  float vec1[3];
  float vec2[3];

  /* initial vector (p0 -> p1) */
  sub_v3_v3v3(vec1, &pt1->x, &pt0->x);

  /* point vector at 3/4 */
  sub_v3_v3v3(vec2, &pt3->x, &pt0->x);

  /* vector orthogonal to polygon plane */
  cross_v3_v3v3(r_normal, vec1, vec2);

  /* Normalize vector */
  normalize_v3(r_normal);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Stroke Simplify
 * \{ */

void BKE_gpencil_stroke_simplify_adaptive(bGPdata *gpd, bGPDstroke *gps, float epsilon)
{
  bGPDspoint *old_points = (bGPDspoint *)MEM_dupallocN(gps->points);
  int totpoints = gps->totpoints;
  char *marked = nullptr;
  char work;

  int start = 0;
  int end = gps->totpoints - 1;

  marked = (char *)MEM_callocN(totpoints, "GP marked array");
  marked[start] = 1;
  marked[end] = 1;

  work = 1;
  int totmarked = 0;
  /* while still reducing */
  while (work) {
    int ls, le;
    work = 0;

    ls = start;
    le = start + 1;

    /* while not over interval */
    while (ls < end) {
      int max_i = 0;
      /* divided to get more control */
      float max_dist = epsilon / 10.0f;

      /* find the next marked point */
      while (marked[le] == 0) {
        le++;
      }

      for (int i = ls + 1; i < le; i++) {
        float point_on_line[3];
        float dist;

        closest_to_line_segment_v3(
            point_on_line, &old_points[i].x, &old_points[ls].x, &old_points[le].x);

        dist = len_v3v3(point_on_line, &old_points[i].x);

        if (dist > max_dist) {
          max_dist = dist;
          max_i = i;
        }
      }

      if (max_i != 0) {
        work = 1;
        marked[max_i] = 1;
        totmarked++;
      }

      ls = le;
      le = ls + 1;
    }
  }
  (void)totmarked; /* Quiet set-but-unused warning (may be removed). */

  /* adding points marked */
  MDeformVert *old_dvert = nullptr;
  MDeformVert *dvert_src = nullptr;

  if (gps->dvert != nullptr) {
    old_dvert = (MDeformVert *)MEM_dupallocN(gps->dvert);
  }
  /* resize gps */
  int j = 0;
  for (int i = 0; i < totpoints; i++) {
    bGPDspoint *pt_src = &old_points[i];
    bGPDspoint *pt = &gps->points[j];

    if ((marked[i]) || (i == 0) || (i == totpoints - 1)) {
      *pt = blender::dna::shallow_copy(*pt_src);
      if (gps->dvert != nullptr) {
        dvert_src = &old_dvert[i];
        MDeformVert *dvert = &gps->dvert[j];
        memcpy(dvert, dvert_src, sizeof(MDeformVert));
        if (dvert_src->dw) {
          memcpy(dvert->dw, dvert_src->dw, sizeof(MDeformWeight));
        }
      }
      j++;
    }
    else {
      if (gps->dvert != nullptr) {
        dvert_src = &old_dvert[i];
        BKE_gpencil_free_point_weights(dvert_src);
      }
    }
  }

  gps->totpoints = j;

  /* Calc geometry data. */
  BKE_gpencil_stroke_geometry_update(gpd, gps);

  MEM_SAFE_FREE(old_points);
  MEM_SAFE_FREE(old_dvert);
  MEM_SAFE_FREE(marked);
}

void BKE_gpencil_stroke_simplify_fixed(bGPdata *gpd, bGPDstroke *gps)
{
  if (gps->totpoints < 4) {
    return;
  }

  /* save points */
  bGPDspoint *old_points = (bGPDspoint *)MEM_dupallocN(gps->points);
  MDeformVert *old_dvert = nullptr;
  MDeformVert *dvert_src = nullptr;

  if (gps->dvert != nullptr) {
    old_dvert = (MDeformVert *)MEM_dupallocN(gps->dvert);
  }

  /* resize gps */
  int newtot = (gps->totpoints - 2) / 2;
  if ((gps->totpoints % 2) != 0) {
    newtot++;
  }
  newtot += 2;

  gps->points = (bGPDspoint *)MEM_recallocN(gps->points, sizeof(*gps->points) * newtot);
  if (gps->dvert != nullptr) {
    gps->dvert = (MDeformVert *)MEM_recallocN(gps->dvert, sizeof(*gps->dvert) * newtot);
  }

  int j = 0;
  for (int i = 0; i < gps->totpoints; i++) {
    bGPDspoint *pt_src = &old_points[i];
    bGPDspoint *pt = &gps->points[j];

    if ((i == 0) || (i == gps->totpoints - 1) || ((i % 2) > 0.0)) {
      *pt = blender::dna::shallow_copy(*pt_src);
      if (gps->dvert != nullptr) {
        dvert_src = &old_dvert[i];
        MDeformVert *dvert = &gps->dvert[j];
        memcpy(dvert, dvert_src, sizeof(MDeformVert));
        if (dvert_src->dw) {
          memcpy(dvert->dw, dvert_src->dw, sizeof(MDeformWeight));
        }
      }
      j++;
    }
    else {
      if (gps->dvert != nullptr) {
        dvert_src = &old_dvert[i];
        BKE_gpencil_free_point_weights(dvert_src);
      }
    }
  }

  gps->totpoints = j;
  /* Calc geometry data. */
  BKE_gpencil_stroke_geometry_update(gpd, gps);

  MEM_SAFE_FREE(old_points);
  MEM_SAFE_FREE(old_dvert);
}

void BKE_gpencil_stroke_subdivide(bGPdata *gpd, bGPDstroke *gps, int level, int type)
{
  bGPDspoint *temp_points;
  MDeformVert *temp_dverts = nullptr;
  MDeformVert *dvert = nullptr;
  MDeformVert *dvert_final = nullptr;
  MDeformVert *dvert_next = nullptr;
  int totnewpoints, oldtotpoints;

  bool cyclic = (gps->flag & GP_STROKE_CYCLIC) != 0;

  for (int s = 0; s < level; s++) {
    totnewpoints = gps->totpoints;
    if (!cyclic) {
      totnewpoints--;
    }
    /* duplicate points in a temp area */
    temp_points = gps->points;
    oldtotpoints = gps->totpoints;

    /* resize the points arrays */
    gps->totpoints += totnewpoints;
    gps->points = (bGPDspoint *)MEM_malloc_arrayN(gps->totpoints, sizeof(*gps->points), __func__);
    if (gps->dvert != nullptr) {
      temp_dverts = gps->dvert;
      gps->dvert = (MDeformVert *)MEM_malloc_arrayN(gps->totpoints, sizeof(*gps->dvert), __func__);
    }

    /* move points from last to first to new place */
    for (int i = 0; i < oldtotpoints; i++) {
      bGPDspoint *pt = &temp_points[i];
      bGPDspoint *pt_final = &gps->points[i * 2];

      copy_v3_v3(&pt_final->x, &pt->x);
      pt_final->pressure = pt->pressure;
      pt_final->strength = pt->strength;
      pt_final->uv_rot = pt->uv_rot;
      pt_final->uv_fac = pt->uv_fac;
      pt_final->time = pt->time;
      pt_final->flag = pt->flag;
      pt_final->runtime.pt_orig = pt->runtime.pt_orig;
      pt_final->runtime.idx_orig = pt->runtime.idx_orig;
      copy_v4_v4(pt_final->vert_color, pt->vert_color);
      copy_v4_v4(pt_final->uv_fill, pt->uv_fill);

      if (gps->dvert != nullptr) {
        dvert = &temp_dverts[i];
        dvert_final = &gps->dvert[i * 2];
        dvert_final->totweight = dvert->totweight;
        dvert_final->dw = dvert->dw;
      }
    }
    /* interpolate mid points */
    for (int i = cyclic ? 0 : 1, j = cyclic ? oldtotpoints - 1 : 0; i < oldtotpoints; j = i, i++) {
      bGPDspoint *pt = &temp_points[j];
      bGPDspoint *next = &temp_points[i];
      bGPDspoint *pt_final = &gps->points[j * 2 + 1];

      /* add a half way point */
      interp_v3_v3v3(&pt_final->x, &pt->x, &next->x, 0.5f);
      pt_final->pressure = interpf(pt->pressure, next->pressure, 0.5f);
      pt_final->strength = interpf(pt->strength, next->strength, 0.5f);
      pt_final->uv_rot = interpf(pt->uv_rot, next->uv_rot, 0.5f);
      pt_final->uv_fac = interpf(pt->uv_fac, next->uv_fac, 0.5f);
      interp_v4_v4v4(pt_final->uv_fill, pt->uv_fill, next->uv_fill, 0.5f);
      CLAMP(pt_final->strength, GPENCIL_STRENGTH_MIN, 1.0f);
      pt_final->time = 0;
      pt_final->runtime.pt_orig = nullptr;
      pt_final->flag = 0;
      interp_v4_v4v4(pt_final->vert_color, pt->vert_color, next->vert_color, 0.5f);

      if (gps->dvert != nullptr) {
        dvert = &temp_dverts[j];
        dvert_next = &temp_dverts[i];
        dvert_final = &gps->dvert[j * 2 + 1];

        dvert_final->totweight = dvert->totweight;
        dvert_final->dw = (MDeformWeight *)MEM_dupallocN(dvert->dw);

        /* interpolate weight values */
        for (int d = 0; d < dvert->totweight; d++) {
          MDeformWeight *dw_a = &dvert->dw[d];
          if (dvert_next->totweight > d) {
            MDeformWeight *dw_b = &dvert_next->dw[d];
            MDeformWeight *dw_final = &dvert_final->dw[d];
            dw_final->weight = interpf(dw_a->weight, dw_b->weight, 0.5f);
          }
        }
      }
    }

    MEM_SAFE_FREE(temp_points);
    MEM_SAFE_FREE(temp_dverts);

    /* Move points to smooth stroke (not simple type). */
    if (type != GP_SUBDIV_SIMPLE) {
      float mid[3];
      /* extreme points are not changed */
      for (int i = cyclic ? 0 : 2, j = cyclic ? gps->totpoints - 2 : 0; i < gps->totpoints - 2;
           j = i, i += 2)
      {
        bGPDspoint *prev = &gps->points[j + 1];
        bGPDspoint *pt = &gps->points[i];
        bGPDspoint *next = &gps->points[i + 1];

        /* move point */
        interp_v3_v3v3(mid, &prev->x, &next->x, 0.5f);
        interp_v3_v3v3(&pt->x, mid, &pt->x, 0.5f);
      }
    }
  }

  /* Calc geometry data. */
  BKE_gpencil_stroke_geometry_update(gpd, gps);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Merge by Distance
 * \{ */

void BKE_gpencil_stroke_merge_distance(bGPdata *gpd,
                                       bGPDframe *gpf,
                                       bGPDstroke *gps,
                                       const float threshold,
                                       const bool use_unselected)
{
  bGPDspoint *pt = nullptr;
  bGPDspoint *pt_next = nullptr;
  float tagged = false;
  /* Use square distance to speed up loop */
  const float th_square = threshold * threshold;
  /* Need to have something to merge. */
  if (gps->totpoints < 2) {
    return;
  }
  int i = 0;
  int step = 1;
  while ((i < gps->totpoints - 1) && (i + step < gps->totpoints)) {
    pt = &gps->points[i];
    if (pt->flag & GP_SPOINT_TAG) {
      i++;
      step = 1;
      continue;
    }
    pt_next = &gps->points[i + step];
    /* Do not recalc tagged points. */
    if (pt_next->flag & GP_SPOINT_TAG) {
      step++;
      continue;
    }
    /* Check if contiguous points are selected. */
    if (!use_unselected) {
      if (((pt->flag & GP_SPOINT_SELECT) == 0) || ((pt_next->flag & GP_SPOINT_SELECT) == 0)) {
        i++;
        step = 1;
        continue;
      }
    }
    float len_square = len_squared_v3v3(&pt->x, &pt_next->x);
    if (len_square <= th_square) {
      tagged = true;
      if (i != gps->totpoints - 1) {
        /* Tag second point for delete. */
        pt_next->flag |= GP_SPOINT_TAG;
      }
      else {
        pt->flag |= GP_SPOINT_TAG;
      }
      /* Jump to next pair of points, keeping first point segment equals. */
      step++;
    }
    else {
      /* Analyze next point. */
      i++;
      step = 1;
    }
  }

  /* Always untag extremes. */
  pt = &gps->points[0];
  pt->flag &= ~GP_SPOINT_TAG;
  pt = &gps->points[gps->totpoints - 1];
  pt->flag &= ~GP_SPOINT_TAG;

  /* Dissolve tagged points */
  if (tagged) {
    BKE_gpencil_dissolve_points(gpd, gpf, gps, GP_SPOINT_TAG);
  }

  /* Calc geometry data. */
  BKE_gpencil_stroke_geometry_update(gpd, gps);
}

void BKE_gpencil_transform(bGPdata *gpd, const float mat[4][4])
{
  if (gpd == nullptr) {
    return;
  }

  const float scalef = mat4_to_scale(mat);
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    /* FIXME: For now, we just skip parented layers.
     * Otherwise, we have to update each frame to find
     * the current parent position/effects.
     */
    if (gpl->parent) {
      continue;
    }

    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
        bGPDspoint *pt;
        int i;

        for (pt = gps->points, i = 0; i < gps->totpoints; pt++, i++) {
          mul_m4_v3(mat, &pt->x);
          pt->pressure *= scalef;
        }

        /* Distortion may mean we need to re-triangulate. */
        BKE_gpencil_stroke_geometry_update(gpd, gps);
      }
    }
  }
}

int BKE_gpencil_stroke_point_count(const bGPdata *gpd)
{
  int total_points = 0;

  if (gpd == nullptr) {
    return 0;
  }

  LISTBASE_FOREACH (const bGPDlayer *, gpl, &gpd->layers) {
    /* FIXME: For now, we just skip parented layers.
     * Otherwise, we have to update each frame to find
     * the current parent position/effects.
     */
    if (gpl->parent) {
      continue;
    }

    LISTBASE_FOREACH (const bGPDframe *, gpf, &gpl->frames) {
      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
        total_points += gps->totpoints;
      }
    }
  }
  return total_points;
}

void BKE_gpencil_point_coords_get(bGPdata *gpd, GPencilPointCoordinates *elem_data)
{
  if (gpd == nullptr) {
    return;
  }

  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    /* FIXME: For now, we just skip parented layers.
     * Otherwise, we have to update each frame to find
     * the current parent position/effects.
     */
    if (gpl->parent) {
      continue;
    }

    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
        bGPDspoint *pt;
        int i;

        for (pt = gps->points, i = 0; i < gps->totpoints; pt++, i++) {
          copy_v3_v3(elem_data->co, &pt->x);
          elem_data->pressure = pt->pressure;
          elem_data++;
        }
      }
    }
  }
}

void BKE_gpencil_point_coords_apply(bGPdata *gpd, const GPencilPointCoordinates *elem_data)
{
  if (gpd == nullptr) {
    return;
  }

  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    /* FIXME: For now, we just skip parented layers.
     * Otherwise, we have to update each frame to find
     * the current parent position/effects.
     */
    if (gpl->parent) {
      continue;
    }

    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
        bGPDspoint *pt;
        int i;

        for (pt = gps->points, i = 0; i < gps->totpoints; pt++, i++) {
          copy_v3_v3(&pt->x, elem_data->co);
          pt->pressure = elem_data->pressure;
          elem_data++;
        }

        /* Distortion may mean we need to re-triangulate. */
        BKE_gpencil_stroke_geometry_update(gpd, gps);
      }
    }
  }
}

void BKE_gpencil_point_coords_apply_with_mat4(bGPdata *gpd,
                                              const GPencilPointCoordinates *elem_data,
                                              const float mat[4][4])
{
  if (gpd == nullptr) {
    return;
  }

  const float scalef = mat4_to_scale(mat);
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    /* FIXME: For now, we just skip parented layers.
     * Otherwise, we have to update each frame to find
     * the current parent position/effects.
     */
    if (gpl->parent) {
      continue;
    }

    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
        bGPDspoint *pt;
        int i;

        for (pt = gps->points, i = 0; i < gps->totpoints; pt++, i++) {
          mul_v3_m4v3(&pt->x, mat, elem_data->co);
          pt->pressure = elem_data->pressure * scalef;
          elem_data++;
        }

        /* Distortion may mean we need to re-triangulate. */
        BKE_gpencil_stroke_geometry_update(gpd, gps);
      }
    }
  }
}

void BKE_gpencil_stroke_set_random_color(bGPDstroke *gps)
{
  BLI_assert(gps->totpoints > 0);

  float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  bGPDspoint *pt = &gps->points[0];
  color[0] *= BLI_hash_int_01(BLI_hash_int_2d(gps->totpoints / 5, pt->x + pt->z));
  color[1] *= BLI_hash_int_01(BLI_hash_int_2d(gps->totpoints + pt->x, pt->y * pt->z + pt->x));
  color[2] *= BLI_hash_int_01(BLI_hash_int_2d(gps->totpoints - pt->x, pt->z * pt->x + pt->y));
  for (int i = 0; i < gps->totpoints; i++) {
    pt = &gps->points[i];
    copy_v4_v4(pt->vert_color, color);
  }
}

void BKE_gpencil_stroke_flip(bGPDstroke *gps)
{
  /* Reverse points. */
  BLI_array_reverse(gps->points, gps->totpoints);

  /* Reverse vertex groups if available. */
  if (gps->dvert) {
    BLI_array_reverse(gps->dvert, gps->totpoints);
  }
}

/* Temp data for storing information about an "island" of points
 * that should be kept when splitting up a stroke. Used in:
 * gpencil_stroke_delete_tagged_points()
 */
struct tGPDeleteIsland {
  int start_idx;
  int end_idx;
};

static void gpencil_stroke_join_islands(bGPdata *gpd,
                                        bGPDframe *gpf,
                                        bGPDstroke *gps_first,
                                        bGPDstroke *gps_last)
{
  bGPDspoint *pt = nullptr;
  bGPDspoint *pt_final = nullptr;
  const int totpoints = gps_first->totpoints + gps_last->totpoints;

  /* create new stroke */
  bGPDstroke *join_stroke = BKE_gpencil_stroke_duplicate(gps_first, false, true);

  join_stroke->points = (bGPDspoint *)MEM_callocN(sizeof(bGPDspoint) * totpoints, __func__);
  join_stroke->totpoints = totpoints;
  join_stroke->flag &= ~GP_STROKE_CYCLIC;

  /* copy points (last before) */
  int e1 = 0;
  int e2 = 0;
  float delta = 0.0f;

  for (int i = 0; i < totpoints; i++) {
    pt_final = &join_stroke->points[i];
    if (i < gps_last->totpoints) {
      pt = &gps_last->points[e1];
      e1++;
    }
    else {
      pt = &gps_first->points[e2];
      e2++;
    }

    /* copy current point */
    copy_v3_v3(&pt_final->x, &pt->x);
    pt_final->pressure = pt->pressure;
    pt_final->strength = pt->strength;
    pt_final->time = delta;
    pt_final->flag = pt->flag;
    copy_v4_v4(pt_final->vert_color, pt->vert_color);

    /* retiming with fixed time interval (we cannot determine real time) */
    delta += 0.01f;
  }

  /* Copy over vertex weight data (if available) */
  if ((gps_first->dvert != nullptr) || (gps_last->dvert != nullptr)) {
    join_stroke->dvert = (MDeformVert *)MEM_callocN(sizeof(MDeformVert) * totpoints, __func__);
    MDeformVert *dvert_src = nullptr;
    MDeformVert *dvert_dst = nullptr;

    /* Copy weights (last before). */
    e1 = 0;
    e2 = 0;
    for (int i = 0; i < totpoints; i++) {
      dvert_dst = &join_stroke->dvert[i];
      dvert_src = nullptr;
      if (i < gps_last->totpoints) {
        if (gps_last->dvert) {
          dvert_src = &gps_last->dvert[e1];
          e1++;
        }
      }
      else {
        if (gps_first->dvert) {
          dvert_src = &gps_first->dvert[e2];
          e2++;
        }
      }

      if ((dvert_src) && (dvert_src->dw)) {
        dvert_dst->dw = (MDeformWeight *)MEM_dupallocN(dvert_src->dw);
      }
    }
  }

  /* add new stroke at head */
  BLI_addhead(&gpf->strokes, join_stroke);
  /* Calc geometry data. */
  BKE_gpencil_stroke_geometry_update(gpd, join_stroke);

  /* remove first stroke */
  BLI_remlink(&gpf->strokes, gps_first);
  BKE_gpencil_free_stroke(gps_first);

  /* remove last stroke */
  BLI_remlink(&gpf->strokes, gps_last);
  BKE_gpencil_free_stroke(gps_last);
}

bGPDstroke *BKE_gpencil_stroke_delete_tagged_points(bGPdata *gpd,
                                                    bGPDframe *gpf,
                                                    bGPDstroke *gps,
                                                    bGPDstroke *next_stroke,
                                                    int tag_flags,
                                                    const bool select,
                                                    const bool flat_cap,
                                                    const int limit)
{
  /* The algorithm used here is as follows:
   * 1) We firstly identify the number of "islands" of non-tagged points
   *    which will all end up being in new strokes.
   *    - In the most extreme case (i.e. every other vert is a 1-vert island),
   *      we have at most `n / 2` islands
   *    - Once we start having larger islands than that, the number required
   *      becomes much less
   * 2) Each island gets converted to a new stroke
   * If the number of points is <= limit, the stroke is deleted. */

  tGPDeleteIsland *islands = (tGPDeleteIsland *)MEM_callocN(
      sizeof(tGPDeleteIsland) * (gps->totpoints + 1) / 2, "gp_point_islands");
  bool in_island = false;
  int num_islands = 0;

  bGPDstroke *new_stroke = nullptr;
  bGPDstroke *gps_first = nullptr;
  const bool is_cyclic = bool(gps->flag & GP_STROKE_CYCLIC);

  /* First Pass: Identify start/end of islands */
  bGPDspoint *pt = gps->points;
  for (int i = 0; i < gps->totpoints; i++, pt++) {
    if (pt->flag & tag_flags) {
      /* selected - stop accumulating to island */
      in_island = false;
    }
    else {
      /* unselected - start of a new island? */
      int idx;

      if (in_island) {
        /* extend existing island */
        idx = num_islands - 1;
        islands[idx].end_idx = i;
      }
      else {
        /* start of new island */
        in_island = true;
        num_islands++;

        idx = num_islands - 1;
        islands[idx].start_idx = islands[idx].end_idx = i;
      }
    }
  }

  /* Watch out for special case where No islands = All points selected = Delete Stroke only */
  if (num_islands) {
    /* There are islands, so create a series of new strokes,
     * adding them before the "next" stroke. */
    int idx;

    /* Create each new stroke... */
    for (idx = 0; idx < num_islands; idx++) {
      tGPDeleteIsland *island = &islands[idx];
      new_stroke = BKE_gpencil_stroke_duplicate(gps, false, true);
      if (flat_cap) {
        new_stroke->caps[1 - (idx % 2)] = GP_STROKE_CAP_FLAT;
      }

      /* if cyclic and first stroke, save to join later */
      if ((is_cyclic) && (gps_first == nullptr)) {
        gps_first = new_stroke;
      }

      new_stroke->flag &= ~GP_STROKE_CYCLIC;

      /* Compute new buffer size (+ 1 needed as the endpoint index is "inclusive") */
      new_stroke->totpoints = island->end_idx - island->start_idx + 1;

      /* Copy over the relevant point data */
      new_stroke->points = (bGPDspoint *)MEM_callocN(sizeof(bGPDspoint) * new_stroke->totpoints,
                                                     "gp delete stroke fragment");
      memcpy(static_cast<void *>(new_stroke->points),
             gps->points + island->start_idx,
             sizeof(bGPDspoint) * new_stroke->totpoints);

      /* Copy over vertex weight data (if available) */
      if (gps->dvert != nullptr) {
        /* Copy over the relevant vertex-weight points */
        new_stroke->dvert = (MDeformVert *)MEM_callocN(sizeof(MDeformVert) * new_stroke->totpoints,
                                                       "gp delete stroke fragment weight");
        memcpy(new_stroke->dvert,
               gps->dvert + island->start_idx,
               sizeof(MDeformVert) * new_stroke->totpoints);

        /* Copy weights */
        int e = island->start_idx;
        for (int i = 0; i < new_stroke->totpoints; i++) {
          MDeformVert *dvert_src = &gps->dvert[e];
          MDeformVert *dvert_dst = &new_stroke->dvert[i];
          if (dvert_src->dw) {
            dvert_dst->dw = (MDeformWeight *)MEM_dupallocN(dvert_src->dw);
          }
          e++;
        }
      }
      /* Each island corresponds to a new stroke.
       * We must adjust the timings of these new strokes:
       *
       * Each point's timing data is a delta from stroke's inittime, so as we erase some points
       * from the start of the stroke, we have to offset this inittime and all remaining points'
       * delta values. This way we get a new stroke with exactly the same timing as if user had
       * started drawing from the first non-removed point.
       */
      {
        bGPDspoint *pts;
        float delta = gps->points[island->start_idx].time;
        int j;

        new_stroke->inittime += double(delta);

        pts = new_stroke->points;
        for (j = 0; j < new_stroke->totpoints; j++, pts++) {
          /* Some points have time = 0, so check to not get negative time values. */
          pts->time = max_ff(pts->time - delta, 0.0f);
          /* set flag for select again later */
          if (select == true) {
            pts->flag &= ~GP_SPOINT_SELECT;
            pts->flag |= GP_SPOINT_TAG;
          }
        }
      }

      /* Add new stroke to the frame or delete if below limit */
      if ((limit > 0) && (new_stroke->totpoints <= limit)) {
        if (gps_first == new_stroke) {
          gps_first = nullptr;
        }
        BKE_gpencil_free_stroke(new_stroke);
      }
      else {
        /* Calc geometry data. */
        BKE_gpencil_stroke_geometry_update(gpd, new_stroke);

        if (next_stroke) {
          BLI_insertlinkbefore(&gpf->strokes, next_stroke, new_stroke);
        }
        else {
          BLI_addtail(&gpf->strokes, new_stroke);
        }
      }
    }
    /* if cyclic, need to join last stroke with first stroke */
    if ((is_cyclic) && (gps_first != nullptr) && (gps_first != new_stroke)) {
      gpencil_stroke_join_islands(gpd, gpf, gps_first, new_stroke);
    }
  }

  /* free islands */
  MEM_freeN(islands);

  /* Delete the old stroke */
  BLI_remlink(&gpf->strokes, gps);
  BKE_gpencil_free_stroke(gps);

  return new_stroke;
}

void BKE_gpencil_curve_delete_tagged_points(bGPdata *gpd,
                                            bGPDframe *gpf,
                                            bGPDstroke *gps,
                                            bGPDstroke *next_stroke,
                                            bGPDcurve *gpc,
                                            int tag_flags)
{
  if (gpc == nullptr) {
    return;
  }
  const bool is_cyclic = gps->flag & GP_STROKE_CYCLIC;
  const int idx_last = gpc->tot_curve_points - 1;
  bGPDstroke *gps_first = nullptr;
  bGPDstroke *gps_last = nullptr;

  int idx_start = 0;
  int idx_end = 0;
  bool prev_selected = gpc->curve_points[0].flag & tag_flags;
  for (int i = 1; i < gpc->tot_curve_points; i++) {
    bool selected = gpc->curve_points[i].flag & tag_flags;
    if (prev_selected == true && selected == false) {
      idx_start = i;
    }
    /* Island ends if the current point is selected or if we reached the end of the stroke */
    if ((prev_selected == false && selected == true) || (selected == false && i == idx_last)) {

      idx_end = selected ? i - 1 : i;
      int island_length = idx_end - idx_start + 1;

      /* If an island has only a single curve point, there is no curve segment, so skip island */
      if (island_length == 1) {
        if (is_cyclic) {
          if (idx_start > 0 && idx_end < idx_last) {
            prev_selected = selected;
            continue;
          }
        }
        else {
          prev_selected = selected;
          continue;
        }
      }

      bGPDstroke *new_stroke = BKE_gpencil_stroke_duplicate(gps, false, false);
      new_stroke->points = nullptr;
      new_stroke->flag &= ~GP_STROKE_CYCLIC;
      new_stroke->editcurve = BKE_gpencil_stroke_editcurve_new(island_length);

      if (gps_first == nullptr) {
        gps_first = new_stroke;
      }

      bGPDcurve *new_gpc = new_stroke->editcurve;
      memcpy(new_gpc->curve_points,
             gpc->curve_points + idx_start,
             sizeof(bGPDcurve_point) * island_length);

      BKE_gpencil_editcurve_recalculate_handles(new_stroke);
      new_stroke->flag |= GP_STROKE_NEEDS_CURVE_UPDATE;

      /* Calc geometry data. */
      BKE_gpencil_stroke_geometry_update(gpd, new_stroke);

      if (next_stroke) {
        BLI_insertlinkbefore(&gpf->strokes, next_stroke, new_stroke);
      }
      else {
        BLI_addtail(&gpf->strokes, new_stroke);
      }

      gps_last = new_stroke;
    }
    prev_selected = selected;
  }

  /* join first and last stroke if cyclic */
  if (is_cyclic && gps_first != nullptr && gps_last != nullptr && gps_first != gps_last) {
    bGPDcurve *gpc_first = gps_first->editcurve;
    bGPDcurve *gpc_last = gps_last->editcurve;
    int first_tot_points = gpc_first->tot_curve_points;
    int old_tot_points = gpc_last->tot_curve_points;

    gpc_last->tot_curve_points = first_tot_points + old_tot_points;
    gpc_last->curve_points = (bGPDcurve_point *)MEM_recallocN(
        gpc_last->curve_points, sizeof(bGPDcurve_point) * gpc_last->tot_curve_points);
    /* copy data from first to last */
    memcpy(gpc_last->curve_points + old_tot_points,
           gpc_first->curve_points,
           sizeof(bGPDcurve_point) * first_tot_points);

    BKE_gpencil_editcurve_recalculate_handles(gps_last);
    gps_last->flag |= GP_STROKE_NEEDS_CURVE_UPDATE;

    /* Calc geometry data. */
    BKE_gpencil_stroke_geometry_update(gpd, gps_last);

    /* remove first one */
    BLI_remlink(&gpf->strokes, gps_first);
    BKE_gpencil_free_stroke(gps_first);
  }

  /* Delete the old stroke */
  BLI_remlink(&gpf->strokes, gps);
  BKE_gpencil_free_stroke(gps);
}

/* Helper: copy point between strokes */
static void gpencil_stroke_copy_point(bGPDstroke *gps,
                                      MDeformVert *dvert,
                                      bGPDspoint *point,
                                      const float delta[3],
                                      float pressure,
                                      float strength,
                                      float deltatime)
{
  bGPDspoint *newpoint;

  gps->points = (bGPDspoint *)MEM_reallocN(gps->points, sizeof(bGPDspoint) * (gps->totpoints + 1));
  if (gps->dvert != nullptr) {
    gps->dvert = (MDeformVert *)MEM_reallocN(gps->dvert,
                                             sizeof(MDeformVert) * (gps->totpoints + 1));
  }
  else {
    /* If destination has weight add weight to origin. */
    if (dvert != nullptr) {
      gps->dvert = (MDeformVert *)MEM_callocN(sizeof(MDeformVert) * (gps->totpoints + 1),
                                              __func__);
    }
  }

  gps->totpoints++;
  newpoint = &gps->points[gps->totpoints - 1];

  newpoint->x = point->x * delta[0];
  newpoint->y = point->y * delta[1];
  newpoint->z = point->z * delta[2];
  newpoint->flag = point->flag;
  newpoint->pressure = pressure;
  newpoint->strength = strength;
  newpoint->time = point->time + deltatime;
  copy_v4_v4(newpoint->vert_color, point->vert_color);

  if (gps->dvert != nullptr) {
    MDeformVert *newdvert = &gps->dvert[gps->totpoints - 1];

    if (dvert != nullptr) {
      newdvert->totweight = dvert->totweight;
      newdvert->dw = (MDeformWeight *)MEM_dupallocN(dvert->dw);
    }
    else {
      newdvert->totweight = 0;
      newdvert->dw = nullptr;
    }
  }
}

void BKE_gpencil_stroke_join(bGPDstroke *gps_a,
                             bGPDstroke *gps_b,
                             const bool leave_gaps,
                             const bool fit_thickness,
                             const bool smooth,
                             bool auto_flip)
{
  bGPDspoint point;
  bGPDspoint *pt;
  int i;
  const float delta[3] = {1.0f, 1.0f, 1.0f};
  float deltatime = 0.0f;

  /* sanity checks */
  if (ELEM(nullptr, gps_a, gps_b)) {
    return;
  }

  if ((gps_a->totpoints == 0) || (gps_b->totpoints == 0)) {
    return;
  }

  if (auto_flip) {
    /* define start and end points of each stroke */
    float start_a[3], start_b[3], end_a[3], end_b[3];
    pt = &gps_a->points[0];
    copy_v3_v3(start_a, &pt->x);

    pt = &gps_a->points[gps_a->totpoints - 1];
    copy_v3_v3(end_a, &pt->x);

    pt = &gps_b->points[0];
    copy_v3_v3(start_b, &pt->x);

    pt = &gps_b->points[gps_b->totpoints - 1];
    copy_v3_v3(end_b, &pt->x);

    /* Check if need flip strokes. */
    float dist = len_squared_v3v3(end_a, start_b);
    bool flip_a = false;
    bool flip_b = false;
    float lowest = dist;

    dist = len_squared_v3v3(end_a, end_b);
    if (dist < lowest) {
      lowest = dist;
      flip_a = false;
      flip_b = true;
    }

    dist = len_squared_v3v3(start_a, start_b);
    if (dist < lowest) {
      lowest = dist;
      flip_a = true;
      flip_b = false;
    }

    dist = len_squared_v3v3(start_a, end_b);
    if (dist < lowest) {
      lowest = dist;
      flip_a = true;
      flip_b = true;
    }

    if (flip_a) {
      BKE_gpencil_stroke_flip(gps_a);
    }
    if (flip_b) {
      BKE_gpencil_stroke_flip(gps_b);
    }
  }

  /* don't visibly link the first and last points? */
  if (leave_gaps) {
    /* 1st: add one tail point to start invisible area */
    point = blender::dna::shallow_copy(gps_a->points[gps_a->totpoints - 1]);
    deltatime = point.time;

    gpencil_stroke_copy_point(gps_a, nullptr, &point, delta, 0.0f, 0.0f, 0.0f);

    /* 2nd: add one head point to finish invisible area */
    point = blender::dna::shallow_copy(gps_b->points[0]);
    gpencil_stroke_copy_point(gps_a, nullptr, &point, delta, 0.0f, 0.0f, deltatime);
  }

  /* Ratio to apply in the points to keep the same thickness in the joined stroke using the
   * destination stroke thickness. */
  const float ratio = (fit_thickness && gps_a->thickness > 0.0f) ?
                          float(gps_b->thickness) / float(gps_a->thickness) :
                          1.0f;

  /* 3rd: add all points */
  const int totpoints_a = gps_a->totpoints;
  for (i = 0, pt = gps_b->points; i < gps_b->totpoints && pt; i++, pt++) {
    MDeformVert *dvert = (gps_b->dvert) ? &gps_b->dvert[i] : nullptr;
    gpencil_stroke_copy_point(
        gps_a, dvert, pt, delta, pt->pressure * ratio, pt->strength, deltatime);
  }
  /* Smooth the join to avoid hard thickness changes. */
  if (smooth) {
    const int sample_points = 8;
    /* Get the segment to smooth using n points on each side of the join. */
    int start = std::max(0, totpoints_a - sample_points);
    int end = std::min(gps_a->totpoints - 1, start + (sample_points * 2));
    const int len = (end - start);
    float step = 1.0f / ((len / 2) + 1);

    /* Calc the average pressure. */
    float avg_pressure = 0.0f;
    for (i = start; i < end; i++) {
      pt = &gps_a->points[i];
      avg_pressure += pt->pressure;
    }
    avg_pressure = avg_pressure / len;

    /* Smooth segment thickness and position. */
    float ratio = step;
    for (i = start; i < end; i++) {
      pt = &gps_a->points[i];
      pt->pressure += (avg_pressure - pt->pressure) * ratio;
      BKE_gpencil_stroke_smooth_point(gps_a, i, ratio * 0.6f, 2, false, true, gps_a);

      ratio += step;
      /* In the center, reverse the ratio. */
      if (ratio > 1.0f) {
        ratio = ratio - step - step;
        step *= -1.0f;
      }
    }
  }
}

void BKE_gpencil_stroke_start_set(bGPDstroke *gps, int start_idx)
{
  if ((start_idx < 1) || (start_idx >= gps->totpoints) || (gps->totpoints < 2)) {
    return;
  }

  /* Only cyclic strokes. */
  if ((gps->flag & GP_STROKE_CYCLIC) == 0) {
    return;
  }

  bGPDstroke *gps_b = BKE_gpencil_stroke_duplicate(gps, true, false);
  BKE_gpencil_stroke_trim_points(gps_b, 0, start_idx - 1, true);
  BKE_gpencil_stroke_trim_points(gps, start_idx, gps->totpoints - 1, true);

  /* Join both strokes. */
  BKE_gpencil_stroke_join(gps, gps_b, false, false, false, false);

  BKE_gpencil_free_stroke(gps_b);
}

/** \} */
void BKE_gpencil_stroke_to_view_space(bGPDstroke *gps,
                                      float viewmat[4][4],
                                      const float diff_mat[4][4])
{
  for (int i = 0; i < gps->totpoints; i++) {
    bGPDspoint *pt = &gps->points[i];
    /* Point to parent space. */
    mul_v3_m4v3(&pt->x, diff_mat, &pt->x);
    /* point to view space */
    mul_m4_v3(viewmat, &pt->x);
  }
}

void BKE_gpencil_stroke_from_view_space(bGPDstroke *gps,
                                        float viewinv[4][4],
                                        const float diff_mat[4][4])
{
  float inverse_diff_mat[4][4];
  invert_m4_m4(inverse_diff_mat, diff_mat);

  for (int i = 0; i < gps->totpoints; i++) {
    bGPDspoint *pt = &gps->points[i];
    mul_v3_m4v3(&pt->x, viewinv, &pt->x);
    mul_m4_v3(inverse_diff_mat, &pt->x);
  }
}

/** \} */

float BKE_gpencil_stroke_average_pressure_get(bGPDstroke *gps)
{

  if (gps->totpoints == 1) {
    return gps->points[0].pressure;
  }

  float tot = 0.0f;
  for (int i = 0; i < gps->totpoints; i++) {
    const bGPDspoint *pt = &gps->points[i];
    tot += pt->pressure;
  }

  return tot / float(gps->totpoints);
}

bool BKE_gpencil_stroke_is_pressure_constant(bGPDstroke *gps)
{
  if (gps->totpoints == 1) {
    return true;
  }

  const float first_pressure = gps->points[0].pressure;
  for (int i = 0; i < gps->totpoints; i++) {
    const bGPDspoint *pt = &gps->points[i];
    if (pt->pressure != first_pressure) {
      return false;
    }
  }

  return true;
}

/** \} */
