/* SPDX-FileCopyrightText: Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spgraph
 */

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "BLI_array.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"
#include "BLI_timeit.hh"
#include "BLI_utildefines.h"
#include "BLI_vector.hh"

#include "DNA_anim_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_userdef_types.h"

#include "BKE_anim_data.hh"
#include "BKE_curve.hh"
#include "BKE_fcurve.hh"
#include "BKE_nla.h"

#include "GPU_immediate.h"
#include "GPU_matrix.h"
#include "GPU_state.h"
#include "GPU_vertex_buffer.h"

#include "ED_anim_api.hh"

#include "graph_intern.h"

#include "UI_interface.hh"
#include "UI_resources.hh"
#include "UI_view2d.hh"

#define MAX_VERTS 1 << 14

static void graph_draw_driver_debug(bAnimContext *ac, ID *id, FCurve *fcu);
using namespace blender;
struct KeyVertex {
  float2 pos;
  float size;
  ColorTheme4b color;
};

struct HandleLine {
  float2 a;
  ColorTheme4b color_a;
  float2 b;
  /* This is a duplicate of data, but the shader expects it this way. */
  ColorTheme4b color_b;
};

struct BuildArguments {
  int2 bounding_indices;
  const View2D *v2d;
  ID *id;
  bAnimContext *anim_context;
  bool draw_extrapolation;
  bool only_keys_of_selected_fcurves;
  bool only_handles_of_selected_keys;
  bool no_handles;
};

struct RenderBatch {
  GPUBatch *batch;
  GPUVertBuf *vert_buf;
  KeyVertex *verts;
  int num_verts;
};

struct FCurveRenderData {
  Array<KeyVertex> key_points;

  Vector<KeyVertex> key_handle_points;
  Vector<HandleLine> key_handle_lines;

  Vector<float2> line_points;
  float4 line_color;
  float line_thickness;
};

/* -------------------------------------------------------------------- */
/** \name Utility Drawing Defines
 * \{ */

/* determine the alpha value that should be used when
 * drawing components for some F-Curve (fcu)
 * - selected F-Curves should be more visible than partially visible ones
 */
static float fcurve_display_alpha(const FCurve *fcu)
{
  return (fcu->flag & FCURVE_SELECTED) ? 1.0f : U.fcu_inactive_alpha;
}

/** Get the first and last index to the bezt array that are just outside min and max. */
static blender::int2 get_bounding_bezt_indices(const FCurve *fcu, const float min, const float max)
{
  bool replace;
  int first, last;
  first = BKE_fcurve_bezt_binarysearch_index(fcu->bezt, min, fcu->totvert, &replace);
  first = clamp_i(first - 1, 0, fcu->totvert - 1);

  last = BKE_fcurve_bezt_binarysearch_index(fcu->bezt, max, fcu->totvert, &replace);
  last = replace ? last + 1 : last;
  last = clamp_i(last, 0, fcu->totvert - 1);
  return {first, last};
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name FCurve Modifier Drawing
 * \{ */

/* Envelope -------------- */

/* TODO: draw a shaded poly showing the region of influence too!!! */
/**
 * \param adt_nla_remap: Send nullptr if no NLA remapping necessary.
 */
static void draw_fcurve_modifier_controls_envelope(FModifier *fcm,
                                                   View2D *v2d,
                                                   AnimData *adt_nla_remap)
{
  FMod_Envelope *env = (FMod_Envelope *)fcm->data;
  FCM_EnvelopeData *fed;
  const float fac = 0.05f * BLI_rctf_size_x(&v2d->cur);
  int i;

  const uint shdr_pos = GPU_vertformat_attr_add(
      immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);

  GPU_line_width(1.0f);

  immBindBuiltinProgram(GPU_SHADER_3D_LINE_DASHED_UNIFORM_COLOR);

  float viewport_size[4];
  GPU_viewport_size_get_f(viewport_size);
  immUniform2f("viewport_size", viewport_size[2] / UI_SCALE_FAC, viewport_size[3] / UI_SCALE_FAC);

  immUniform1i("colors_len", 0); /* Simple dashes. */
  immUniformColor3f(0.0f, 0.0f, 0.0f);
  immUniform1f("dash_width", 10.0f);
  immUniform1f("udash_factor", 0.5f);

  /* draw two black lines showing the standard reference levels */

  immBegin(GPU_PRIM_LINES, 4);
  immVertex2f(shdr_pos, v2d->cur.xmin, env->midval + env->min);
  immVertex2f(shdr_pos, v2d->cur.xmax, env->midval + env->min);

  immVertex2f(shdr_pos, v2d->cur.xmin, env->midval + env->max);
  immVertex2f(shdr_pos, v2d->cur.xmax, env->midval + env->max);
  immEnd();

  immUnbindProgram();

  if (env->totvert > 0) {
    /* set size of vertices (non-adjustable for now) */
    GPU_point_size(2.0f);

    immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

    /* for now, point color is fixed, and is white */
    immUniformColor3f(1.0f, 1.0f, 1.0f);

    immBeginAtMost(GPU_PRIM_POINTS, env->totvert * 2);

    for (i = 0, fed = env->data; i < env->totvert; i++, fed++) {
      const float env_scene_time = BKE_nla_tweakedit_remap(
          adt_nla_remap, fed->time, NLATIME_CONVERT_MAP);

      /* only draw if visible
       * - min/max here are fixed, not relative
       */
      if (IN_RANGE(env_scene_time, (v2d->cur.xmin - fac), (v2d->cur.xmax + fac))) {
        immVertex2f(shdr_pos, env_scene_time, fed->min);
        immVertex2f(shdr_pos, env_scene_time, fed->max);
      }
    }

    immEnd();

    immUnbindProgram();
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name FCurve Modifier Drawing
 * \{ */

/* Points ---------------- */

/* helper func - set color to draw F-Curve data with */
static ColorTheme4b get_fcurve_vertex_color(const FCurve *fcu, bool sel)
{
  ColorTheme4b color;

  if (fcu->flag & FCURVE_PROTECTED) {
    /* Curve is locked */
    UI_GetThemeColorShade4ubv(TH_HEADER, 50, color);
  }
  else {
    UI_GetThemeColor4ubv(sel ? TH_VERTEX_SELECT : TH_VERTEX, color);
  }

  /* Fade the 'intensity' of the vertices based on the selection of the curves too
   * - Only fade by 50% the amount the curves were faded by, so that the points
   *   still stand out for easier selection
   */
  const float diff = 1.0f - fcurve_display_alpha(fcu);
  color[3] = 1.0f - (diff * 0.5f);
  CLAMP(color[3], 0.2f, 1.0f);

  return color;
}

/* Draw a cross at the given position. Shader must already be bound.
 * NOTE: the caller MUST HAVE GL_LINE_SMOOTH & GL_BLEND ENABLED, otherwise the controls don't
 * have a consistent appearance (due to off-pixel alignments).
 */
static void draw_cross(float position[2], float scale[2], uint attr_id)
{
  GPU_matrix_push();
  GPU_matrix_translate_2fv(position);
  GPU_matrix_scale_2f(1.0f / scale[0], 1.0f / scale[1]);

  /* Draw X shape. */
  const float line_length = 0.7f;
  immBegin(GPU_PRIM_LINES, 4);
  immVertex2f(attr_id, -line_length, -line_length);
  immVertex2f(attr_id, +line_length, +line_length);

  immVertex2f(attr_id, -line_length, +line_length);
  immVertex2f(attr_id, +line_length, -line_length);
  immEnd();

  GPU_matrix_pop();
}

/* Handles ---------------- */

static bool draw_fcurve_handles_check(SpaceGraph *sipo, FCurve *fcu)
{
  /* don't draw handle lines if handles are not to be shown */
  if (/* handles shouldn't be shown anywhere */
      (sipo->flag & SIPO_NOHANDLES) ||
      /* keyframes aren't editable */
      (fcu->flag & FCURVE_PROTECTED) ||
#if 0
      /* handles can still be selected and handle types set, better draw - campbell */
      /* editing the handles here will cause weird/incorrect interpolation issues */
      (fcu->flag & FCURVE_INT_VALUES) ||
#endif
      /* group that curve belongs to is not editable */
      ((fcu->grp) && (fcu->grp->flag & AGRP_PROTECTED)))
  {
    return false;
  }
  return true;
}

/* Samples ---------------- */

/* helper func - draw keyframe vertices only for an F-Curve */
static void draw_fcurve_samples(ARegion *region, FCurve *fcu, const float unit_scale)
{
  FPoint *first, *last;
  float scale[2];

  /* get view settings */
  const float hsize = UI_GetThemeValuef(TH_VERTEX_SIZE);
  UI_view2d_scale_get(&region->v2d, &scale[0], &scale[1]);

  scale[0] /= hsize;
  scale[1] /= hsize / unit_scale;

  /* get verts */
  first = fcu->fpt;
  last = (first) ? (first + (fcu->totvert - 1)) : (nullptr);

  /* draw */
  if (first && last) {
    /* anti-aliased lines for more consistent appearance */
    if (U.animation_flag & USER_ANIM_HIGH_QUALITY_DRAWING) {
      GPU_line_smooth(true);
    }
    GPU_blend(GPU_BLEND_ALPHA);

    uint pos = GPU_vertformat_attr_add(immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
    immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

    immUniformThemeColor((fcu->flag & FCURVE_SELECTED) ? TH_TEXT_HI : TH_TEXT);

    draw_cross(first->vec, scale, pos);
    draw_cross(last->vec, scale, pos);

    immUnbindProgram();

    GPU_blend(GPU_BLEND_NONE);
    if (U.animation_flag & USER_ANIM_HIGH_QUALITY_DRAWING) {
      GPU_line_smooth(false);
    }
  }
}

/* Curve ---------------- */

/* Helper func - just draw the F-Curve by sampling the visible region
 * (for drawing curves with modifiers). */
static void draw_fcurve_curve(bAnimContext *ac,
                              ID *id,
                              FCurve *fcu_,
                              View2D *v2d,
                              uint pos,
                              const bool use_nla_remap,
                              const bool draw_extrapolation)
{
  short mapping_flag = ANIM_get_normalization_flags(ac->sl);

  /* when opening a blend file on a different sized screen or while dragging the toolbar this can
   * happen best just bail out in this case. */
  if (UI_view2d_scale_get_x(v2d) <= 0.0f) {
    return;
  }

  /* disable any drivers */
  FCurve fcurve_for_draw = *fcu_;
  fcurve_for_draw.driver = nullptr;

  /* compute unit correction factor */
  float offset;
  float unitFac = ANIM_unit_mapping_get_factor(
      ac->scene, id, &fcurve_for_draw, mapping_flag, &offset);

  /* Note about sampling frequency:
   * Ideally, this is chosen such that we have 1-2 pixels = 1 segment
   * which means that our curves can be as smooth as possible. However,
   * this does mean that curves may not be fully accurate (i.e. if they have
   * sudden spikes which happen at the sampling point, we may have problems).
   * Also, this may introduce lower performance on less densely detailed curves,
   * though it is impossible to predict this from the modifiers!
   *
   * If the automatically determined sampling frequency is likely to cause an infinite
   * loop (i.e. too close to 0), then clamp it to a determined "safe" value. The value
   * chosen here is just the coarsest value which still looks reasonable.
   */

  /* TODO: perhaps we should have 1.0 frames
   * as upper limit so that curves don't get too distorted? */
  float pixels_per_sample = 1.5f;
  float samplefreq = pixels_per_sample / UI_view2d_scale_get_x(v2d);

  if (!(U.animation_flag & USER_ANIM_HIGH_QUALITY_DRAWING)) {
    /* Low Precision = coarse lower-bound clamping
     *
     * Although the "Beauty Draw" flag was originally for AA'd
     * line drawing, the sampling rate here has a much greater
     * impact on performance (e.g. for #40372)!
     *
     * This one still amounts to 10 sample-frames for each 1-frame interval
     * which should be quite a decent approximation in many situations.
     */
    if (samplefreq < 0.1f) {
      samplefreq = 0.1f;
    }
  }
  else {
    /* "Higher Precision" but slower - especially on larger windows (e.g. #40372) */
    if (samplefreq < 0.00001f) {
      samplefreq = 0.00001f;
    }
  }

  /* the start/end times are simply the horizontal extents of the 'cur' rect */
  float stime = v2d->cur.xmin;
  float etime = v2d->cur.xmax;

  AnimData *adt = use_nla_remap ? BKE_animdata_from_id(id) : nullptr;

  /* If not drawing extrapolation, then change fcurve drawing bounds to its keyframe bounds clamped
   * by graph editor bounds. */
  if (!draw_extrapolation) {
    float fcu_start = 0;
    float fcu_end = 0;
    BKE_fcurve_calc_range(fcu_, &fcu_start, &fcu_end, false);

    fcu_start = BKE_nla_tweakedit_remap(adt, fcu_start, NLATIME_CONVERT_MAP);
    fcu_end = BKE_nla_tweakedit_remap(adt, fcu_end, NLATIME_CONVERT_MAP);

    /* Account for reversed NLA strip effect. */
    if (fcu_end < fcu_start) {
      std::swap(fcu_start, fcu_end);
    }

    /* Clamp to graph editor rendering bounds. */
    stime = max_ff(stime, fcu_start);
    etime = min_ff(etime, fcu_end);
  }

  const int total_samples = roundf((etime - stime) / samplefreq);
  if (total_samples <= 0) {
    return;
  }

  /* NLA remapping is linear so we don't have to remap per iteration. */
  const float eval_start = BKE_nla_tweakedit_remap(adt, stime, NLATIME_CONVERT_UNMAP);
  const float eval_freq = BKE_nla_tweakedit_remap(adt, stime + samplefreq, NLATIME_CONVERT_UNMAP) -
                          eval_start;
  const float eval_end = BKE_nla_tweakedit_remap(adt, etime, NLATIME_CONVERT_UNMAP);

  immBegin(GPU_PRIM_LINE_STRIP, (total_samples + 1));

  /* At each sampling interval, add a new vertex.
   *
   * Apply the unit correction factor to the calculated values so that the displayed values appear
   * correctly in the viewport.
   */
  for (int i = 0; i < total_samples; i++) {
    const float ctime = stime + i * samplefreq;
    float eval_time = eval_start + i * eval_freq;

    /* Prevent drawing past bounds, due to floating point problems.
     * User-wise, prevent visual flickering.
     *
     * This is to cover the case where:
     * eval_start + total_samples * eval_freq > eval_end
     * due to floating point problems.
     */
    if (eval_time > eval_end) {
      eval_time = eval_end;
    }

    immVertex2f(pos, ctime, (evaluate_fcurve(&fcurve_for_draw, eval_time) + offset) * unitFac);
  }

  /* Ensure we include end boundary point.
   * User-wise, prevent visual flickering.
   *
   * This is to cover the case where:
   * eval_start + total_samples * eval_freq < eval_end
   * due to floating point problems.
   */
  immVertex2f(pos, etime, (evaluate_fcurve(&fcurve_for_draw, eval_end) + offset) * unitFac);

  immEnd();
}

/* helper func - draw a samples-based F-Curve */
static void draw_fcurve_curve_samples(bAnimContext *ac,
                                      ID *id,
                                      FCurve *fcu,
                                      View2D *v2d,
                                      const uint shdr_pos,
                                      const bool draw_extrapolation)
{
  if (!draw_extrapolation && fcu->totvert == 1) {
    return;
  }

  FPoint *prevfpt = fcu->fpt;
  FPoint *fpt = prevfpt + 1;
  float fac, v[2];
  int b = fcu->totvert;
  float unit_scale, offset;
  short mapping_flag = ANIM_get_normalization_flags(ac->sl);
  int count = fcu->totvert;

  const bool extrap_left = draw_extrapolation && prevfpt->vec[0] > v2d->cur.xmin;
  if (extrap_left) {
    count++;
  }

  const bool extrap_right = draw_extrapolation && (prevfpt + b - 1)->vec[0] < v2d->cur.xmax;
  if (extrap_right) {
    count++;
  }

  /* apply unit mapping */
  GPU_matrix_push();
  unit_scale = ANIM_unit_mapping_get_factor(ac->scene, id, fcu, mapping_flag, &offset);
  GPU_matrix_scale_2f(1.0f, unit_scale);
  GPU_matrix_translate_2f(0.0f, offset);

  immBegin(GPU_PRIM_LINE_STRIP, count);

  /* extrapolate to left? - left-side of view comes before first keyframe? */
  if (extrap_left) {
    v[0] = v2d->cur.xmin;

    /* y-value depends on the interpolation */
    if ((fcu->extend == FCURVE_EXTRAPOLATE_CONSTANT) || (fcu->flag & FCURVE_INT_VALUES) ||
        (fcu->totvert == 1))
    {
      /* just extend across the first keyframe's value */
      v[1] = prevfpt->vec[1];
    }
    else {
      /* extrapolate linear doesn't use the handle, use the next points center instead */
      fac = (prevfpt->vec[0] - fpt->vec[0]) / (prevfpt->vec[0] - v[0]);
      if (fac) {
        fac = 1.0f / fac;
      }
      v[1] = prevfpt->vec[1] - fac * (prevfpt->vec[1] - fpt->vec[1]);
    }

    immVertex2fv(shdr_pos, v);
  }

  /* loop over samples, drawing segments */
  /* draw curve between first and last keyframe (if there are enough to do so) */
  while (b--) {
    /* Linear interpolation: just add one point (which should add a new line segment) */
    immVertex2fv(shdr_pos, prevfpt->vec);

    /* get next pointers */
    if (b > 0) {
      prevfpt++;
    }
  }

  /* extrapolate to right? (see code for left-extrapolation above too) */
  if (extrap_right) {
    v[0] = v2d->cur.xmax;

    /* y-value depends on the interpolation */
    if ((fcu->extend == FCURVE_EXTRAPOLATE_CONSTANT) || (fcu->flag & FCURVE_INT_VALUES) ||
        (fcu->totvert == 1))
    {
      /* based on last keyframe's value */
      v[1] = prevfpt->vec[1];
    }
    else {
      /* extrapolate linear doesn't use the handle, use the previous points center instead */
      fpt = prevfpt - 1;
      fac = (prevfpt->vec[0] - fpt->vec[0]) / (prevfpt->vec[0] - v[0]);
      if (fac) {
        fac = 1.0f / fac;
      }
      v[1] = prevfpt->vec[1] - fac * (prevfpt->vec[1] - fpt->vec[1]);
    }

    immVertex2fv(shdr_pos, v);
  }

  immEnd();

  GPU_matrix_pop();
}

static int calculate_bezt_draw_resolution(BezTriple *bezt,
                                          BezTriple *prevbezt,
                                          const blender::float2 pixels_per_unit)
{
  const float points_per_pixel = 0.25f;
  const int resolution_x = int(((bezt->vec[1][0] - prevbezt->vec[1][0]) * pixels_per_unit[0]) *
                               points_per_pixel);
  /* Include the handles in the resolution calculation to cover the case where keys have the same
   * y-value, but their handles are offset to create an arc. */
  const float min_y = min_ffff(
      bezt->vec[1][1], bezt->vec[2][1], prevbezt->vec[1][1], prevbezt->vec[0][1]);
  const float max_y = max_ffff(
      bezt->vec[1][1], bezt->vec[2][1], prevbezt->vec[1][1], prevbezt->vec[0][1]);
  const int resolution_y = int(((max_y - min_y) * pixels_per_unit[1]) * points_per_pixel);

  /* Using a simple sum instead of calculating the diagonal. This gives a slightly higher
   * resolution but it does compensate for the fact that bezier curves can create long arcs between
   * keys. */
  return resolution_x + resolution_y;
}

/**
 * Add points on the bezier between `prevbezt` and `bezt` to `curve_vertices`.
 * The amount of points added is based on the given `resolution`.
 */
static void add_bezt_vertices(BezTriple *bezt,
                              BezTriple *prevbezt,
                              int resolution,
                              blender::Vector<blender::float2> &curve_vertices)
{
  if (resolution < 2) {
    curve_vertices.append({prevbezt->vec[1][0], prevbezt->vec[1][1]});
    return;
  }

  /* If the resolution goes too high the line will not end exactly at the keyframe. Probably due to
   * accumulating floating point issues in BKE_curve_forward_diff_bezier.*/
  resolution = min_ii(64, resolution);

  float prev_key[2], prev_handle[2], bez_handle[2], bez_key[2];
  /* Allocation needs +1 on resolution because BKE_curve_forward_diff_bezier uses it to iterate
   * inclusively. */
  float *bezier_diff_points = static_cast<float *>(
      MEM_mallocN(sizeof(float) * ((resolution + 1) * 2), "Draw bezt data"));

  prev_key[0] = prevbezt->vec[1][0];
  prev_key[1] = prevbezt->vec[1][1];
  prev_handle[0] = prevbezt->vec[2][0];
  prev_handle[1] = prevbezt->vec[2][1];

  bez_handle[0] = bezt->vec[0][0];
  bez_handle[1] = bezt->vec[0][1];
  bez_key[0] = bezt->vec[1][0];
  bez_key[1] = bezt->vec[1][1];

  BKE_fcurve_correct_bezpart(prev_key, prev_handle, bez_handle, bez_key);

  BKE_curve_forward_diff_bezier(prev_key[0],
                                prev_handle[0],
                                bez_handle[0],
                                bez_key[0],
                                bezier_diff_points,
                                resolution,
                                sizeof(float[2]));
  BKE_curve_forward_diff_bezier(prev_key[1],
                                prev_handle[1],
                                bez_handle[1],
                                bez_key[1],
                                bezier_diff_points + 1,
                                resolution,
                                sizeof(float[2]));

  for (float *fp = bezier_diff_points; resolution; resolution--, fp += 2) {
    const float x = *fp;
    const float y = *(fp + 1);
    curve_vertices.append({x, y});
  }
  MEM_freeN(bezier_diff_points);
}

static void add_extrapolation_point_left(const FCurve *fcu,
                                         const float v2d_xmin,
                                         blender::Vector<blender::float2> &curve_vertices)
{
  /* left-side of view comes before first keyframe, so need to extend as not cyclic */
  float vertex_position[2];
  vertex_position[0] = v2d_xmin;
  BezTriple *bezt = &fcu->bezt[0];

  /* y-value depends on the interpolation */
  if ((fcu->extend == FCURVE_EXTRAPOLATE_CONSTANT) || (bezt->ipo == BEZT_IPO_CONST) ||
      (bezt->ipo == BEZT_IPO_LIN && fcu->totvert == 1))
  {
    /* just extend across the first keyframe's value */
    vertex_position[1] = bezt->vec[1][1];
  }
  else if (bezt->ipo == BEZT_IPO_LIN) {
    BezTriple *next_bezt = bezt + 1;
    /* extrapolate linear doesn't use the handle, use the next points center instead */
    float fac = (bezt->vec[1][0] - next_bezt->vec[1][0]) / (bezt->vec[1][0] - vertex_position[0]);
    if (fac) {
      fac = 1.0f / fac;
    }
    vertex_position[1] = bezt->vec[1][1] - fac * (bezt->vec[1][1] - next_bezt->vec[1][1]);
  }
  else {
    /* based on angle of handle 1 (relative to keyframe) */
    float fac = (bezt->vec[0][0] - bezt->vec[1][0]) / (bezt->vec[1][0] - vertex_position[0]);
    if (fac) {
      fac = 1.0f / fac;
    }
    vertex_position[1] = bezt->vec[1][1] - fac * (bezt->vec[0][1] - bezt->vec[1][1]);
  }

  curve_vertices.append(vertex_position);
}

static void add_extrapolation_point_right(const FCurve *fcu,
                                          const float v2d_xmax,
                                          blender::Vector<blender::float2> &curve_vertices)
{
  float vertex_position[2];
  vertex_position[0] = v2d_xmax;
  BezTriple *bezt = &fcu->bezt[fcu->totvert - 1];

  /* y-value depends on the interpolation. */
  if ((fcu->extend == FCURVE_EXTRAPOLATE_CONSTANT) || (fcu->flag & FCURVE_INT_VALUES) ||
      (bezt->ipo == BEZT_IPO_CONST) || (bezt->ipo == BEZT_IPO_LIN && fcu->totvert == 1))
  {
    /* based on last keyframe's value */
    vertex_position[1] = bezt->vec[1][1];
  }
  else if (bezt->ipo == BEZT_IPO_LIN) {
    /* Extrapolate linear doesn't use the handle, use the previous points center instead. */
    BezTriple *prev_bezt = bezt - 1;
    float fac = (bezt->vec[1][0] - prev_bezt->vec[1][0]) / (bezt->vec[1][0] - vertex_position[0]);
    if (fac) {
      fac = 1.0f / fac;
    }
    vertex_position[1] = bezt->vec[1][1] - fac * (bezt->vec[1][1] - prev_bezt->vec[1][1]);
  }
  else {
    /* Based on angle of handle 1 (relative to keyframe). */
    float fac = (bezt->vec[2][0] - bezt->vec[1][0]) / (bezt->vec[1][0] - vertex_position[0]);
    if (fac) {
      fac = 1.0f / fac;
    }
    vertex_position[1] = bezt->vec[1][1] - fac * (bezt->vec[2][1] - bezt->vec[1][1]);
  }

  curve_vertices.append(vertex_position);
}

static blender::float2 calculate_pixels_per_unit(const View2D *v2d)
{
  const int window_width = BLI_rcti_size_x(&v2d->mask);
  const int window_height = BLI_rcti_size_y(&v2d->mask);

  const float v2d_frame_range = BLI_rctf_size_x(&v2d->cur);
  const float v2d_value_range = BLI_rctf_size_y(&v2d->cur);
  const blender::float2 pixels_per_unit = {window_width / v2d_frame_range,
                                           window_height / v2d_value_range};
  return pixels_per_unit;
}

static inline float calculate_pixel_distance(const rctf &bounds, const float2 &pixels_per_unit)
{
  return BLI_rctf_size_x(&bounds) * pixels_per_unit[0] +
         BLI_rctf_size_y(&bounds) * pixels_per_unit[1];
}

static inline void expand_key_bounds(const BezTriple *left_key,
                                     const BezTriple *right_key,
                                     rctf &bounds)
{
  bounds.xmax = right_key->vec[1][0];
  if (left_key->ipo == BEZT_IPO_BEZ) {
    /* Respect handles of bezier keys. */
    for (int i = 0; i < 3; i++) {
      if (right_key->vec[i][1] < bounds.ymin) {
        bounds.ymin = right_key->vec[i][1];
      }
      if (right_key->vec[i][1] > bounds.ymax) {
        bounds.ymax = right_key->vec[i][1];
      }
    }
  }
  else {
    if (right_key->vec[1][1] > bounds.ymax) {
      bounds.ymax = right_key->vec[1][1];
    }
    if (right_key->vec[1][1] < bounds.ymin) {
      bounds.ymin = right_key->vec[1][1];
    }
  }
}

static void draw_fcurve(bAnimContext *ac, SpaceGraph *sipo, ARegion *region, bAnimListElem *ale)
{
  FCurve *fcu = (FCurve *)ale->key_data;
  AnimData *adt = ANIM_nla_mapping_get(ac, ale);

  /* map keyframes for drawing if scaled F-Curve */
  ANIM_nla_mapping_apply_fcurve(adt, static_cast<FCurve *>(ale->key_data), false, false);

  /* 3) draw driver debugging stuff */
  if ((ac->datatype == ANIMCONT_DRIVERS) && (fcu->flag & FCURVE_ACTIVE)) {
    graph_draw_driver_debug(ac, ale->id, fcu);
  }

  /* undo mapping of keyframes for drawing if scaled F-Curve */
  if (adt) {
    ANIM_nla_mapping_apply_fcurve(adt, static_cast<FCurve *>(ale->key_data), true, false);
  }
}

/* Debugging -------------------------------- */

/* Draw indicators which show the value calculated from the driver,
 * and how this is mapped to the value that comes out of it. This
 * is handy for helping users better understand how to interpret
 * the graphs, and also facilitates debugging.
 */
static void graph_draw_driver_debug(bAnimContext *ac, ID *id, FCurve *fcu)
{
  ChannelDriver *driver = fcu->driver;
  View2D *v2d = &ac->region->v2d;
  short mapping_flag = ANIM_get_normalization_flags(ac->sl);
  float offset;
  float unitfac = ANIM_unit_mapping_get_factor(ac->scene, id, fcu, mapping_flag, &offset);

  const uint shdr_pos = GPU_vertformat_attr_add(
      immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_LINE_DASHED_UNIFORM_COLOR);

  float viewport_size[4];
  GPU_viewport_size_get_f(viewport_size);
  immUniform2f("viewport_size", viewport_size[2] / UI_SCALE_FAC, viewport_size[3] / UI_SCALE_FAC);

  immUniform1i("colors_len", 0); /* Simple dashes. */

  /* No curve to modify/visualize the result?
   * => We still want to show the 1-1 default...
   */
  if ((fcu->totvert == 0) && BLI_listbase_is_empty(&fcu->modifiers)) {
    float t;

    /* draw with thin dotted lines in style of what curve would have been */
    immUniformColor3fv(fcu->color);

    immUniform1f("dash_width", 40.0f);
    immUniform1f("udash_factor", 0.5f);
    GPU_line_width(2.0f);

    /* draw 1-1 line, stretching just past the screen limits
     * NOTE: we need to scale the y-values to be valid for the units
     */
    immBegin(GPU_PRIM_LINES, 2);

    t = v2d->cur.xmin;
    immVertex2f(shdr_pos, t, (t + offset) * unitfac);

    t = v2d->cur.xmax;
    immVertex2f(shdr_pos, t, (t + offset) * unitfac);

    immEnd();
  }

  /* draw driver only if actually functional */
  if ((driver->flag & DRIVER_FLAG_INVALID) == 0) {
    /* grab "coordinates" for driver outputs */
    float x = driver->curval;
    float y = fcu->curval * unitfac;

    /* Only draw indicators if the point is in range. */
    if (x >= v2d->cur.xmin) {
      float co[2];

      /* draw dotted lines leading towards this point from both axes ....... */
      immUniformColor3f(0.9f, 0.9f, 0.9f);
      immUniform1f("dash_width", 10.0f);
      immUniform1f("udash_factor", 0.5f);
      GPU_line_width(1.0f);

      immBegin(GPU_PRIM_LINES, (y <= v2d->cur.ymax) ? 4 : 2);

      /* x-axis lookup */
      co[0] = x;

      if (y <= v2d->cur.ymax) {
        co[1] = v2d->cur.ymax + 1.0f;
        immVertex2fv(shdr_pos, co);

        co[1] = y;
        immVertex2fv(shdr_pos, co);
      }

      /* y-axis lookup */
      co[1] = y;

      co[0] = v2d->cur.xmin - 1.0f;
      immVertex2fv(shdr_pos, co);

      co[0] = x;
      immVertex2fv(shdr_pos, co);

      immEnd();

      immUnbindProgram();

      /* GPU_PRIM_POINTS do not survive dashed line geometry shader... */
      immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

      /* x marks the spot .................................................... */
      /* -> outer frame */
      immUniformColor3f(0.9f, 0.9f, 0.9f);
      GPU_point_size(7.0);

      immBegin(GPU_PRIM_POINTS, 1);
      immVertex2f(shdr_pos, x, y);
      immEnd();

      /* inner frame */
      immUniformColor3f(0.9f, 0.0f, 0.0f);
      GPU_point_size(3.0);

      immBegin(GPU_PRIM_POINTS, 1);
      immVertex2f(shdr_pos, x, y);
      immEnd();
    }
  }

  immUnbindProgram();
}

/* Public Curve-Drawing API  ---------------- */

void graph_draw_ghost_curves(bAnimContext *ac, SpaceGraph *sipo, ARegion *region)
{
  /* draw with thick dotted lines */
  GPU_line_width(3.0f);

  /* anti-aliased lines for less jagged appearance */
  if (U.animation_flag & USER_ANIM_HIGH_QUALITY_DRAWING) {
    GPU_line_smooth(true);
  }
  GPU_blend(GPU_BLEND_ALPHA);

  const uint shdr_pos = GPU_vertformat_attr_add(
      immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);

  immBindBuiltinProgram(GPU_SHADER_3D_LINE_DASHED_UNIFORM_COLOR);

  float viewport_size[4];
  GPU_viewport_size_get_f(viewport_size);
  immUniform2f("viewport_size", viewport_size[2] / UI_SCALE_FAC, viewport_size[3] / UI_SCALE_FAC);

  immUniform1i("colors_len", 0); /* Simple dashes. */
  immUniform1f("dash_width", 20.0f);
  immUniform1f("udash_factor", 0.5f);

  /* Don't draw extrapolation on sampled ghost curves because it doesn't
   * match the curves they're ghosting anyway.
   * See issue #109920 for details. */
  const bool draw_extrapolation = false;
  /* the ghost curves are simply sampled F-Curves stored in sipo->runtime.ghost_curves */
  LISTBASE_FOREACH (FCurve *, fcu, &sipo->runtime.ghost_curves) {
    /* set whatever color the curve has set
     * - this is set by the function which creates these
     * - draw with a fixed opacity of 2
     */
    immUniformColor3fvAlpha(fcu->color, 0.5f);

    /* simply draw the stored samples */
    draw_fcurve_curve_samples(ac, nullptr, fcu, &region->v2d, shdr_pos, draw_extrapolation);
  }

  immUnbindProgram();

  if (U.animation_flag & USER_ANIM_HIGH_QUALITY_DRAWING) {
    GPU_line_smooth(false);
  }
  GPU_blend(GPU_BLEND_NONE);
}

/** \name Data build code
 * \{ */

static void build_keyframe_render_data(const FCurve *fcu,
                                       FCurveRenderData &fcu_render_data,
                                       const BuildArguments &args)
{
  if (args.only_keys_of_selected_fcurves && !(fcu->flag & FCURVE_SELECTED)) {
    fcu_render_data.key_points = Array<KeyVertex>();
    return;
  }

  float size = UI_GetThemeValuef(TH_VERTEX_SIZE) * UI_SCALE_FAC;
  if (fcu->flag & FCURVE_PROTECTED) {
    size *= 0.8f;
  }

  ColorTheme4b color_sel = get_fcurve_vertex_color(fcu, true);
  ColorTheme4b color_desel = get_fcurve_vertex_color(fcu, false);

  const int key_count = (args.bounding_indices[1] - args.bounding_indices[0]) + 1;
  fcu_render_data.key_points = Array<KeyVertex>(key_count);

  int array_index = 0;
  for (int i = args.bounding_indices[0]; i <= args.bounding_indices[1]; i++) {
    BezTriple &bezt = fcu->bezt[i];
    KeyVertex &key_vertex = fcu_render_data.key_points[array_index];
    key_vertex.pos = {bezt.vec[1][0], bezt.vec[1][1]};
    key_vertex.size = size;

    if (bezt.f2 & SELECT) {
      key_vertex.color = color_sel;
    }
    else {
      key_vertex.color = color_desel;
    }
    array_index++;
  }
}

static void build_key_handle_render_data(const FCurve *fcu,
                                         FCurveRenderData &fcu_render_data,
                                         const BuildArguments &args)
{
  fcu_render_data.key_handle_points = Vector<KeyVertex>();
  fcu_render_data.key_handle_lines = Vector<HandleLine>();

  if (args.no_handles) {
    return;
  }

  const float size = UI_GetThemeValuef(TH_VERTEX_SIZE) * UI_SCALE_FAC;

  const uchar alpha = fcurve_display_alpha(fcu) * 255;
  ColorTheme4b color_sel;
  UI_GetThemeColor4ubv(TH_HANDLE_VERTEX_SELECT, color_sel);
  color_sel[3] = alpha;

  ColorTheme4b color_desel;
  UI_GetThemeColor4ubv(TH_HANDLE_VERTEX, color_desel);
  color_desel[3] = alpha;

  int handle_count = 0;
  BezTriple *prevbezt = nullptr;
  for (int i = args.bounding_indices[0]; i <= args.bounding_indices[1]; i++) {
    BezTriple *bezt = &fcu->bezt[i];
    if (args.only_handles_of_selected_keys) {
      if (BEZT_ISSEL_ANY(bezt) == 0) {
        prevbezt = bezt;
        continue;
      }
    }

    KeyVertex handle_point;
    handle_point.size = size;
    HandleLine handle_line;
    handle_line.b = {bezt->vec[1][0], bezt->vec[1][1]};
    handle_line.color_a[3] = alpha;
    handle_line.color_b[3] = alpha;
    if ((!prevbezt && (bezt->ipo == BEZT_IPO_BEZ)) ||
        (prevbezt && (prevbezt->ipo == BEZT_IPO_BEZ)))
    {
      handle_point.pos = {bezt->vec[0][0], bezt->vec[0][1]};
      handle_line.a = {bezt->vec[0][0], bezt->vec[0][1]};
      if (bezt->f1 & SELECT) {
        handle_point.color = color_sel;
        UI_GetThemeColor3ubv(TH_HANDLE_SEL_FREE + bezt->h1, handle_line.color_a);
      }
      else {
        handle_point.color = color_desel;
        UI_GetThemeColor3ubv(TH_HANDLE_FREE + bezt->h1, handle_line.color_a);
      }
      fcu_render_data.key_handle_points.append(handle_point);
      handle_line.color_b = handle_line.color_a;
      fcu_render_data.key_handle_lines.append(handle_line);
      handle_count++;
    }

    if (bezt->ipo == BEZT_IPO_BEZ) {
      handle_point.pos = {bezt->vec[2][0], bezt->vec[2][1]};
      handle_line.a = {bezt->vec[2][0], bezt->vec[2][1]};
      if (bezt->f3 & SELECT) {
        handle_point.color = color_sel;
        UI_GetThemeColor3ubv(TH_HANDLE_SEL_FREE + bezt->h2, handle_line.color_a);
      }
      else {
        handle_point.color = color_desel;
        UI_GetThemeColor3ubv(TH_HANDLE_FREE + bezt->h2, handle_line.color_a);
      }
      fcu_render_data.key_handle_points.append(handle_point);
      handle_line.color_b = handle_line.color_a;
      fcu_render_data.key_handle_lines.append(handle_line);
      handle_count++;
    }
  }
}

static void build_line_render_data(FCurve *fcu,
                                   FCurveRenderData &fcu_render_data,
                                   BuildArguments &args)
{
  if (args.draw_extrapolation && fcu->totvert == 1) {
    return;
  }

  float offset;
  short mapping_flag = ANIM_get_normalization_flags(args.anim_context->sl);
  const float unit_scale = ANIM_unit_mapping_get_factor(
      args.anim_context->scene, args.id, fcu, mapping_flag, &offset);

  fcu_render_data.line_color = {
      fcu->color[0], fcu->color[1], fcu->color[2], fcurve_display_alpha(fcu)};
  fcu_render_data.line_points = Vector<float2>();

  if (fcu->flag & FCURVE_ACTIVE && !BKE_fcurve_is_protected(fcu)) {
    fcu_render_data.line_thickness = 2.5f;
  }
  else {
    fcu_render_data.line_thickness = 1.0f;
  }

  Vector<float2> &curve_vertices = fcu_render_data.line_points;

  /* Extrapolate to the left? */
  if (args.draw_extrapolation && fcu->bezt[0].vec[1][0] > args.v2d->cur.xmin) {
    add_extrapolation_point_left(fcu, args.v2d->cur.xmin, curve_vertices);
  }

  /* Always add the first point so the extrapolation line doesn't jump. */
  curve_vertices.append({fcu->bezt[args.bounding_indices[0]].vec[1][0],
                         fcu->bezt[args.bounding_indices[0]].vec[1][1]});

  const blender::float2 pixels_per_unit = calculate_pixels_per_unit(args.v2d);
  const int window_width = BLI_rcti_size_x(&args.v2d->mask);
  const float v2d_frame_range = BLI_rctf_size_x(&args.v2d->cur);
  const float pixel_width = v2d_frame_range / window_width;
  const float samples_per_pixel = 0.66f;
  const float evaluation_step = pixel_width / samples_per_pixel;

  BezTriple *first_key = &fcu->bezt[args.bounding_indices[0]];
  rctf key_bounds = {
      first_key->vec[1][0], first_key->vec[1][1], first_key->vec[1][0], first_key->vec[1][1]};
  /* Used when skipping keys. */
  bool has_skipped_keys = false;
  const float min_pixel_distance = 3.0f;

  /* Draw curve between first and last keyframe (if there are enough to do so). */
  for (int i = args.bounding_indices[0] + 1; i <= args.bounding_indices[1]; i++) {
    BezTriple *prevbezt = &fcu->bezt[i - 1];
    BezTriple *bezt = &fcu->bezt[i];
    expand_key_bounds(prevbezt, bezt, key_bounds);
    float pixel_distance = calculate_pixel_distance(key_bounds, pixels_per_unit);

    if (pixel_distance >= min_pixel_distance && has_skipped_keys) {
      /* When the pixel distance is greater than the threshold, and we've skipped at least one, add
       * a point. The point position is the average of all keys from INCLUDING prevbezt to
       * EXCLUDING bezt. prevbezt then gets reset to the key before bezt because the distance
       * between those is potentially below the threshold. */
      curve_vertices.append({BLI_rctf_cent_x(&key_bounds), BLI_rctf_cent_y(&key_bounds)});
      has_skipped_keys = false;
      key_bounds = {
          prevbezt->vec[1][0], prevbezt->vec[1][1], prevbezt->vec[1][0], prevbezt->vec[1][1]};
      expand_key_bounds(prevbezt, bezt, key_bounds);
      /* Calculate again based on the new prevbezt. */
      pixel_distance = calculate_pixel_distance(key_bounds, pixels_per_unit);
    }

    if (pixel_distance < min_pixel_distance) {
      /* Skip any keys that are too close to each other in screen space. */
      has_skipped_keys = true;
      continue;
    }

    switch (prevbezt->ipo) {

      case BEZT_IPO_CONST:
        /* Constant-Interpolation: draw segment between previous keyframe and next,
         * but holding same value */
        curve_vertices.append({prevbezt->vec[1][0], prevbezt->vec[1][1]});
        curve_vertices.append({bezt->vec[1][0], prevbezt->vec[1][1]});
        break;

      case BEZT_IPO_LIN:
        /* Linear interpolation: just add one point (which should add a new line segment) */
        curve_vertices.append({prevbezt->vec[1][0], prevbezt->vec[1][1]});
        break;

      case BEZT_IPO_BEZ: {
        const int resolution = calculate_bezt_draw_resolution(bezt, prevbezt, pixels_per_unit);
        add_bezt_vertices(bezt, prevbezt, resolution, curve_vertices);
        break;
      }

      default: {
        /* In case there is no other way to get curve points, evaluate the FCurve. */
        curve_vertices.append(prevbezt->vec[1]);
        float current_frame = prevbezt->vec[1][0] + evaluation_step;
        while (current_frame < bezt->vec[1][0]) {
          curve_vertices.append({current_frame, evaluate_fcurve(fcu, current_frame)});
          current_frame += evaluation_step;
        }
        break;
      }
    }

    prevbezt = bezt;
  }

  /* Always add the last point so the extrapolation line doesn't jump. */
  curve_vertices.append({fcu->bezt[args.bounding_indices[1]].vec[1][0],
                         fcu->bezt[args.bounding_indices[1]].vec[1][1]});

  /* Extrapolate to the right? (see code for left-extrapolation above too) */
  if (args.draw_extrapolation && fcu->bezt[fcu->totvert - 1].vec[1][0] < args.v2d->cur.xmax) {
    add_extrapolation_point_right(fcu, args.v2d->cur.xmax, curve_vertices);
  }
}

static void build_fcurve_render_data(FCurve *fcu,
                                     FCurveRenderData &fcu_render_data,
                                     ID *id,
                                     bAnimContext *anim_context,
                                     SpaceGraph *sipo)
{
  const View2D *v2d = &anim_context->region->v2d;
  const int2 bounding_indices = get_bounding_bezt_indices(fcu, v2d->cur.xmin, v2d->cur.xmax);
  BuildArguments args;
  args.bounding_indices = bounding_indices;
  args.draw_extrapolation = (sipo->flag & SIPO_NO_DRAW_EXTRAPOLATION) == 0;
  args.only_handles_of_selected_keys = sipo->flag & SIPO_SELVHANDLESONLY;
  args.no_handles = sipo->flag & SIPO_NOHANDLES;
  args.only_keys_of_selected_fcurves = U.animation_flag & USER_ANIM_ONLY_SHOW_SELECTED_CURVE_KEYS;
  args.v2d = v2d;
  args.id = id;
  args.anim_context = anim_context;

  build_line_render_data(fcu, fcu_render_data, args);
  build_keyframe_render_data(fcu, fcu_render_data, args);
  build_key_handle_render_data(fcu, fcu_render_data, args);
}

/** \} */

/** \name Drawing Code
 * \{ */

static void draw_fcurve_keys(Array<FCurveRenderData> &render_data)
{
  GPU_program_point_size(true);
  GPUIndexBufBuilder ibuf_builder;
  GPU_indexbuf_init(&ibuf_builder, GPU_PRIM_POINTS, MAX_VERTS, MAX_VERTS);
  for (uint i = 0; i < MAX_VERTS; i++) {
    GPU_indexbuf_add_point_vert(&ibuf_builder, i);
  }
  GPUIndexBuf *index_buffer = GPU_indexbuf_build(&ibuf_builder);

  GPUVertFormat format;
  GPU_vertformat_clear(&format);
  GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  GPU_vertformat_attr_add(&format, "size", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
  GPU_vertformat_attr_add(&format, "color", GPU_COMP_U8, 4, GPU_FETCH_INT_TO_FLOAT_UNIT);

  GPUVertBuf *vertex_buffer_keys = GPU_vertbuf_create_with_format_ex(&format, GPU_USAGE_STREAM);
  GPU_vertbuf_data_alloc(vertex_buffer_keys, MAX_VERTS);

  GPUBatch *batch_keys = GPU_batch_create_ex(GPU_PRIM_POINTS,
                                             vertex_buffer_keys,
                                             index_buffer,
                                             GPU_BATCH_OWNS_VBO | GPU_BATCH_OWNS_INDEX);
  GPU_batch_program_set_builtin(batch_keys, GPU_SHADER_3D_POINT_VARYING_SIZE_VARYING_COLOR);

  int verts_in_buffer = 0;
  KeyVertex *vertex_buffer_data = static_cast<KeyVertex *>(
      GPU_vertbuf_get_data(vertex_buffer_keys));
  for (const FCurveRenderData &fcu_render_data : render_data) {
    for (const KeyVertex &kv : fcu_render_data.key_points) {
      vertex_buffer_data[verts_in_buffer] = kv;
      verts_in_buffer++;
      if (verts_in_buffer >= MAX_VERTS) {
        GPU_vertbuf_tag_dirty(vertex_buffer_keys);
        GPU_vertbuf_use(vertex_buffer_keys);
        GPU_batch_draw(batch_keys);
        verts_in_buffer = 0;
      }
    }
  }

  if (verts_in_buffer != 0) {
    GPU_vertbuf_tag_dirty(vertex_buffer_keys);
    GPU_vertbuf_use(vertex_buffer_keys);
    GPU_batch_draw_range(batch_keys, 0, verts_in_buffer);
  }

  GPU_batch_discard(batch_keys);
  GPU_program_point_size(false);
}

static void draw_fcurve_lines(const Span<FCurveRenderData> &render_data)
{
  if (U.animation_flag & USER_ANIM_HIGH_QUALITY_DRAWING) {
    GPU_line_smooth(true);
  }
  GPU_blend(GPU_BLEND_ALPHA);

  GPUIndexBufBuilder ibuf_builder;
  GPU_indexbuf_init_ex(&ibuf_builder, GPU_PRIM_LINE_STRIP, MAX_VERTS, MAX_VERTS);
  for (uint i = 0; i < MAX_VERTS; i++) {
    GPU_indexbuf_add_generic_vert(&ibuf_builder, i);
  }
  GPUIndexBuf *index_buffer = GPU_indexbuf_build(&ibuf_builder);

  GPUVertFormat format;
  GPU_vertformat_clear(&format);
  GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);

  GPUVertBuf *vertex_buffer_lines = GPU_vertbuf_create_with_format_ex(&format, GPU_USAGE_STREAM);
  GPU_vertbuf_data_alloc(vertex_buffer_lines, MAX_VERTS);

  GPUBatch *batch_lines = GPU_batch_create_ex(GPU_PRIM_LINE_STRIP,
                                              vertex_buffer_lines,
                                              index_buffer,
                                              GPU_BATCH_OWNS_VBO | GPU_BATCH_OWNS_INDEX);
  GPU_batch_program_set_builtin(batch_lines, GPU_SHADER_3D_POLYLINE_UNIFORM_COLOR);

  GPU_shader_uniform_1f(batch_lines->shader, "lineWidth", GPU_line_width_get());
  float viewport_size[4];
  GPU_viewport_size_get_f(viewport_size);
  GPU_shader_uniform_2fv(batch_lines->shader, "viewport_size", &viewport_size[2]);

  int verts_in_buffer = 0;
  float2 *vertex_buffer_data = static_cast<float2 *>(GPU_vertbuf_get_data(vertex_buffer_lines));
  for (const FCurveRenderData &fcu_render_data : render_data) {
    const int32_t uniform_loc = GPU_shader_get_builtin_uniform(batch_lines->shader,
                                                               GPU_UNIFORM_COLOR);
    GPU_shader_uniform_float_ex(
        batch_lines->shader, uniform_loc, 4, 1, fcu_render_data.line_color);

    GPU_line_width(fcu_render_data.line_thickness);

    for (const float2 &p : fcu_render_data.line_points) {
      vertex_buffer_data[verts_in_buffer] = p;
      verts_in_buffer++;
      if (verts_in_buffer >= MAX_VERTS) {
        GPU_vertbuf_tag_dirty(vertex_buffer_lines);
        GPU_vertbuf_use(vertex_buffer_lines);
        GPU_batch_draw(batch_lines);
        verts_in_buffer = 0;
      }
    }

    /* Flush buffer after every line to make sure the line strip ends and is drawn with the right
     * color. */
    if (verts_in_buffer != 0) {
      GPU_vertbuf_tag_dirty(vertex_buffer_lines);
      GPU_vertbuf_use(vertex_buffer_lines);
      GPU_batch_draw_range(batch_lines, 0, verts_in_buffer);
    }
    verts_in_buffer = 0;
  }

  GPU_batch_discard(batch_lines);

  if (U.animation_flag & USER_ANIM_HIGH_QUALITY_DRAWING) {
    GPU_line_smooth(false);
  }
  GPU_blend(GPU_BLEND_NONE);
}

static void draw_fcurve_handle_lines(const Span<FCurveRenderData> &render_data)
{
  GPU_blend(GPU_BLEND_ALPHA);
  if (U.animation_flag & USER_ANIM_HIGH_QUALITY_DRAWING) {
    GPU_line_smooth(true);
  }

  GPUIndexBufBuilder ibuf_builder;
  GPU_indexbuf_init_ex(&ibuf_builder, GPU_PRIM_LINES, MAX_VERTS, MAX_VERTS);
  for (uint i = 0; i < MAX_VERTS; i += 2) {
    GPU_indexbuf_add_line_verts(&ibuf_builder, i, i + 1);
  }
  GPUIndexBuf *index_buffer = GPU_indexbuf_build(&ibuf_builder);

  GPUVertFormat format;
  GPU_vertformat_clear(&format);
  GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  GPU_vertformat_attr_add(&format, "color", GPU_COMP_U8, 4, GPU_FETCH_INT_TO_FLOAT_UNIT);

  GPUVertBuf *vertex_buffer_lines = GPU_vertbuf_create_with_format_ex(&format, GPU_USAGE_STREAM);
  GPU_vertbuf_data_alloc(vertex_buffer_lines, MAX_VERTS);

  GPUBatch *batch_lines = GPU_batch_create_ex(GPU_PRIM_LINES,
                                              vertex_buffer_lines,
                                              index_buffer,
                                              GPU_BATCH_OWNS_VBO | GPU_BATCH_OWNS_INDEX);
  GPU_batch_program_set_builtin(batch_lines, GPU_SHADER_3D_FLAT_COLOR);

  int verts_in_buffer = 0;
  int buffer_write_head = 0;
  HandleLine *vertex_buffer_data = static_cast<HandleLine *>(
      GPU_vertbuf_get_data(vertex_buffer_lines));
  for (const FCurveRenderData &fcu_render_data : render_data) {
    for (const HandleLine &handle_line : fcu_render_data.key_handle_lines) {
      vertex_buffer_data[buffer_write_head] = handle_line;
      buffer_write_head++;
      verts_in_buffer += 2;
      if (verts_in_buffer >= MAX_VERTS) {
        GPU_vertbuf_tag_dirty(vertex_buffer_lines);
        GPU_vertbuf_use(vertex_buffer_lines);
        GPU_batch_draw(batch_lines);
        buffer_write_head = 0;
        verts_in_buffer = 0;
      }
    }
  }

  if (verts_in_buffer != 0) {
    GPU_vertbuf_tag_dirty(vertex_buffer_lines);
    GPU_vertbuf_use(vertex_buffer_lines);
    GPU_batch_draw_range(batch_lines, 0, verts_in_buffer);
  }

  if (U.animation_flag & USER_ANIM_HIGH_QUALITY_DRAWING) {
    GPU_line_smooth(false);
  }
  GPU_blend(GPU_BLEND_NONE);
}

static void draw_fcurve_handle_points(const Span<FCurveRenderData> &render_data)
{
  GPU_program_point_size(true);
  GPUIndexBufBuilder ibuf_builder;
  GPU_indexbuf_init_ex(&ibuf_builder, GPU_PRIM_POINTS, MAX_VERTS, MAX_VERTS);
  for (uint i = 0; i < MAX_VERTS; i++) {
    GPU_indexbuf_add_generic_vert(&ibuf_builder, i);
  }
  GPUIndexBuf *index_buffer = GPU_indexbuf_build(&ibuf_builder);

  GPUVertFormat format;
  GPU_vertformat_clear(&format);
  GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  GPU_vertformat_attr_add(&format, "size", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
  GPU_vertformat_attr_add(&format, "color", GPU_COMP_U8, 4, GPU_FETCH_INT_TO_FLOAT_UNIT);

  GPUVertBuf *vertex_buffer_handles = GPU_vertbuf_create_with_format_ex(&format, GPU_USAGE_STREAM);
  GPU_vertbuf_data_alloc(vertex_buffer_handles, MAX_VERTS);

  GPUBatch *batch_handles = GPU_batch_create_ex(GPU_PRIM_POINTS,
                                                vertex_buffer_handles,
                                                index_buffer,
                                                GPU_BATCH_OWNS_VBO | GPU_BATCH_OWNS_INDEX);
  GPU_batch_program_set_builtin(batch_handles, GPU_SHADER_3D_POINT_VARYING_SIZE_VARYING_COLOR);

  int verts_in_buffer = 0;
  KeyVertex *vertex_buffer_data = static_cast<KeyVertex *>(
      GPU_vertbuf_get_data(vertex_buffer_handles));
  for (const FCurveRenderData &fcu_render_data : render_data) {
    for (const KeyVertex &kv : fcu_render_data.key_handle_points) {
      vertex_buffer_data[verts_in_buffer] = kv;
      verts_in_buffer++;
      if (verts_in_buffer >= MAX_VERTS) {
        GPU_vertbuf_tag_dirty(vertex_buffer_handles);
        GPU_vertbuf_use(vertex_buffer_handles);
        GPU_batch_draw(batch_handles);
        verts_in_buffer = 0;
      }
    }
  }

  if (verts_in_buffer != 0) {
    GPU_vertbuf_tag_dirty(vertex_buffer_handles);
    GPU_vertbuf_use(vertex_buffer_handles);
    GPU_batch_draw_range(batch_handles, 0, verts_in_buffer);
  }

  GPU_batch_discard(batch_handles);

  GPU_program_point_size(false);
}

static void draw_fcurve_render_data(Array<FCurveRenderData> &render_data)
{
  draw_fcurve_lines(render_data);
  draw_fcurve_handle_lines(render_data);
  draw_fcurve_handle_points(render_data);
  draw_fcurve_keys(render_data);
}

/** \} */

static void free_fcurve_render_data(Array<FCurveRenderData> &render_data)
{
  /* No need? */
}

void graph_draw_curves(bAnimContext *ac, SpaceGraph *sipo, ARegion *region, short sel)
{
  SCOPED_TIMER_AVERAGED("draw_curves");
  ListBase anim_data = {nullptr, nullptr};

  int filter = (ANIMFILTER_DATA_VISIBLE | ANIMFILTER_CURVE_VISIBLE | ANIMFILTER_FCURVESONLY);
  filter |= ((sel) ? (ANIMFILTER_SEL) : (ANIMFILTER_UNSEL));
  const size_t fcurve_count = ANIM_animdata_filter(
      ac, &anim_data, eAnimFilter_Flags(filter), ac->data, eAnimCont_Types(ac->datatype));

  Array<bAnimListElem *> anim_list_elements = Array<bAnimListElem *>(fcurve_count);

  int list_index;
  LISTBASE_FOREACH_INDEX (bAnimListElem *, ale, &anim_data, list_index) {
    anim_list_elements[list_index] = ale;
  }

  Array<FCurveRenderData> render_data(fcurve_count);
  threading::parallel_for(anim_list_elements.index_range(), 64, [&](const IndexRange range) {
    for (const int i : range) {
      bAnimListElem *ale = anim_list_elements[i];
      AnimData *adt = ANIM_nla_mapping_get(ac, ale);
      FCurve *fcu = (FCurve *)ale->key_data;
      ANIM_nla_mapping_apply_fcurve(adt, fcu, false, false);
      build_fcurve_render_data(fcu, render_data[i], ale->id, ac, sipo);
      ANIM_nla_mapping_apply_fcurve(adt, fcu, true, false);
    }
  });

  draw_fcurve_render_data(render_data);

  free_fcurve_render_data(render_data);

  /* free list of curves */
  ANIM_animdata_freelist(&anim_data);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Channel List
 * \{ */

void graph_draw_channel_names(bContext *C,
                              bAnimContext *ac,
                              ARegion *region,
                              const ListBase /* bAnimListElem */ &anim_data)
{
  bAnimListElem *ale;

  View2D *v2d = &region->v2d;

  const float channel_step = ANIM_UI_get_channel_step();

  /* Loop through channels, and set up drawing depending on their type. */
  { /* first pass: just the standard GL-drawing for backdrop + text */
    size_t channel_index = 0;
    float ymax = ANIM_UI_get_first_channel_top(v2d);

    for (ale = static_cast<bAnimListElem *>(anim_data.first); ale;
         ale = ale->next, ymax -= channel_step, channel_index++)
    {
      const float ymin = ymax - ANIM_UI_get_channel_height();

      /* check if visible */
      if (IN_RANGE(ymin, v2d->cur.ymin, v2d->cur.ymax) ||
          IN_RANGE(ymax, v2d->cur.ymin, v2d->cur.ymax))
      {
        /* draw all channels using standard channel-drawing API */
        ANIM_channel_draw(ac, ale, ymin, ymax, channel_index);
      }
    }
  }
  { /* second pass: widgets */
    uiBlock *block = UI_block_begin(C, region, __func__, UI_EMBOSS);
    size_t channel_index = 0;
    float ymax = ANIM_UI_get_first_channel_top(v2d);

    /* set blending again, as may not be set in previous step */
    GPU_blend(GPU_BLEND_ALPHA);

    for (ale = static_cast<bAnimListElem *>(anim_data.first); ale;
         ale = ale->next, ymax -= channel_step, channel_index++)
    {
      const float ymin = ymax - ANIM_UI_get_channel_height();

      /* check if visible */
      if (IN_RANGE(ymin, v2d->cur.ymin, v2d->cur.ymax) ||
          IN_RANGE(ymax, v2d->cur.ymin, v2d->cur.ymax))
      {
        /* draw all channels using standard channel-drawing API */
        rctf channel_rect;
        BLI_rctf_init(&channel_rect, 0, v2d->cur.xmax - V2D_SCROLL_WIDTH, ymin, ymax);
        ANIM_channel_draw_widgets(C, ac, ale, block, &channel_rect, channel_index);
      }
    }

    UI_block_end(C, block);
    UI_block_draw(C, block);

    GPU_blend(GPU_BLEND_NONE);
  }
}

/** \} */
