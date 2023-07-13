/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BLI_index_mask.hh"
#include "BLI_index_range.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

#include "BKE_context.h"
#include "BKE_grease_pencil.hh"

#include "RNA_access.h"
#include "RNA_define.h"

#include "DEG_depsgraph.h"

#include "ED_curves.h"
#include "ED_grease_pencil.h"
#include "ED_screen.h"

#include "WM_api.h"

namespace blender::ed::greasepencil {

bool active_grease_pencil_poll(bContext *C)
{
  Object *object = CTX_data_active_object(C);
  if (object == nullptr || object->type != OB_GREASE_PENCIL) {
    return false;
  }
  return true;
}

bool editable_grease_pencil_poll(bContext *C)
{
  Object *object = CTX_data_active_object(C);
  if (object == nullptr || object->type != OB_GREASE_PENCIL) {
    return false;
  }
  if (!ED_operator_object_active_editable_ex(C, object)) {
    return false;
  }
  if ((object->mode & OB_MODE_EDIT) == 0) {
    return false;
  }
  return true;
}

bool editable_grease_pencil_point_selection_poll(bContext *C)
{
  if (!editable_grease_pencil_poll(C)) {
    return false;
  }

  /* Allowed: point and segment selection mode, not allowed: stroke selection mode. */
  ToolSettings *ts = CTX_data_tool_settings(C);
  return (ts->gpencil_selectmode_edit != GP_SELECTMODE_STROKE);
}

bool grease_pencil_painting_poll(bContext *C)
{
  if (!active_grease_pencil_poll(C)) {
    return false;
  }
  Object *object = CTX_data_active_object(C);
  if ((object->mode & OB_MODE_PAINT_GREASE_PENCIL) == 0) {
    return false;
  }
  ToolSettings *ts = CTX_data_tool_settings(C);
  if (!ts || !ts->gp_paint) {
    return false;
  }
  return true;
}

static void keymap_grease_pencil_editing(wmKeyConfig *keyconf)
{
  wmKeyMap *keymap = WM_keymap_ensure(keyconf, "Grease Pencil Edit Mode", 0, 0);
  keymap->poll = editable_grease_pencil_poll;
}

static void keymap_grease_pencil_painting(wmKeyConfig *keyconf)
{
  wmKeyMap *keymap = WM_keymap_ensure(keyconf, "Grease Pencil Paint Mode", 0, 0);
  keymap->poll = grease_pencil_painting_poll;
}

/* -------------------------------------------------------------------- */
/** \name Smooth Stroke Operator.
 * \{ */

template<typename T>
static Span<T> gaussian_blur_1D_ex(const IndexRange curve_points,
                                   const int iterations,
                                   const float influence,
                                   const bool smooth_ends,
                                   const bool keep_shape,
                                   const bool is_cyclic,
                                   MutableSpan<T> dst,
                                   const Span<T> src,
                                   const IndexMask &mask)
{
  /* 1D Gaussian-like smoothing function.
   *
   * Note : This is the same algorithm used by BKE_gpencil_stroke_smooth_point (Legacy)
   * Overview of the algorithm here and in the following smooth functions:
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
   *
   * This function uses a binomial kernel, which is the discrete version of gaussian blur.
   * The weight for a vertex at the relative index point_index is
   * w = nCr(n, j + n/2) / 2^n = (n/1 * (n-1)/2 * ... * (n-j-n/2)/(j+n/2)) / 2^n
   * All weights together sum up to 1
   * This is equivalent to doing multiple iterations of averaging neighbors,
   * where n = iterations * 2 and -n/2 <= j <= n/2
   *
   * Now the problem is that nCr(n, j + n/2) is very hard to compute for n > 500, since even
   * double precision isn't sufficient. A very good robust approximation for n > 20 is
   * nCr(n, j + n/2) / 2^n = sqrt(2/(pi*n)) * exp(-2*j*j/n)
   *
   * There is one more problem left: The old smooth algorithm was doing a more aggressive
   * smooth. To solve that problem, choose a different n/2, which does not match the range and
   * normalize the weights on finish. This may cause some artifacts at low values.
   *
   * keep_shape is a new option to stop the stroke from severely deforming.
   * It uses different partially negative weights.
   * w = 2 * (nCr(n, j + n/2) / 2^n) - (nCr(3*n, j + n) / 2^(3*n))
   *   ~ 2 * sqrt(2/(pi*n)) * exp(-2*j*j/n) - sqrt(2/(pi*3*n)) * exp(-2*j*j/(3*n))
   * All weights still sum up to 1.
   * Note these weights only work because the averaging is done in relative coordinates.
   */

  /* Avoid computation if the mask is empty. */
  if (mask.is_empty()) {
    return dst;
  }

  /* Weight Initialization. */
  const int n_half = keep_shape ? (iterations * iterations) / 8 + iterations :
                                  (iterations * iterations) / 4 + 2 * iterations + 12;
  double w = keep_shape ? 2.0 : 1.0;
  double w2 = keep_shape ?
                  (1.0 / M_SQRT3) * exp((2 * iterations * iterations) / double(n_half * 3)) :
                  0.0;
  Array<double> total_weight(mask.size(), 0.0);

  const int64_t first_pt = curve_points.first();
  const int64_t last_pt = curve_points.last();
  const int64_t total_points = curve_points.size();

  auto is_end_and_fixed = [smooth_ends, is_cyclic, first_pt, last_pt](int point_index) {
    return !smooth_ends && !is_cyclic && ((point_index == first_pt) || (point_index == last_pt));
  };

  /* Initialize at zero. */
  mask.foreach_index(GrainSize(256), [&](const int64_t point_index) {
    if (!is_end_and_fixed(point_index)) {
      dst[point_index] = T(0);
    }
  });

  for (int step = iterations; step > 0; step--) {

    mask.foreach_index(GrainSize(256), [&](const int64_t point_index, const int64_t mask_index) {
      /* Filter out endpoints if smooth ends is disabled. */
      if (is_end_and_fixed(point_index)) {
        return;
      }

      float w_before = float(w - w2);
      float w_after = float(w - w2);

      /* Compute the neighboring points. */
      int64_t before = point_index - step;
      int64_t after = point_index + step;
      if (is_cyclic) {
        before = ((before - first_pt) % total_points + total_points) % total_points + first_pt;
        after = (after - first_pt) % total_points + first_pt;
      }
      else {
        if (!smooth_ends && (before < first_pt)) {
          w_before *= -(before - first_pt) / float(point_index - first_pt);
        }
        before = std::max(before, first_pt);

        if (!smooth_ends && (after > last_pt)) {
          w_after *= (after - first_pt - (total_points - 1)) /
                     float(total_points - 1 - point_index + first_pt);
        }
        after = std::min(after, last_pt);
      }

      /* Add the neighboring values. */
      const T bval = src[before];
      const T aval = src[after];
      const T cval = src[point_index];

      dst[point_index] += (bval - cval) * w_before;
      dst[point_index] += (aval - cval) * w_after;

      /* Update the weight values. */
      total_weight[mask_index] += w_before;
      total_weight[mask_index] += w_after;
    });

    w *= (n_half + step) / double(n_half + 1 - step);
    w2 *= (n_half * 3 + step) / double(n_half * 3 + 1 - step);
  }

  /* Normalize the weights. */
  mask.foreach_index(GrainSize(256), [&](const int64_t point_index, const int64_t mask_index) {
    if (!is_end_and_fixed(point_index)) {
      total_weight[mask_index] += w - w2;
      dst[point_index] = src[point_index] +
                         influence * dst[point_index] / total_weight[mask_index];
    }
  });

  return dst.as_span();
}

Span<float3> gaussian_blur_1D_float3(const IndexRange curve_points,
                                     const int iterations,
                                     const float influence,
                                     const bool smooth_ends,
                                     const bool keep_shape,
                                     const bool is_cyclic,
                                     MutableSpan<float3> dst,
                                     const Span<float3> src,
                                     const IndexMask &mask)
{
  return gaussian_blur_1D_ex<float3>(
      curve_points, iterations, influence, smooth_ends, keep_shape, is_cyclic, dst, src, mask);
}

Span<float> gaussian_blur_1D_float(const IndexRange curve_points,
                                   const int iterations,
                                   const float influence,
                                   const bool smooth_ends,
                                   const bool keep_shape,
                                   const bool is_cyclic,
                                   MutableSpan<float> dst,
                                   const Span<float> src,
                                   const IndexMask &mask)
{
  return gaussian_blur_1D_ex<float>(
      curve_points, iterations, influence, smooth_ends, keep_shape, is_cyclic, dst, src, mask);
}

static int grease_pencil_stroke_smooth_exec(bContext *C, wmOperator *op)
{
  using namespace blender;
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  const int iterations = RNA_int_get(op->ptr, "iterations");
  const float influence = RNA_float_get(op->ptr, "factor");
  const bool keep_shape = RNA_boolean_get(op->ptr, "keep_shape");
  const bool smooth_ends = RNA_boolean_get(op->ptr, "smooth_ends");

  const bool smooth_position = RNA_boolean_get(op->ptr, "smooth_position");
  const bool smooth_radius = RNA_boolean_get(op->ptr, "smooth_radius");
  const bool smooth_opacity = RNA_boolean_get(op->ptr, "smooth_opacity");

  if (!(smooth_position || smooth_radius || smooth_opacity)) {
    /* If there's nothing to be done, then do nothing. */
    return OPERATOR_FINISHED;
  }

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int /*drawing_index*/, bke::greasepencil::Drawing &drawing) {
        /* Smooth all selected curves in the current drawing. */

        /* Curves geometry and attributes*/
        bke::CurvesGeometry &curves = drawing.strokes_for_write();
        /* Position. */
        Array<float3> curves_positions_copy;
        if (smooth_position && !curves.positions().is_empty()) {
          curves_positions_copy = curves.positions();
        }
        /* Opacity. */
        Array<float> curves_opacities_copy;
        if (smooth_opacity && drawing.opacities().is_span()) {
          curves_opacities_copy = drawing.opacities().get_internal_span();
        }
        /* Radius. */
        Array<float> curves_radii_copy;
        if (smooth_radius && drawing.radii().is_span()) {
          curves_radii_copy = drawing.radii().get_internal_span();
        }
        /* Cyclic. */
        const offset_indices::OffsetIndices<int> points_by_curve = curves.points_by_curve();
        const VArray<bool> cyclic = curves.cyclic();
        /* Selection. */
        bke::AttributeAccessor curves_attributes = curves.attributes();
        bke::AttributeReader<bool> selection_attribute = curves_attributes.lookup_or_default<bool>(
            ".selection", ATTR_DOMAIN_POINT, true);

        if (!ed::curves::has_anything_selected(selection_attribute.varray,
                                               curves.points_range()) ||
            (curves_positions_copy.is_empty() && curves_opacities_copy.is_empty() &&
             curves_radii_copy.is_empty()))
        {
          return;
        }

        threading::parallel_for(curves.curves_range(), 256, [&](const IndexRange range) {
          for (const int curve_i : range) {
            /* Smooth a single curve*/

            const IndexRange points = points_by_curve[curve_i];
            const bool is_cyclic = cyclic[curve_i];

            IndexMaskMemory memory;
            const IndexMask curve_mask = IndexMask::from_bools(
                points, selection_attribute.varray, memory);

            if (!curves_positions_copy.is_empty()) {
              gaussian_blur_1D_float3(points,
                                      iterations,
                                      influence,
                                      smooth_ends,
                                      keep_shape,
                                      is_cyclic,
                                      curves.positions_for_write(),
                                      curves_positions_copy,
                                      curve_mask);
            }

            if (!curves_opacities_copy.is_empty()) {
              gaussian_blur_1D_float(points,
                                     iterations,
                                     influence,
                                     smooth_ends,
                                     false,
                                     is_cyclic,
                                     drawing.opacities_for_write(),
                                     curves_opacities_copy,
                                     curve_mask);
            }

            if (!curves_radii_copy.is_empty()) {
              gaussian_blur_1D_float(points,
                                     iterations,
                                     influence,
                                     smooth_ends,
                                     false,
                                     is_cyclic,
                                     drawing.radii_for_write(),
                                     curves_radii_copy,
                                     curve_mask);
            }
          }
        });
      });

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_stroke_smooth(wmOperatorType *ot)
{
  PropertyRNA *prop;

  /* Identifiers. */
  ot->name = "Smooth Stroke";
  ot->idname = "GREASE_PENCIL_OT_stroke_smooth";
  ot->description = "Smooth selected strokes";

  /* Callbacks. */
  ot->exec = grease_pencil_stroke_smooth_exec;
  ot->poll = editable_grease_pencil_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* Smooth parameters. */
  prop = RNA_def_int(ot->srna, "iterations", 10, 1, 100, "Iterations", "", 1, 30);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  RNA_def_float(ot->srna, "factor", 1.0f, 0.0f, 1.0f, "Factor", "", 0.0f, 1.0f);
  RNA_def_boolean(ot->srna, "smooth_ends", false, "Smooth Endpoints", "");
  RNA_def_boolean(ot->srna, "keep_shape", false, "Keep Shape", "");

  RNA_def_boolean(ot->srna, "smooth_position", true, "Position", "");
  RNA_def_boolean(ot->srna, "smooth_radius", true, "Radius", "");
  RNA_def_boolean(ot->srna, "smooth_opacity", false, "Opacity", "");
}

}  // namespace blender::ed::greasepencil

void ED_operatortypes_grease_pencil_edit(void)
{
  using namespace blender::ed::greasepencil;
  WM_operatortype_append(GREASE_PENCIL_OT_stroke_smooth);
}

void ED_keymap_grease_pencil(wmKeyConfig *keyconf)
{
  using namespace blender::ed::greasepencil;
  keymap_grease_pencil_editing(keyconf);
  keymap_grease_pencil_painting(keyconf);
}
