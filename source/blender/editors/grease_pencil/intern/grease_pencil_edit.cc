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
/** \name Smooth Stroke Operator
 * \{ */

template<typename T>
static Span<T> gaussian_blur_1D_ex(const bool is_cyclic,
                                   const IndexRange curve_points,
                                   const int iterations,
                                   const float influence,
                                   const bool smooth_ends,
                                   const bool keep_shape,
                                   MutableSpan<T> dst,
                                   const Span<T> src,
                                   const IndexMask &mask)
{
  /* TODO ? Unlike for the legacy algorithm,
     we don't have a smooth cap parameter */

  /* Avoid computation if the mask is empty */
  if (mask.is_empty()) {
    return dst;
  }

  /* Initialize at zero */
  mask.foreach_index(GrainSize(256), [&](const int64_t point_index) { dst[point_index] = T(0); });

  /* Weight Initialization */
  const int n_half = keep_shape ? (iterations * iterations) / 8 + iterations :
                                  (iterations * iterations) / 4 + 2 * iterations + 12;
  double w = keep_shape ? 2.0 : 1.0;
  double w2 = keep_shape ?
                  (1.0 / M_SQRT3) * exp((2 * iterations * iterations) / double(n_half * 3)) :
                  0.0;
  double total_w = 0.0;

  const int64_t first_pt = curve_points.first();
  const int64_t last_pt = curve_points.last();
  const int64_t nb_pts = curve_points.size();

  for (int step = iterations; step > 0; step--) {
    float w_before = float(w - w2);
    float w_after = float(w - w2);

    mask.foreach_index(GrainSize(256), [&](const int64_t point_index) {
      /* Filter out endpoints if smooth ends is disabled */
      if (!smooth_ends && !is_cyclic && ((point_index == first_pt) || (point_index == last_pt))) {
        return;
      }

      /* Compute the neighboring points */
      int64_t before = point_index - step;
      int64_t after = point_index + step;
      if (is_cyclic) {
        before = ((before - first_pt) % nb_pts + nb_pts) % nb_pts + first_pt;
        after = (after - first_pt) % nb_pts + first_pt;
      }
      else {
        before = std::max(before, first_pt);
        after = std::min(after, last_pt);
      }

      /* Add the neighboring values */
      const T bval = src[before];
      const T aval = src[after];
      const T cval = src[point_index];

      dst[point_index] += (bval - cval) * w_before;
      dst[point_index] += (aval - cval) * w_after;
    });

    /* Update the weight values */
    total_w += w_before;
    total_w += w_after;

    w *= (n_half + step) / double(n_half + 1 - step);
    w2 *= (n_half * 3 + step) / double(n_half * 3 + 1 - step);
  }
  total_w += w - w2;

  /* Normalize the weights */
  mask.foreach_index(GrainSize(256), [&](const int64_t point_index) {
    dst[point_index] = src[point_index] + influence * dst[point_index] / total_w;
  });

  return dst.as_span();
}

Span<float3> gaussian_blur_1D_float3(const bool is_cyclic,
                                     const IndexRange curve_points,
                                     const int iterations,
                                     const float influence,
                                     const bool smooth_ends,
                                     const bool keep_shape,
                                     MutableSpan<float3> dst,
                                     const Span<float3> src,
                                     const IndexMask &mask)
{
  return gaussian_blur_1D_ex<float3>(
      is_cyclic, curve_points, iterations, influence, smooth_ends, keep_shape, dst, src, mask);
}

Span<float> gaussian_blur_1D_float(const bool is_cyclic,
                                   const IndexRange curve_points,
                                   const int iterations,
                                   const float influence,
                                   const bool smooth_ends,
                                   const bool keep_shape,
                                   MutableSpan<float> dst,
                                   const Span<float> src,
                                   const IndexMask &mask)
{
  return gaussian_blur_1D_ex<float>(
      is_cyclic, curve_points, iterations, influence, smooth_ends, keep_shape, dst, src, mask);
}

static int grease_pencil_stroke_smooth_exec(bContext *C, wmOperator * /*op*/)
{
  using namespace blender;
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  /* TODO : these variables should be operator's properties */
  const int iterations = 1;
  const float influence = 0.8;
  const bool keep_shape = false;
  const bool smooth_ends = false;

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int /*drawing_index*/, bke::greasepencil::Drawing &drawing) {
        /* Smooth all selected curves in the current drawing */

        /* Curves geometry and attributes*/
        bke::CurvesGeometry &curves = drawing.strokes_for_write();
        Array<float3> curves_positions_copy(curves.positions());
        bke::AttributeAccessor curves_attributes = curves.attributes();

        const offset_indices::OffsetIndices<int> points_by_curve = curves.points_by_curve();
        const VArray<bool> cyclic = curves.cyclic();

        /* Selection-based mask */
        bke::AttributeReader<bool> selection_attribute = curves_attributes.lookup_or_default<bool>(
            ".selection", ATTR_DOMAIN_POINT, true);

        if (!ed::curves::has_anything_selected(selection_attribute.varray, curves.points_range()))
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

            gaussian_blur_1D_float3(is_cyclic,
                                    points,
                                    iterations,
                                    influence,
                                    smooth_ends,
                                    keep_shape,
                                    curves.positions_for_write(),
                                    curves_positions_copy,
                                    curve_mask);
          }
        });
      });

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_stroke_smooth(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Smooth Stroke";
  ot->idname = "GREASE_PENCIL_OT_stroke_smooth";
  ot->description = "Smooth selected strokes";

  /* callbacks */
  ot->exec = grease_pencil_stroke_smooth_exec;
  ot->poll = editable_grease_pencil_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* TODO : add operator properties */
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
