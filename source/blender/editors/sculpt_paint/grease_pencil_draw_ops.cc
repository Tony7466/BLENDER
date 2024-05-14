/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"
#include "BKE_deform.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_object_deform.h"
#include "BKE_report.hh"

#include "BLT_translation.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_brush_types.h"
#include "DNA_grease_pencil_types.h"
#include "DNA_scene_types.h"
#include "DNA_windowmanager_types.h"

#include "ED_grease_pencil.hh"
#include "ED_image.hh"
#include "ED_numinput.hh"
#include "ED_object.hh"
#include "ED_screen.hh"

#include "MEM_guardedalloc.h"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "UI_interface.hh"

#include "WM_api.hh"
#include "WM_toolsystem.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"
#include "wm_event_types.hh"
#include <algorithm>
#include <iterator>
#include <optional>

namespace blender::ed::sculpt_paint {

/* -------------------------------------------------------------------- */
/** \name Common Paint Operator Functions
 * \{ */

static bool stroke_get_location(bContext * /*C*/,
                                float out[3],
                                const float mouse[2],
                                bool /*force_original*/)
{
  out[0] = mouse[0];
  out[1] = mouse[1];
  out[2] = 0;
  return true;
}

static void stroke_start(bContext &C,
                         wmOperator &op,
                         const float2 &mouse,
                         GreasePencilStrokeOperation &operation)
{
  PaintStroke *paint_stroke = static_cast<PaintStroke *>(op.customdata);

  InputSample start_sample;
  start_sample.mouse_position = float2(mouse);
  start_sample.pressure = 0.0f;

  paint_stroke_set_mode_data(paint_stroke, &operation);
  operation.on_stroke_begin(C, start_sample);
}

static void stroke_update_step(bContext *C,
                               wmOperator * /*op*/,
                               PaintStroke *stroke,
                               PointerRNA *stroke_element)
{
  GreasePencilStrokeOperation *operation = static_cast<GreasePencilStrokeOperation *>(
      paint_stroke_mode_data(stroke));

  InputSample extension_sample;
  RNA_float_get_array(stroke_element, "mouse", extension_sample.mouse_position);
  extension_sample.pressure = RNA_float_get(stroke_element, "pressure");

  if (operation) {
    operation->on_stroke_extended(*C, extension_sample);
  }
}

static void stroke_redraw(const bContext *C, PaintStroke * /*stroke*/, bool /*final*/)
{
  ED_region_tag_redraw(CTX_wm_region(C));
}

static void stroke_done(const bContext *C, PaintStroke *stroke)
{
  GreasePencilStrokeOperation *operation = static_cast<GreasePencilStrokeOperation *>(
      paint_stroke_mode_data(stroke));
  operation->on_stroke_done(*C);
  operation->~GreasePencilStrokeOperation();
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Brush Stroke Operator
 * \{ */

static bool grease_pencil_brush_stroke_poll(bContext *C)
{
  if (!ed::greasepencil::grease_pencil_painting_poll(C)) {
    return false;
  }
  if (!WM_toolsystem_active_tool_is_brush(C)) {
    return false;
  }
  return true;
}

static GreasePencilStrokeOperation *grease_pencil_brush_stroke_operation(bContext &C)
{
  const Scene &scene = *CTX_data_scene(&C);
  const GpPaint &gp_paint = *scene.toolsettings->gp_paint;
  const Brush &brush = *BKE_paint_brush_for_read(&gp_paint.paint);
  switch (eBrushGPaintTool(brush.gpencil_tool)) {
    case GPAINT_TOOL_DRAW:
      /* FIXME: Somehow store the unique_ptr in the PaintStroke. */
      return greasepencil::new_paint_operation().release();
    case GPAINT_TOOL_ERASE:
      return greasepencil::new_erase_operation().release();
    case GPAINT_TOOL_FILL:
      return nullptr;
    case GPAINT_TOOL_TINT:
      return greasepencil::new_tint_operation().release();
  }
  return nullptr;
}

static bool grease_pencil_brush_stroke_test_start(bContext *C,
                                                  wmOperator *op,
                                                  const float mouse[2])
{
  GreasePencilStrokeOperation *operation = grease_pencil_brush_stroke_operation(*C);
  if (operation) {
    stroke_start(*C, *op, float2(mouse), *operation);
    return true;
  }
  return false;
}

static int grease_pencil_brush_stroke_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  int return_value = ed::greasepencil::grease_pencil_draw_operator_invoke(C, op);
  if (return_value != OPERATOR_RUNNING_MODAL) {
    return return_value;
  }

  op->customdata = paint_stroke_new(C,
                                    op,
                                    stroke_get_location,
                                    grease_pencil_brush_stroke_test_start,
                                    stroke_update_step,
                                    stroke_redraw,
                                    stroke_done,
                                    event->type);

  return_value = op->type->modal(C, op, event);
  if (return_value == OPERATOR_FINISHED) {
    return OPERATOR_FINISHED;
  }

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static int grease_pencil_brush_stroke_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  return paint_stroke_modal(C, op, event, reinterpret_cast<PaintStroke **>(&op->customdata));
}

static void grease_pencil_brush_stroke_cancel(bContext *C, wmOperator *op)
{
  paint_stroke_cancel(C, op, static_cast<PaintStroke *>(op->customdata));
}

static void GREASE_PENCIL_OT_brush_stroke(wmOperatorType *ot)
{
  ot->name = "Grease Pencil Draw";
  ot->idname = "GREASE_PENCIL_OT_brush_stroke";
  ot->description = "Draw a new stroke in the active Grease Pencil object";

  ot->poll = grease_pencil_brush_stroke_poll;
  ot->invoke = grease_pencil_brush_stroke_invoke;
  ot->modal = grease_pencil_brush_stroke_modal;
  ot->cancel = grease_pencil_brush_stroke_cancel;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  paint_stroke_operator_properties(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Sculpt Operator
 * \{ */

static bool grease_pencil_sculpt_paint_poll(bContext *C)
{
  if (!ed::greasepencil::grease_pencil_sculpting_poll(C)) {
    return false;
  }
  if (!WM_toolsystem_active_tool_is_brush(C)) {
    return false;
  }
  return true;
}

static GreasePencilStrokeOperation *grease_pencil_sculpt_paint_operation(
    bContext &C, const BrushStrokeMode stroke_mode)
{
  const Scene &scene = *CTX_data_scene(&C);
  const GpSculptPaint &gp_sculptpaint = *scene.toolsettings->gp_sculptpaint;
  const Brush &brush = *BKE_paint_brush_for_read(&gp_sculptpaint.paint);
  switch (eBrushGPSculptTool(brush.gpencil_sculpt_tool)) {
    case GPSCULPT_TOOL_SMOOTH:
      return greasepencil::new_smooth_operation(stroke_mode).release();
    case GPSCULPT_TOOL_THICKNESS:
      return greasepencil::new_thickness_operation(stroke_mode).release();
    case GPSCULPT_TOOL_STRENGTH:
      return greasepencil::new_strength_operation(stroke_mode).release();
    case GPSCULPT_TOOL_GRAB:
      return greasepencil::new_grab_operation(stroke_mode).release();
    case GPSCULPT_TOOL_PUSH:
      return greasepencil::new_push_operation(stroke_mode).release();
    case GPSCULPT_TOOL_TWIST:
      return greasepencil::new_twist_operation(stroke_mode).release();
    case GPSCULPT_TOOL_PINCH:
      return greasepencil::new_pinch_operation(stroke_mode).release();
    case GPSCULPT_TOOL_RANDOMIZE:
      return greasepencil::new_randomize_operation(stroke_mode).release();
    case GPSCULPT_TOOL_CLONE:
      return greasepencil::new_clone_operation(stroke_mode).release();
  }
  return nullptr;
}

static bool grease_pencil_sculpt_paint_test_start(bContext *C,
                                                  wmOperator *op,
                                                  const float mouse[2])
{
  const BrushStrokeMode stroke_mode = BrushStrokeMode(RNA_enum_get(op->ptr, "mode"));
  GreasePencilStrokeOperation *operation = grease_pencil_sculpt_paint_operation(*C, stroke_mode);
  if (operation) {
    stroke_start(*C, *op, float2(mouse), *operation);
    return true;
  }
  return false;
}

static int grease_pencil_sculpt_paint_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  const Scene *scene = CTX_data_scene(C);
  const Object *object = CTX_data_active_object(C);
  if (!object || object->type != OB_GREASE_PENCIL) {
    return OPERATOR_CANCELLED;
  }

  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
  if (!grease_pencil.has_active_layer()) {
    BKE_report(op->reports, RPT_ERROR, "No active Grease Pencil layer");
    return OPERATOR_CANCELLED;
  }

  const Paint *paint = BKE_paint_get_active_from_context(C);
  const Brush *brush = BKE_paint_brush_for_read(paint);
  if (brush == nullptr) {
    return OPERATOR_CANCELLED;
  }

  bke::greasepencil::Layer &active_layer = *grease_pencil.get_active_layer();

  if (!active_layer.is_editable()) {
    BKE_report(op->reports, RPT_ERROR, "Active layer is locked or hidden");
    return OPERATOR_CANCELLED;
  }

  /* Ensure a drawing at the current keyframe. */
  bool inserted_keyframe = false;
  if (!ed::greasepencil::ensure_active_keyframe(*scene, grease_pencil, inserted_keyframe)) {
    BKE_report(op->reports, RPT_ERROR, "No Grease Pencil frame to draw on");
    return OPERATOR_CANCELLED;
  }
  if (inserted_keyframe) {
    WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);
  }

  op->customdata = paint_stroke_new(C,
                                    op,
                                    stroke_get_location,
                                    grease_pencil_sculpt_paint_test_start,
                                    stroke_update_step,
                                    stroke_redraw,
                                    stroke_done,
                                    event->type);

  const int return_value = op->type->modal(C, op, event);
  if (return_value == OPERATOR_FINISHED) {
    return OPERATOR_FINISHED;
  }

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static int grease_pencil_sculpt_paint_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  return paint_stroke_modal(C, op, event, reinterpret_cast<PaintStroke **>(&op->customdata));
}

static void grease_pencil_sculpt_paint_cancel(bContext *C, wmOperator *op)
{
  paint_stroke_cancel(C, op, static_cast<PaintStroke *>(op->customdata));
}

static void GREASE_PENCIL_OT_sculpt_paint(wmOperatorType *ot)
{
  ot->name = "Grease Pencil Draw";
  ot->idname = "GREASE_PENCIL_OT_sculpt_paint";
  ot->description = "Draw a new stroke in the active Grease Pencil object";

  ot->poll = grease_pencil_sculpt_paint_poll;
  ot->invoke = grease_pencil_sculpt_paint_invoke;
  ot->modal = grease_pencil_sculpt_paint_modal;
  ot->cancel = grease_pencil_sculpt_paint_cancel;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  paint_stroke_operator_properties(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Weight Brush Stroke Operator
 * \{ */

static bool weight_stroke_test_start(bContext *C, wmOperator *op, const float mouse[2])
{
  InputSample start_sample;
  start_sample.mouse_position = float2(mouse);
  start_sample.pressure = 0.0f;

  GreasePencilStrokeOperation *operation = nullptr;
  Paint *paint = BKE_paint_get_active_from_context(C);
  Brush *brush = BKE_paint_brush(paint);
  const BrushStrokeMode brush_mode = BrushStrokeMode(RNA_enum_get(op->ptr, "mode"));

  switch (eBrushGPWeightTool(brush->gpencil_weight_tool)) {
    case GPWEIGHT_TOOL_DRAW:
      operation = greasepencil::new_weight_paint_draw_operation(brush_mode).release();
      break;
    case GPWEIGHT_TOOL_BLUR:
      operation = greasepencil::new_weight_paint_blur_operation().release();
      break;
    case GPWEIGHT_TOOL_AVERAGE:
      operation = greasepencil::new_weight_paint_average_operation().release();
      break;
    case GPWEIGHT_TOOL_SMEAR:
      operation = greasepencil::new_weight_paint_smear_operation().release();
      break;
  }

  if (operation == nullptr) {
    return false;
  }

  PaintStroke *paint_stroke = static_cast<PaintStroke *>(op->customdata);
  paint_stroke_set_mode_data(paint_stroke, operation);
  operation->on_stroke_begin(*C, start_sample);
  return true;
}

static int grease_pencil_weight_brush_stroke_invoke(bContext *C,
                                                    wmOperator *op,
                                                    const wmEvent *event)
{
  const Scene *scene = CTX_data_scene(C);
  const Object *object = CTX_data_active_object(C);
  if (!object || object->type != OB_GREASE_PENCIL) {
    return OPERATOR_CANCELLED;
  }

  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);
  const Paint *paint = BKE_paint_get_active_from_context(C);
  const Brush *brush = BKE_paint_brush_for_read(paint);
  if (brush == nullptr) {
    return OPERATOR_CANCELLED;
  }

  const Vector<ed::greasepencil::MutableDrawingInfo> drawings =
      ed::greasepencil::retrieve_editable_drawings(*scene, grease_pencil);
  if (drawings.is_empty()) {
    BKE_report(op->reports, RPT_ERROR, "No Grease Pencil frame to draw weight on");
    return OPERATOR_CANCELLED;
  }

  const int active_defgroup_nr = BKE_object_defgroup_active_index_get(object) - 1;
  if (active_defgroup_nr >= 0 && BKE_object_defgroup_active_is_locked(object)) {
    BKE_report(op->reports, RPT_WARNING, "Active group is locked, aborting");
    return OPERATOR_CANCELLED;
  }

  op->customdata = paint_stroke_new(C,
                                    op,
                                    stroke_get_location,
                                    weight_stroke_test_start,
                                    stroke_update_step,
                                    stroke_redraw,
                                    stroke_done,
                                    event->type);

  const int return_value = op->type->modal(C, op, event);
  if (return_value == OPERATOR_FINISHED) {
    return OPERATOR_FINISHED;
  }

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static int grease_pencil_weight_brush_stroke_modal(bContext *C,
                                                   wmOperator *op,
                                                   const wmEvent *event)
{
  return paint_stroke_modal(C, op, event, reinterpret_cast<PaintStroke **>(&op->customdata));
}

static void grease_pencil_weight_brush_stroke_cancel(bContext *C, wmOperator *op)
{
  paint_stroke_cancel(C, op, static_cast<PaintStroke *>(op->customdata));
}

static bool grease_pencil_weight_brush_stroke_poll(bContext *C)
{
  if (!ed::greasepencil::grease_pencil_weight_painting_poll(C)) {
    return false;
  }
  if (!WM_toolsystem_active_tool_is_brush(C)) {
    return false;
  }
  return true;
}

static void GREASE_PENCIL_OT_weight_brush_stroke(wmOperatorType *ot)
{
  ot->name = "Grease Pencil Paint Weight";
  ot->idname = "GREASE_PENCIL_OT_weight_brush_stroke";
  ot->description = "Draw weight on stroke points in the active Grease Pencil object";

  ot->poll = grease_pencil_weight_brush_stroke_poll;
  ot->invoke = grease_pencil_weight_brush_stroke_invoke;
  ot->modal = grease_pencil_weight_brush_stroke_modal;
  ot->cancel = grease_pencil_weight_brush_stroke_cancel;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  paint_stroke_operator_properties(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Interpolate Operator
 * \{ */

enum class GreasePencilInterpolateFlipMode : int8_t {
  /* No flip. */
  None = 0,
  /* Flip always. */
  Flip,
  /* Flip if needed. */
  FlipAuto,
};

enum class GreasePencilInterpolateLayerMode : int8_t {
  /* Only interpolate on the active layer. */
  Active = 0,
  /* Interpolate strokes on every layer. */
  All,
};

constexpr const float interpolate_shift_min = -1.0f;
constexpr const float interpolate_shift_max = 2.0f;

struct GreasePencilInterpolateOpData {
  struct LayerData {
    int prev_frame;
    int next_frame;
  };

  /* Layers to include. */
  IndexMaskMemory layer_mask_memory;
  IndexMask layer_mask;
  /* Interpolate only selected keyframes. */
  bool interpolate_selected_only;
  /* Exclude breakdown keyframes when finding intervals. */
  bool exclude_breakdowns;

  /* Interpolation factor bias controlled by the user. */
  float shift;
  /* Interpolation base factor for the active layer. */
  float init_factor;

  NumInput num;
  Array<LayerData> layer_data;
  int active_layer_index;
};

// /* Helper: Update all strokes interpolated */
// static void gpencil_interpolate_update_strokes(bContext *C, tGPDinterpolate *tgpi)
// {
//   bGPdata *gpd = tgpi->gpd;
//   const float shift = tgpi->shift;

//   LISTBASE_FOREACH (tGPDinterpolate_layer *, tgpil, &tgpi->ilayers) {
//     const float factor = tgpil->factor + shift;

//     bGPDframe *gpf = tgpil->gpl->actframe;
//     /* Free temp strokes used for display. */
//     gpencil_interpolate_free_tagged_strokes(gpf);

//     /* Clear previous interpolations. */
//     gpencil_interpolate_free_tagged_strokes(tgpil->interFrame);

//     LISTBASE_FOREACH (LinkData *, link, &tgpil->selected_strokes) {
//       bGPDstroke *gps_from = static_cast<bGPDstroke *>(link->data);
//       if (!BLI_ghash_haskey(tgpil->pair_strokes, gps_from)) {
//         continue;
//       }
//       bGPDstroke *gps_to = (bGPDstroke *)BLI_ghash_lookup(tgpil->pair_strokes, gps_from);

//       /* Create new stroke. */
//       bGPDstroke *new_stroke = BKE_gpencil_stroke_duplicate(gps_from, true, true);
//       new_stroke->flag |= GP_STROKE_TAG;
//       new_stroke->select_index = 0;

//       /* Update points position. */
//       gpencil_interpolate_update_points(gps_from, gps_to, new_stroke, factor);

//       /* Calc geometry data. */
//       BKE_gpencil_stroke_geometry_update(gpd, new_stroke);
//       /* Add to strokes. */
//       BLI_addtail(&tgpil->interFrame->strokes, new_stroke);

//       /* Add temp strokes to display. */
//       if (gpf) {
//         bGPDstroke *gps_eval = BKE_gpencil_stroke_duplicate(new_stroke, true, true);
//         gps_eval->flag |= GP_STROKE_TAG;
//         BLI_addtail(&gpf->strokes, gps_eval);
//       }
//     }
//   }

//   DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
//   WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);
// }

// /* Helper: Create internal strokes interpolated */
// static void gpencil_interpolate_set_points(bContext *C, tGPDinterpolate *tgpi)
// {
//   Scene *scene = tgpi->scene;
//   bGPdata *gpd = tgpi->gpd;
//   bGPDlayer *active_gpl = CTX_data_active_gpencil_layer(C);
//   bGPDframe *actframe = active_gpl->actframe;
//   const bool exclude_breakdowns = (tgpi->flag & GP_TOOLFLAG_INTERPOLATE_EXCLUDE_BREAKDOWNS) !=
//   0;

//   /* save initial factor for active layer to define shift limits */
//   tgpi->init_factor = float(tgpi->cframe - actframe->framenum) /
//                       (actframe->next->framenum - actframe->framenum + 1);

//   /* limits are 100% below 0 and 100% over the 100% */
//   tgpi->low_limit = -1.0f - tgpi->init_factor;
//   tgpi->high_limit = 2.0f - tgpi->init_factor;

//   /* set layers */
//   LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
//     tGPDinterpolate_layer *tgpil;
//     /* all layers or only active */
//     if (!(tgpi->flag & GP_TOOLFLAG_INTERPOLATE_ALL_LAYERS) && (gpl != active_gpl)) {
//       continue;
//     }
//     /* only editable and visible layers are considered */
//     if (!BKE_gpencil_layer_is_editable(gpl) || (gpl->actframe == nullptr)) {
//       continue;
//     }
//     if ((gpl->actframe == nullptr) || (gpl->actframe->next == nullptr)) {
//       continue;
//     }

//     bGPDframe *gpf_prv = gpencil_get_previous_keyframe(gpl, scene->r.cfra, exclude_breakdowns);
//     if (gpf_prv == nullptr) {
//       continue;
//     }
//     bGPDframe *gpf_next = gpencil_get_next_keyframe(gpl, scene->r.cfra, exclude_breakdowns);
//     if (gpf_next == nullptr) {
//       continue;
//     }

//     /* Create temp data for each layer. */
//     tgpil = static_cast<tGPDinterpolate_layer *>(
//         MEM_callocN(sizeof(tGPDinterpolate_layer), "GPencil Interpolate Layer"));
//     tgpil->gpl = gpl;
//     tgpil->prevFrame = BKE_gpencil_frame_duplicate(gpf_prv, true);
//     tgpil->nextFrame = BKE_gpencil_frame_duplicate(gpf_next, true);

//     BLI_addtail(&tgpi->ilayers, tgpil);

//     /* Create a new temporary frame. */
//     tgpil->interFrame = static_cast<bGPDframe *>(MEM_callocN(sizeof(bGPDframe), "bGPDframe"));
//     tgpil->interFrame->framenum = tgpi->cframe;

//     /* get interpolation factor by layer (usually must be equal for all layers, but not sure) */
//     tgpil->factor = float(tgpi->cframe - tgpil->prevFrame->framenum) /
//                     (tgpil->nextFrame->framenum - tgpil->prevFrame->framenum + 1);

//     /* Load the relationship between frames. */
//     gpencil_stroke_pair_table(C, tgpi, tgpil);

//     /* Create new strokes data with interpolated points reading original stroke. */
//     LISTBASE_FOREACH (LinkData *, link, &tgpil->selected_strokes) {
//       bGPDstroke *gps_from = static_cast<bGPDstroke *>(link->data);
//       if (!BLI_ghash_haskey(tgpil->pair_strokes, gps_from)) {
//         continue;
//       }
//       bGPDstroke *gps_to = (bGPDstroke *)BLI_ghash_lookup(tgpil->pair_strokes, gps_from);

//       /* If destination stroke is smaller, resize new_stroke to size of gps_to stroke. */
//       if (gps_from->totpoints > gps_to->totpoints) {
//         BKE_gpencil_stroke_uniform_subdivide(gpd, gps_to, gps_from->totpoints, true);
//       }
//       if (gps_to->totpoints > gps_from->totpoints) {
//         BKE_gpencil_stroke_uniform_subdivide(gpd, gps_from, gps_to->totpoints, true);
//       }

//       /* Flip stroke. */
//       if (tgpi->flipmode == GP_INTERPOLATE_FLIP) {
//         BKE_gpencil_stroke_flip(gps_to);
//       }
//       else if (tgpi->flipmode == GP_INTERPOLATE_FLIPAUTO) {
//         if (gpencil_stroke_need_flip(tgpi->depsgraph, tgpi->ob, gpl, &tgpi->gsc, gps_from,
//         gps_to))
//         {
//           BKE_gpencil_stroke_flip(gps_to);
//         }
//       }

//       /* Create new stroke. */
//       bGPDstroke *new_stroke = BKE_gpencil_stroke_duplicate(gps_from, true, true);
//       new_stroke->flag |= GP_STROKE_TAG;
//       new_stroke->select_index = 0;

//       /* Update points position. */
//       gpencil_interpolate_update_points(gps_from, gps_to, new_stroke, tgpil->factor);
//       BKE_gpencil_stroke_smooth(new_stroke,
//                                 tgpi->smooth_factor,
//                                 tgpi->smooth_steps,
//                                 true,
//                                 true,
//                                 false,
//                                 false,
//                                 true,
//                                 nullptr);

//       /* Calc geometry data. */
//       BKE_gpencil_stroke_geometry_update(gpd, new_stroke);
//       /* add to strokes */
//       BLI_addtail(&tgpil->interFrame->strokes, new_stroke);
//     }
//   }
// }

// /* Helper: calculate shift based on position of mouse (we only use x-axis for now.
//  * since this is more convenient for users to do), and store new shift value
//  */
// static void gpencil_mouse_update_shift(tGPDinterpolate *tgpi, wmOperator *op, const wmEvent
// *event)
// {
//   float mid = float(tgpi->region->winx - tgpi->region->winrct.xmin) / 2.0f;
//   float mpos = event->xy[0] - tgpi->region->winrct.xmin;

//   if (mpos >= mid) {
//     tgpi->shift = ((mpos - mid) * tgpi->high_limit) / mid;
//   }
//   else {
//     tgpi->shift = tgpi->low_limit - ((mpos * tgpi->low_limit) / mid);
//   }

//   CLAMP(tgpi->shift, tgpi->low_limit, tgpi->high_limit);
//   RNA_float_set(op->ptr, "shift", tgpi->shift);
// }

static void grease_pencil_interpolate_status_indicators(
    bContext &C, const GreasePencilInterpolateOpData &opdata)
{
  Scene &scene = *CTX_data_scene(&C);
  ScrArea &area = *CTX_wm_area(&C);

  const StringRef msg = IFACE_("GPencil Interpolation: ");

  std::string status;
  if (hasNumInput(&opdata.num)) {
    char str_ofs[NUM_STR_REP_LEN];
    outputNumInput(&const_cast<NumInput &>(opdata.num), str_ofs, &scene.unit);
    status = msg + std::string(str_ofs);
  }
  else {
    status = msg + std::to_string(int((opdata.init_factor + opdata.shift) * 100.0f)) + " %";
  }

  ED_area_status_text(&area, status.c_str());
  ED_workspace_status_text(
      &C, IFACE_("ESC/RMB to cancel, Enter/LMB to confirm, WHEEL/MOVE to adjust factor"));
}

static void grease_pencil_interpolate_update(bContext &C,
                                             const wmOperator &op,
                                             const GreasePencilInterpolateOpData &opdata)
{
  grease_pencil_interpolate_status_indicators(C, opdata);

  //   /* update points position */
  //   gpencil_interpolate_update_strokes(C, tgpi);
}

using FramesMapKeyInterval = std::pair<int, int>;

static std::optional<FramesMapKeyInterval> find_frames_interval(
    const bke::greasepencil::Layer &layer, const int frame_number)
{
  using bke::greasepencil::FramesMapKey;
  using SortedKeysIterator = Span<FramesMapKey>::iterator;

  const Span<FramesMapKey> sorted_keys = layer.sorted_keys();
  const SortedKeysIterator next_key_it = std::upper_bound(
      sorted_keys.begin(), sorted_keys.end(), frame_number);

  if (next_key_it == sorted_keys.end() || next_key_it == sorted_keys.begin()) {
    return std::nullopt;
  }

  return std::make_pair(*(next_key_it - 1), *next_key_it);
}

static bool grease_pencil_interpolate_init(const bContext &C, wmOperator &op)
{
  using bke::greasepencil::Layer;

  const Scene &scene = *CTX_data_scene(&C);
  const int current_frame = scene.r.cfra;
  const Object &object = *CTX_data_active_object(&C);
  const GreasePencil &grease_pencil = *static_cast<const GreasePencil *>(object.data);

  BLI_assert(grease_pencil.has_active_layer());

  op.customdata = MEM_new<GreasePencilInterpolateOpData>(__func__);
  GreasePencilInterpolateOpData &data = *static_cast<GreasePencilInterpolateOpData *>(
      op.customdata);

  data.shift = RNA_float_get(op.ptr, "shift");
  data.interpolate_selected_only = RNA_boolean_get(op.ptr, "interpolate_selected_only");
  data.exclude_breakdowns = RNA_boolean_get(op.ptr, "exclude_breakdowns");
  data.active_layer_index = *grease_pencil.get_layer_index(*grease_pencil.get_active_layer());

  const auto layer_mode = GreasePencilInterpolateLayerMode(RNA_enum_get(op.ptr, "layers"));
  switch (layer_mode) {
    case GreasePencilInterpolateLayerMode::Active:
      data.layer_mask = IndexRange::from_single(data.active_layer_index);
      break;
    case GreasePencilInterpolateLayerMode::All:
      data.layer_mask = IndexMask::from_predicate(
          grease_pencil.layers().index_range(),
          GrainSize(1024),
          data.layer_mask_memory,
          [&](const int layer_index) {
            return grease_pencil.layers()[layer_index]->is_editable();
          });
      break;
  }

  data.layer_data.reinitialize(grease_pencil.layers().size());
  data.layer_mask.foreach_index([&](const int layer_index) {
    const Layer &layer = *grease_pencil.layers()[layer_index];
    GreasePencilInterpolateOpData::LayerData &layer_data = data.layer_data[layer_index];

    if (const std::optional<FramesMapKeyInterval> interval = find_frames_interval(layer,
                                                                                  current_frame))
    {
      layer_data.prev_frame = interval->first;
      layer_data.next_frame = interval->second;
    }
    else {
      layer_data.prev_frame = 0;
      layer_data.next_frame = 0;
    }
  });

  const GreasePencilInterpolateOpData::LayerData &active_layer_data =
      data.layer_data[data.active_layer_index];
  if (active_layer_data.prev_frame == active_layer_data.next_frame) {
    BKE_report(
        op.reports,
        RPT_ERROR,
        "Cannot find valid keyframes to interpolate (Breakdowns keyframes are not allowed)");
    MEM_delete(&data);
    return false;
  }
  data.init_factor = float(current_frame - active_layer_data.prev_frame) /
                     (active_layer_data.next_frame - active_layer_data.prev_frame + 1);

  return true;
}

/* Exit and free memory. */
static void grease_pencil_interpolate_exit(bContext &C, wmOperator &op)
{
  ScrArea &area = *CTX_wm_area(&C);

  if (op.customdata == nullptr) {
    return;
  }

  ED_area_status_text(&area, nullptr);
  ED_workspace_status_text(&C, nullptr);

  MEM_delete(static_cast<GreasePencilInterpolateOpData *>(op.customdata));
  op.customdata = nullptr;

  // DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  // WM_event_add_notifier(&C, NC_GPENCIL | NA_EDITED, nullptr);
}

// /* Init new temporary interpolation data */
// static bool gpencil_interpolate_set_init_values(bContext *C, wmOperator *op, tGPDinterpolate
// *tgpi)
// {
//   /* set current scene and window */
//   tgpi->depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
//   tgpi->scene = CTX_data_scene(C);
//   tgpi->area = CTX_wm_area(C);
//   tgpi->region = CTX_wm_region(C);
//   tgpi->ob = CTX_data_active_object(C);
//   /* Setup space conversions. */
//   gpencil_point_conversion_init(C, &tgpi->gsc);

//   /* set current frame number */
//   tgpi->cframe = tgpi->scene->r.cfra;

//   /* set GP datablock */
//   tgpi->gpd = static_cast<bGPdata *>(tgpi->ob->data);
//   /* set interpolation weight */
//   tgpi->shift = RNA_float_get(op->ptr, "shift");
//   SET_FLAG_FROM_TEST(
//       tgpi->flag, (RNA_enum_get(op->ptr, "layers") == 1), GP_TOOLFLAG_INTERPOLATE_ALL_LAYERS);
//   SET_FLAG_FROM_TEST(
//       tgpi->flag,
//       (GPENCIL_EDIT_MODE(tgpi->gpd) && RNA_boolean_get(op->ptr, "interpolate_selected_only")),
//       GP_TOOLFLAG_INTERPOLATE_ONLY_SELECTED);
//   SET_FLAG_FROM_TEST(tgpi->flag,
//                      RNA_boolean_get(op->ptr, "exclude_breakdowns"),
//                      GP_TOOLFLAG_INTERPOLATE_EXCLUDE_BREAKDOWNS);

//   tgpi->flipmode = RNA_enum_get(op->ptr, "flip");

//   tgpi->smooth_factor = RNA_float_get(op->ptr, "smooth_factor");
//   tgpi->smooth_steps = RNA_int_get(op->ptr, "smooth_steps");

//   /* Untag strokes to be sure nothing is pending due any canceled process. */
//   LISTBASE_FOREACH (bGPDlayer *, gpl, &tgpi->gpd->layers) {
//     gpencil_interpolate_untag_strokes(gpl);
//   }

//   /* Set layers */
//   gpencil_interpolate_set_points(C, tgpi);

//   return true;
// }

static bool grease_pencil_interpolate_poll(bContext *C)
{
  if (!ed::greasepencil::grease_pencil_painting_poll(C)) {
    return false;
  }

  /* Only 3D view */
  ScrArea *area = CTX_wm_area(C);
  if (area && area->spacetype != SPACE_VIEW3D) {
    return false;
  }

  return true;
}

/* Invoke handler: Initialize the operator */
static int grease_pencil_interpolate_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  wmWindow &win = *CTX_wm_window(C);
  // bGPdata *gpd = CTX_data_gpencil_data(C);
  // bGPDlayer *gpl = CTX_data_active_gpencil_layer(C);
  // tGPDinterpolate *tgpi = nullptr;

  // /* Cannot interpolate if not between 2 frames. */
  // int cfra = scene->r.cfra;
  // const bool exclude_breakdowns = RNA_boolean_get(op->ptr, "exclude_breakdowns");
  // bGPDframe *gpf_prv = gpencil_get_previous_keyframe(gpl, cfra, exclude_breakdowns);
  // bGPDframe *gpf_next = gpencil_get_next_keyframe(gpl, cfra, exclude_breakdowns);
  // if (ELEM(nullptr, gpf_prv, gpf_next)) {
  //   BKE_report(
  //       op->reports,
  //       RPT_ERROR,
  //       "Cannot find valid keyframes to interpolate (Breakdowns keyframes are not allowed)");
  //   return OPERATOR_CANCELLED;
  // }

  if (!grease_pencil_interpolate_init(*C, *op)) {
    grease_pencil_interpolate_exit(*C, *op);
    return OPERATOR_CANCELLED;
  }
  GreasePencilInterpolateOpData &opdata = *static_cast<GreasePencilInterpolateOpData *>(
      op->customdata);

  /* Set cursor to indicate modal operator. */
  WM_cursor_modal_set(&win, WM_CURSOR_EW_SCROLL);

  grease_pencil_interpolate_status_indicators(*C, opdata);

  // DEG_id_tag_update(&gpd->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);

  WM_event_add_modal_handler(C, op);

  return OPERATOR_RUNNING_MODAL;
}

enum class InterpolateToolModalEvent : int8_t {
  Cancel = 1,
  Confirm,
  Increase,
  Decrease,
};

/* Modal handler: Events handling during interactive part */
static int grease_pencil_interpolate_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  wmWindow &win = *CTX_wm_window(C);
  ScrArea &area = *CTX_wm_area(C);
  GreasePencilInterpolateOpData &opdata = *static_cast<GreasePencilInterpolateOpData *>(
      op->customdata);
  const bool has_numinput = hasNumInput(&opdata.num);

  switch (event->type) {
    case EVT_MODAL_MAP: {
      switch (InterpolateToolModalEvent(event->val)) {
        case InterpolateToolModalEvent::Cancel:
          ED_area_status_text(&area, nullptr);
          ED_workspace_status_text(C, nullptr);
          WM_cursor_modal_restore(&win);

          grease_pencil_interpolate_exit(*C, *op);
          return OPERATOR_CANCELLED;
        case InterpolateToolModalEvent::Confirm:
          ED_area_status_text(&area, nullptr);
          ED_workspace_status_text(C, nullptr);
          WM_cursor_modal_restore(&win);

          //     /* insert keyframes as required... */
          //     LISTBASE_FOREACH (tGPDinterpolate_layer *, tgpil, &tgpi->ilayers) {
          //       gpf_dst = BKE_gpencil_layer_frame_get(tgpil->gpl, tgpi->cframe,
          //       GP_GETFRAME_ADD_NEW); gpf_dst->key_type = BEZT_KEYTYPE_BREAKDOWN;

          //       /* Copy strokes. */
          //       LISTBASE_FOREACH (bGPDstroke *, gps_src, &tgpil->interFrame->strokes) {
          //         if (gps_src->totpoints == 0) {
          //           continue;
          //         }

          //         /* make copy of source stroke, then adjust pointer to points too */
          //         gps_dst = BKE_gpencil_stroke_duplicate(gps_src, true, true);
          //         gps_dst->flag &= ~GP_STROKE_TAG;

          //         /* Calc geometry data. */
          //         BKE_gpencil_stroke_geometry_update(tgpi->gpd, gps_dst);

          //         BLI_addtail(&gpf_dst->strokes, gps_dst);
          //       }
          //     }

          /* Write current factor to properties for the next execution. */
          RNA_float_set(op->ptr, "shift", opdata.shift);

          grease_pencil_interpolate_exit(*C, *op);
          return OPERATOR_FINISHED;
        case InterpolateToolModalEvent::Increase:
          opdata.shift = std::clamp(opdata.init_factor + opdata.shift + 0.01f,
                                    interpolate_shift_min,
                                    interpolate_shift_max) -
                         opdata.init_factor;
          grease_pencil_interpolate_update(*C, *op, opdata);
          break;
        case InterpolateToolModalEvent::Decrease:
          opdata.shift = std::clamp(opdata.init_factor + opdata.shift - 0.01f,
                                    interpolate_shift_min,
                                    interpolate_shift_max) -
                         opdata.init_factor;
          grease_pencil_interpolate_update(*C, *op, opdata);
          break;
      }
      break;
    }
    case MOUSEMOVE:
      /* calculate new position */
      /* Only handle mouse-move if not doing numeric-input. */
      if (!has_numinput) {
        //       /* Update shift based on position of mouse. */
        //       gpencil_mouse_update_shift(tgpi, op, event);

        grease_pencil_interpolate_update(*C, *op, opdata);
      }
      break;
    default: {
      if ((event->val == KM_PRESS) && handleNumInput(C, &opdata.num, event)) {
        //       const float factor = tgpi->init_factor;
        //       float value;

        //       /* Grab shift from numeric input, and store this new value (the user see an int)
        //       */ value = (factor + tgpi->shift) * 100.0f; applyNumInput(&tgpi->num, &value);
        //       tgpi->shift = value / 100.0f;

        //       /* recalculate the shift to get the right value in the frame scale */
        //       tgpi->shift = tgpi->shift - factor;

        //       CLAMP(tgpi->shift, tgpi->low_limit, tgpi->high_limit);
        //       RNA_float_set(op->ptr, "shift", tgpi->shift);

        grease_pencil_interpolate_update(*C, *op, opdata);
        break;
      }
      /* Unhandled event, allow to pass through. */
      return OPERATOR_RUNNING_MODAL | OPERATOR_PASS_THROUGH;
    }
  }

  return OPERATOR_RUNNING_MODAL;
}

static void grease_pencil_interpolate_cancel(bContext *C, wmOperator *op)
{
  grease_pencil_interpolate_exit(*C, *op);
}

static void GREASE_PENCIL_OT_interpolate(wmOperatorType *ot)
{
  static const EnumPropertyItem flip_modes[] = {
      {int(GreasePencilInterpolateFlipMode::None), "NONE", 0, "No Flip", ""},
      {int(GreasePencilInterpolateFlipMode::Flip), "FLIP", 0, "Flip", ""},
      {int(GreasePencilInterpolateFlipMode::FlipAuto), "AUTO", 0, "Automatic", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem gpencil_interpolation_layer_items[] = {
      {int(GreasePencilInterpolateLayerMode::Active), "ACTIVE", 0, "Active", ""},
      {int(GreasePencilInterpolateLayerMode::All), "ALL", 0, "All Layers", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  /* identifiers */
  ot->name = "Grease Pencil Interpolation";
  ot->idname = "GREASE_PENCIL_OT_interpolate";
  ot->description = "Interpolate grease pencil strokes between frames";

  /* callbacks */
  ot->invoke = grease_pencil_interpolate_invoke;
  ot->modal = grease_pencil_interpolate_modal;
  ot->cancel = grease_pencil_interpolate_cancel;
  ot->poll = grease_pencil_interpolate_poll;

  /* flags */
  ot->flag = OPTYPE_UNDO | OPTYPE_BLOCKING;

  /* properties */
  RNA_def_float_factor(
      ot->srna,
      "shift",
      0.0f,
      -1.0f,
      1.0f,
      "Shift",
      "Bias factor for which frame has more influence on the interpolated strokes",
      -0.9f,
      0.9f);

  RNA_def_enum(ot->srna,
               "layers",
               gpencil_interpolation_layer_items,
               0,
               "Layer",
               "Layers included in the interpolation");

  RNA_def_boolean(ot->srna,
                  "interpolate_selected_only",
                  false,
                  "Only Selected",
                  "Interpolate only selected strokes");

  RNA_def_boolean(ot->srna,
                  "exclude_breakdowns",
                  false,
                  "Exclude Breakdowns",
                  "Exclude existing Breakdowns keyframes as interpolation extremes");

  RNA_def_enum(ot->srna,
               "flip",
               flip_modes,
               int(GreasePencilInterpolateFlipMode::FlipAuto),
               "Flip Mode",
               "Invert destination stroke to match start and end with source stroke");

  RNA_def_int(ot->srna,
              "smooth_steps",
              1,
              1,
              3,
              "Iterations",
              "Number of times to smooth newly created strokes",
              1,
              3);

  RNA_def_float(ot->srna,
                "smooth_factor",
                0.0f,
                0.0f,
                2.0f,
                "Smooth",
                "Amount of smoothing to apply to interpolated strokes, to reduce jitter/noise",
                0.0f,
                2.0f);

  prop = RNA_def_boolean(ot->srna, "release_confirm", false, "Confirm on Release", "");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

/** \} */

}  // namespace blender::ed::sculpt_paint

/* -------------------------------------------------------------------- */
/** \name Registration
 * \{ */

void ED_operatortypes_grease_pencil_draw()
{
  using namespace blender::ed::sculpt_paint;
  WM_operatortype_append(GREASE_PENCIL_OT_brush_stroke);
  WM_operatortype_append(GREASE_PENCIL_OT_sculpt_paint);
  WM_operatortype_append(GREASE_PENCIL_OT_weight_brush_stroke);
  WM_operatortype_append(GREASE_PENCIL_OT_interpolate);
}

void ED_interpolatetool_modal_keymap(wmKeyConfig *keyconf)
{
  using namespace blender::ed::sculpt_paint;
  static const EnumPropertyItem modal_items[] = {
      {int(InterpolateToolModalEvent::Cancel), "CANCEL", 0, "Cancel", ""},
      {int(InterpolateToolModalEvent::Confirm), "CONFIRM", 0, "Confirm", ""},
      {int(InterpolateToolModalEvent::Increase), "INCREASE", 0, "Increase", ""},
      {int(InterpolateToolModalEvent::Decrease), "DECREASE", 0, "Decrease", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  wmKeyMap *keymap = WM_modalkeymap_find(keyconf, "Interpolate Tool Modal Map");

  /* This function is called for each space-type, only needs to add map once. */
  if (keymap && keymap->modal_items) {
    return;
  }

  keymap = WM_modalkeymap_ensure(keyconf, "Interpolate Tool Modal Map", modal_items);

  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_interpolate");
}

/** \} */
