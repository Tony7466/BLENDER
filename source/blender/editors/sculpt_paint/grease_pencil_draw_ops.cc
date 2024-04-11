/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"
#include "BKE_report.hh"

#include "BLI_string.h"
#include "DEG_depsgraph_query.hh"

#include "DNA_brush_types.h"
#include "DNA_grease_pencil_types.h"

#include "DNA_scene_types.h"
#include "ED_grease_pencil.hh"
#include "ED_image.hh"
#include "ED_object.hh"
#include "ED_screen.hh"

#include "ANIM_keyframing.hh"

#include "MEM_guardedalloc.h"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "UI_interface.hh"

#include "BLT_translation.hh"

#include "WM_api.hh"
#include "WM_message.hh"
#include "WM_toolsystem.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"

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
      /* Fill tool keymap uses the paint operator as alternative mode. */
      return greasepencil::new_paint_operation().release();
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
  if (!ed::greasepencil::ensure_active_keyframe(*scene, grease_pencil)) {
    BKE_report(op->reports, RPT_ERROR, "No Grease Pencil frame to draw on");
    return OPERATOR_CANCELLED;
  }

  op->customdata = paint_stroke_new(C,
                                    op,
                                    stroke_get_location,
                                    grease_pencil_brush_stroke_test_start,
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

static GreasePencilStrokeOperation *grease_pencil_sculpt_paint_operation(bContext &C)
{
  const Scene &scene = *CTX_data_scene(&C);
  const GpSculptPaint &gp_sculptpaint = *scene.toolsettings->gp_sculptpaint;
  const Brush &brush = *BKE_paint_brush_for_read(&gp_sculptpaint.paint);
  switch (eBrushGPSculptTool(brush.gpencil_sculpt_tool)) {
    case GPSCULPT_TOOL_SMOOTH:
      return nullptr;
    case GPSCULPT_TOOL_THICKNESS:
      return nullptr;
    case GPSCULPT_TOOL_STRENGTH:
      return nullptr;
    case GPSCULPT_TOOL_GRAB:
      return nullptr;
    case GPSCULPT_TOOL_PUSH:
      return nullptr;
    case GPSCULPT_TOOL_TWIST:
      return nullptr;
    case GPSCULPT_TOOL_PINCH:
      return nullptr;
    case GPSCULPT_TOOL_RANDOMIZE:
      return nullptr;
    case GPSCULPT_TOOL_CLONE:
      return nullptr;
  }
  return nullptr;
}

static bool grease_pencil_sculpt_paint_test_start(bContext *C,
                                                  wmOperator *op,
                                                  const float mouse[2])
{
  GreasePencilStrokeOperation *operation = grease_pencil_sculpt_paint_operation(*C);
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
  if (!ed::greasepencil::ensure_active_keyframe(*scene, grease_pencil)) {
    BKE_report(op->reports, RPT_ERROR, "No Grease Pencil frame to draw on");
    return OPERATOR_CANCELLED;
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
/** \name Toggle Draw Mode
 * \{ */

static bool grease_pencil_mode_poll_paint_cursor(bContext *C)
{
  if (!grease_pencil_brush_stroke_poll(C)) {
    return false;
  }
  if (CTX_wm_region_view3d(C) == nullptr) {
    return false;
  }
  return true;
}

static void grease_pencil_draw_mode_enter(bContext *C)
{
  Scene *scene = CTX_data_scene(C);
  wmMsgBus *mbus = CTX_wm_message_bus(C);

  Object *ob = CTX_data_active_object(C);
  GpPaint *grease_pencil_paint = scene->toolsettings->gp_paint;
  BKE_paint_ensure(scene->toolsettings, (Paint **)&grease_pencil_paint);

  ob->mode = OB_MODE_PAINT_GREASE_PENCIL;

  /* TODO: Setup cursor color. BKE_paint_init() could be used, but creates an additional brush. */
  ED_paint_cursor_start(&grease_pencil_paint->paint, grease_pencil_mode_poll_paint_cursor);
  paint_init_pivot(ob, scene);

  /* Necessary to change the object mode on the evaluated object. */
  DEG_id_tag_update(&ob->id, ID_RECALC_SYNC_TO_EVAL);
  WM_msg_publish_rna_prop(mbus, &ob->id, ob, Object, mode);
  WM_event_add_notifier(C, NC_SCENE | ND_MODE, nullptr);
}

static void grease_pencil_draw_mode_exit(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  ob->mode = OB_MODE_OBJECT;
}

static int grease_pencil_draw_mode_toggle_exec(bContext *C, wmOperator *op)
{
  Object *ob = CTX_data_active_object(C);
  wmMsgBus *mbus = CTX_wm_message_bus(C);

  const bool is_mode_set = ob->mode == OB_MODE_PAINT_GREASE_PENCIL;

  if (is_mode_set) {
    if (!object::mode_compat_set(C, ob, OB_MODE_PAINT_GREASE_PENCIL, op->reports)) {
      return OPERATOR_CANCELLED;
    }
  }

  if (is_mode_set) {
    grease_pencil_draw_mode_exit(C);
  }
  else {
    grease_pencil_draw_mode_enter(C);
  }

  WM_toolsystem_update_from_context_view3d(C);

  /* Necessary to change the object mode on the evaluated object. */
  DEG_id_tag_update(&ob->id, ID_RECALC_SYNC_TO_EVAL);
  WM_msg_publish_rna_prop(mbus, &ob->id, ob, Object, mode);
  WM_event_add_notifier(C, NC_SCENE | ND_MODE, nullptr);
  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_draw_mode_toggle(wmOperatorType *ot)
{
  ot->name = "Grease Pencil Draw Mode Toggle";
  ot->idname = "GREASE_PENCIL_OT_draw_mode_toggle";
  ot->description = "Enter/Exit draw mode for grease pencil";

  ot->exec = grease_pencil_draw_mode_toggle_exec;
  ot->poll = ed::greasepencil::active_grease_pencil_poll;

  ot->flag = OPTYPE_UNDO | OPTYPE_REGISTER;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Bucket Fill Operator
 * \{ */

struct GreasePencilFillOpData {
  GreasePencilFillOpData(bContext &C, blender::bke::greasepencil::Layer & /*layer*/)
  {
    using blender::bke::greasepencil::Layer;

    const ToolSettings &ts = *CTX_data_tool_settings(&C);
    const Brush &brush = *BKE_paint_brush(&ts.gp_paint->paint);
    // const ARegion &region = *CTX_wm_region(&C);

    /* Enable custom drawing handlers to show help lines */
    const bool do_extend = (brush.gpencil_settings->flag & GP_BRUSH_FILL_SHOW_EXTENDLINES);
    const bool help_lines = do_extend ||
                            (brush.gpencil_settings->flag & GP_BRUSH_FILL_SHOW_HELPLINES);
    if (help_lines) {
      // this->region_type = region.type;
      // this->draw_handle_3d = ED_region_draw_cb_activate(
      //     region.type, grease_pencil_fill_draw_3d, tgpf, REGION_DRAW_POST_VIEW);
    }
  }

  ~GreasePencilFillOpData()
  {
    /* clear status message area */
    // MEM_SAFE_FREE(tgpf->sbuffer);
    // MEM_SAFE_FREE(tgpf->depth_arr);

    /* Clean temp strokes. */
    // stroke_array_free(tgpf);

    /* Remove any temp stroke. */
    // gpencil_delete_temp_stroke_extension(tgpf, true);

    /* remove drawing handler */
    // if (this->draw_handle_3d) {
    //   ED_region_draw_cb_exit(this->region_type, this->draw_handle_3d);
    // }

    /* Remove depth buffer in cache. */
    // if (tgpf->depths) {
    //   ED_view3d_depths_free(tgpf->depths);
    // }
  }
};

static void grease_pencil_fill_status_indicators(bContext &C)
{
  ToolSettings &ts = *CTX_data_tool_settings(&C);
  Brush &brush = *BKE_paint_brush(&ts.gp_paint->paint);

  const bool is_extend = (brush.gpencil_settings->fill_extend_mode == GP_FILL_EMODE_EXTEND);
  const bool use_stroke_collide = (brush.gpencil_settings->flag & GP_BRUSH_FILL_STROKE_COLLIDE) !=
                                  0;
  const float fill_extend_fac = brush.gpencil_settings->fill_extend_fac;

  char status_str[UI_MAX_DRAW_STR];
  BLI_snprintf(status_str,
               sizeof(status_str),
               IFACE_("Fill: ESC/RMB cancel, LMB Fill, Shift Draw on Back, MMB Adjust Extend, S: "
                      "Switch Mode, D: "
                      "Stroke Collision | %s %s (%.3f)"),
               (is_extend) ? IFACE_("Extend") : IFACE_("Radius"),
               (is_extend && use_stroke_collide) ? IFACE_("Stroke: ON") : IFACE_("Stroke: OFF"),
               fill_extend_fac);

  ED_workspace_status_text(&C, status_str);
}

static bool grease_pencil_fill_init(bContext &C, wmOperator &op)
{
  using blender::bke::greasepencil::Layer;

  Object &ob = *CTX_data_active_object(&C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob.data);
  Layer *layer = grease_pencil.get_active_layer();
  /* Cannot paint in locked layer. */
  if (layer && layer->is_locked()) {
    return false;
  }

  if (layer == nullptr) {
    layer = &grease_pencil.add_layer("GP_Layer");
  }

  op.customdata = MEM_new<GreasePencilFillOpData>(__func__, C, *layer);
  return true;
}

static void grease_pencil_fill_exit(bContext &C, wmOperator &op)
{
  Object &ob = *CTX_data_active_object(&C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob.data);

  WM_cursor_modal_restore(CTX_wm_window(&C));

  if (op.customdata) {
    MEM_delete(static_cast<GreasePencilFillOpData *>(op.customdata));
    op.customdata = nullptr;
  }

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);

  WM_main_add_notifier(NC_GEOM | ND_DATA, nullptr);
  WM_event_add_notifier(&C, NC_GPENCIL | ND_DATA | NA_EDITED, nullptr);
}

static int grease_pencil_fill_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  ToolSettings &ts = *CTX_data_tool_settings(C);
  Brush &brush = *BKE_paint_brush(&ts.gp_paint->paint);
  Object &ob = *CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob.data);

  /* Fill tool needs a material (cannot use default material) */
  if (brush.gpencil_settings->flag & GP_BRUSH_MATERIAL_PINNED) {
    if (brush.gpencil_settings->material == nullptr) {
      BKE_report(op->reports, RPT_ERROR, "Fill tool needs active material");
      return OPERATOR_CANCELLED;
    }
  }
  else {
    if (BKE_object_material_get(&ob, ob.actcol) == nullptr) {
      BKE_report(op->reports, RPT_ERROR, "Fill tool needs active material");
      return OPERATOR_CANCELLED;
    }
  }

  if (!grease_pencil_fill_init(*C, *op)) {
    grease_pencil_fill_exit(*C, *op);
    return OPERATOR_CANCELLED;
  }

  WM_cursor_modal_set(CTX_wm_window(C), WM_CURSOR_PAINT_BRUSH);

  grease_pencil_fill_status_indicators(*C);

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);

  /* Add a modal handler for this operator. */
  WM_event_add_modal_handler(C, op);

  return OPERATOR_RUNNING_MODAL;
}

static int grease_pencil_fill_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  int estate = OPERATOR_RUNNING_MODAL;

  switch (event->type) {
    case EVT_ESCKEY:
    case RIGHTMOUSE:
      estate = OPERATOR_CANCELLED;
      break;
    case LEFTMOUSE:
      /* TODO apply the operator. */
      // if (!blender::animrig::is_autokey_on(tgpf->scene) && (!is_multiedit) &&
      //     (tgpf->gpl->actframe == nullptr))
      // {
      //   BKE_report(op->reports, RPT_INFO, "No available frame for creating stroke");
      //   estate = OPERATOR_CANCELLED;
      //   break;
      // }
      // /* if doing a extend transform with the pen, avoid false contacts of
      //  * the pen with the tablet. */
      // if (tgpf->mouse_init[0] != -1.0f) {
      //   break;
      // }
      // copy_v2fl_v2i(tgpf->mouse_center, event->mval);

      // /* first time the event is not enabled to show help lines. */
      // if ((tgpf->oldkey != -1) || (!help_lines)) {
      //   ARegion *region = BKE_area_find_region_xy(CTX_wm_area(C), RGN_TYPE_ANY, event->xy);
      //   if (region) {
      //     bool in_bounds = false;
      //     /* Perform bounds check */
      //     in_bounds = BLI_rcti_isect_pt_v(&region->winrct, event->xy);

      //     if ((in_bounds) && (region->regiontype == RGN_TYPE_WINDOW)) {
      //       tgpf->mouse[0] = event->mval[0];
      //       tgpf->mouse[1] = event->mval[1];
      //       tgpf->is_render = true;
      //       /* Define Zoom level. */
      //       gpencil_zoom_level_set(tgpf);

      //       /* Create Temp stroke. */
      //       tgpf->gps_mouse = BKE_gpencil_stroke_new(0, 1, 10.0f);
      //       tGPspoint point2D;
      //       bGPDspoint *pt = &tgpf->gps_mouse->points[0];
      //       copy_v2fl_v2i(point2D.m_xy, tgpf->mouse);
      //       gpencil_stroke_convertcoords_tpoint(
      //           tgpf->scene, tgpf->region, tgpf->ob, &point2D, nullptr, &pt->x);

      //       /* Hash of selected frames. */
      //       GHash *frame_list = BLI_ghash_int_new_ex(__func__, 64);

      //       /* If not multi-frame and there is no frame in scene->r.cfra for the active layer,
      //        * create a new frame. */
      //       if (!is_multiedit) {
      //         tgpf->gpf = BKE_gpencil_layer_frame_get(
      //             tgpf->gpl,
      //             tgpf->active_cfra,
      //             blender::animrig::is_autokey_on(tgpf->scene) ? GP_GETFRAME_ADD_NEW :
      //                                                            GP_GETFRAME_USE_PREV);
      //         tgpf->gpf->flag |= GP_FRAME_SELECT;

      //         BLI_ghash_insert(
      //             frame_list, POINTER_FROM_INT(tgpf->active_cfra), tgpf->gpl->actframe);
      //       }
      //       else {
      //         BKE_gpencil_frame_selected_hash(tgpf->gpd, frame_list);
      //       }

      //       /* Loop all frames. */
      //       wmWindow *win = CTX_wm_window(C);

      //       GHashIterator gh_iter;
      //       int total = BLI_ghash_len(frame_list);
      //       int i = 1;
      //       GHASH_ITER (gh_iter, frame_list) {
      //         /* Set active frame as current for filling. */
      //         tgpf->active_cfra = POINTER_AS_INT(BLI_ghashIterator_getKey(&gh_iter));
      //         int step = (float(i) / float(total)) * 100.0f;
      //         WM_cursor_time(win, step);

      //         if (extend_lines) {
      //           gpencil_update_extend(tgpf);
      //         }

      //         /* Repeat loop until get something. */
      //         tgpf->done = false;
      //         int loop_limit = 0;
      //         while ((!tgpf->done) && (loop_limit < 2)) {
      //           WM_cursor_time(win, loop_limit + 1);
      //           /* Render screen to temp image and do fill. */
      //           gpencil_do_frame_fill(tgpf, is_inverted);

      //           /* restore size */
      //           tgpf->region->winx = short(tgpf->bwinx);
      //           tgpf->region->winy = short(tgpf->bwiny);
      //           tgpf->region->winrct = tgpf->brect;
      //           if (!tgpf->done) {
      //             /* If the zoom was not set before, avoid a loop. */
      //             if (tgpf->zoom == 1.0f) {
      //               loop_limit++;
      //             }
      //             else {
      //               tgpf->zoom = 1.0f;
      //               tgpf->fill_factor = max_ff(
      //                   GPENCIL_MIN_FILL_FAC,
      //                   min_ff(brush->gpencil_settings->fill_factor, GPENCIL_MAX_FILL_FAC));
      //             }
      //           }
      //           loop_limit++;
      //         }

      //         if (extend_lines) {
      //           stroke_array_free(tgpf);
      //           gpencil_delete_temp_stroke_extension(tgpf, true);
      //         }

      //         i++;
      //       }
      //       WM_cursor_modal_restore(win);
      //       /* Free hash table. */
      //       BLI_ghash_free(frame_list, nullptr, nullptr);

      //       /* Free temp stroke. */
      //       BKE_gpencil_free_stroke(tgpf->gps_mouse);

      //       /* push undo data */
      //       gpencil_undo_push(tgpf->gpd);

      //       /* Save extend value for next operation. */
      //       brush_settings->fill_extend_fac = tgpf->fill_extend_fac;

      //       estate = OPERATOR_FINISHED;
      //     }
      //     else {
      //       estate = OPERATOR_CANCELLED;
      //     }
      //   }
      //   else {
      //     estate = OPERATOR_CANCELLED;
      //   }
      // }
      // else if (extend_lines) {
      //   gpencil_update_extend(tgpf);
      // }
      // tgpf->oldkey = event->type;
      break;
    case EVT_SKEY:
      // if ((show_extend) && (event->val == KM_PRESS)) {
      //   /* Clean temp strokes. */
      //   stroke_array_free(tgpf);

      //   /* Toggle mode. */
      //   if (tgpf->fill_extend_mode == GP_FILL_EMODE_EXTEND) {
      //     tgpf->fill_extend_mode = GP_FILL_EMODE_RADIUS;
      //   }
      //   else {
      //     tgpf->fill_extend_mode = GP_FILL_EMODE_EXTEND;
      //   }
      //   gpencil_delete_temp_stroke_extension(tgpf, true);
      //   gpencil_update_extend(tgpf);
      // }
      break;
    case EVT_DKEY:
      // if ((show_extend) && (event->val == KM_PRESS)) {
      //   tgpf->flag ^= GP_BRUSH_FILL_STROKE_COLLIDE;
      //   /* Clean temp strokes. */
      //   stroke_array_free(tgpf);
      //   gpencil_delete_temp_stroke_extension(tgpf, true);
      //   gpencil_update_extend(tgpf);
      // }
      break;
    case EVT_PAGEUPKEY:
    case WHEELUPMOUSE:
      // if (tgpf->oldkey == 1) {
      //   tgpf->fill_extend_fac -= (event->modifier & KM_SHIFT) ? 0.01f : 0.1f;
      //   CLAMP_MIN(tgpf->fill_extend_fac, 0.0f);
      //   gpencil_update_extend(tgpf);
      // }
      break;
    case EVT_PAGEDOWNKEY:
    case WHEELDOWNMOUSE:
      // if (tgpf->oldkey == 1) {
      //   tgpf->fill_extend_fac += (event->modifier & KM_SHIFT) ? 0.01f : 0.1f;
      //   CLAMP_MAX(tgpf->fill_extend_fac, 10.0f);
      //   gpencil_update_extend(tgpf);
      // }
      break;
    case MIDDLEMOUSE: {
      // if (event->val == KM_PRESS) {
      //   /* Consider initial offset as zero position. */
      //   copy_v2fl_v2i(tgpf->mouse_init, event->mval);
      //   float mlen[2];
      //   sub_v2_v2v2(mlen, tgpf->mouse_init, tgpf->mouse_center);

      //   /* Offset the center a little to get enough space to reduce the extend moving the pen.
      //   */ const float gap = 300.0f; if (len_v2(mlen) < gap) {
      //     tgpf->mouse_center[0] -= gap;
      //     sub_v2_v2v2(mlen, tgpf->mouse_init, tgpf->mouse_center);
      //   }

      //   WM_cursor_set(CTX_wm_window(C), WM_CURSOR_EW_ARROW);

      //   tgpf->initial_length = len_v2(mlen);
      // }
      // if (event->val == KM_RELEASE) {
      //   WM_cursor_modal_set(CTX_wm_window(C), WM_CURSOR_PAINT_BRUSH);

      //   tgpf->mouse_init[0] = -1.0f;
      //   tgpf->mouse_init[1] = -1.0f;
      // }
      // /* Update cursor line. */
      // WM_main_add_notifier(NC_GEOM | ND_DATA, nullptr);
      // WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, nullptr);
      break;
    }
    case MOUSEMOVE: {
      // if (tgpf->mouse_init[0] == -1.0f) {
      //   break;
      // }
      // copy_v2fl_v2i(tgpf->mouse_pos, event->mval);

      // float mlen[2];
      // sub_v2_v2v2(mlen, tgpf->mouse_pos, tgpf->mouse_center);
      // float delta = (len_v2(mlen) - tgpf->initial_length) * tgpf->pixel_size * 0.5f;
      // tgpf->fill_extend_fac += delta;
      // CLAMP(tgpf->fill_extend_fac, 0.0f, 10.0f);

      // /* Update cursor line and extend lines. */
      // WM_main_add_notifier(NC_GEOM | ND_DATA, nullptr);
      // WM_event_add_notifier(C, NC_GPENCIL | ND_DATA | NA_EDITED, nullptr);

      // gpencil_update_extend(tgpf);
      break;
    }
    default:
      break;
  }
  /* Process last operations before exiting. */
  switch (estate) {
    case OPERATOR_FINISHED:
      grease_pencil_fill_exit(*C, *op);
      WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);
      break;

    case OPERATOR_CANCELLED:
      grease_pencil_fill_exit(*C, *op);
      break;

    default:
      break;
  }

  /* return status code */
  return estate;
}

static void grease_pencil_fill_cancel(bContext *C, wmOperator *op)
{
  grease_pencil_fill_exit(*C, *op);
}

static void GREASE_PENCIL_OT_fill(wmOperatorType *ot)
{
  PropertyRNA *prop;

  ot->name = "Grease Pencil Fill";
  ot->idname = "GREASE_PENCIL_OT_fill";
  ot->description = "Fill with color the shape formed by strokes";

  ot->poll = ed::greasepencil::grease_pencil_painting_poll;
  ot->invoke = grease_pencil_fill_invoke;
  ot->modal = grease_pencil_fill_modal;
  ot->cancel = grease_pencil_fill_cancel;

  ot->flag = OPTYPE_UNDO | OPTYPE_REGISTER;

  prop = RNA_def_boolean(ot->srna, "on_back", false, "Draw on Back", "Send new stroke to back");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
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
  WM_operatortype_append(GREASE_PENCIL_OT_draw_mode_toggle);
  WM_operatortype_append(GREASE_PENCIL_OT_fill);
}

/** \} */
