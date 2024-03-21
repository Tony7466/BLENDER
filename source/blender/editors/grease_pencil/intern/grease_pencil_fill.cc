/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "grease_pencil_fill.hh"

namespace blender::ed::greasepencil::fill {

/**
 * Draw gap closure lines on an overlay in the 3D viewport.
 */
void draw_overlay(const bContext * /*C*/, ARegion *region, void *arg)
{
  FillOperation &fill_op = *static_cast<FillOperation *>(arg);

  /* Draw only in the region that originated the operator. */
  if (region != fill_op.vc.region) {
    return;
  }

  /* Anything to draw? */
  if (!(fill_op.use_gap_close_extend || fill_op.use_gap_close_radius ||
        fill_op.use_gap_close_proximity))
  {
    return;
  }

  const uint shdr_pos = GPU_vertformat_attr_add(
      immVertexFormat(), "pos", GPU_COMP_F32, 2, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  GPU_line_width(1.5f);
  GPU_blend(GPU_BLEND_ALPHA);
  GPU_line_smooth(true);

  /* Draw indications of curve proximities. */
  if (fill_op.use_gap_close_proximity) {
    /* Draw filled circle at start, middle and end point of each curve. */
    int point_indices[3];
    immUniformColor4fv(fill_op.gap_proximity_color);
    for (const int curve_i : fill_op.curves_2d.point_offset.index_range()) {
      if (fill_op.curves_2d.point_size[curve_i] < 2) {
        continue;
      }

      point_indices[0] = 0;
      point_indices[1] = fill_op.curves_2d.point_size[curve_i] - 1;
      point_indices[2] = int(point_indices[1] / 2);
      const int point_count = (point_indices[1] >= 2) ? 3 : 2;
      const int point_offset = fill_op.curves_2d.point_offset[curve_i];

      for (int point = 0; point < point_count; point++) {
        const int point_i = point_offset + point_indices[point];
        imm_draw_circle_fill_2d(shdr_pos,
                                fill_op.curves_2d.points_2d[point_i][0],
                                fill_op.curves_2d.points_2d[point_i][1],
                                fill_op.proximity_distance,
                                40);
      }
    }
  }

  /* Draw curve end extensions. */
  if (fill_op.use_gap_close_extend) {
    /* Draw extensions of full length. */
    for (const int ext_i : fill_op.extensions_2d.point_offset.index_range()) {
      /* Skip intersected extensions or extensions of cyclic curves. */
      if ((fill_op.extensions_stop_at_first_intersection &&
           fill_op.extension_has_intersection[ext_i]) ||
          fill_op.extensions_2d.is_cyclic[ext_i])
      {
        continue;
      }

      if (fill_op.extension_has_intersection[ext_i]) {
        immUniformColor3fv(fill_op.gap_closed_color);
      }
      else {
        immUniformColor3fv(fill_op.gap_closure_color);
      }

      const int point_i = fill_op.extensions_2d.point_offset[ext_i];
      immBegin(GPU_PRIM_LINES, 2);
      immVertex2fv(shdr_pos, fill_op.extensions_2d.points_2d[point_i]);
      immVertex2fv(shdr_pos, fill_op.extensions_2d.points_2d[point_i + 1]);
      immEnd();
    }

    /* Draw shortened extensions (that intersect a curve or other extension). */
    if (fill_op.extensions_stop_at_first_intersection) {
      immUniformColor3fv(fill_op.gap_closed_color);
      int ext_prev = -1;
      for (const auto &intersection : fill_op.extension_intersections) {
        const int ext_i = intersection.extension_index;
        const int point_i = fill_op.extensions_2d.point_offset[ext_i];
        if (ext_i == ext_prev) {
          continue;
        }
        ext_prev = ext_i;

        /* Limit the end extension when it is intersecting something else. */
        const float distance = intersection.distance[0];
        const float2 p0 = fill_op.extensions_2d.points_2d[point_i];
        const float2 p1 = fill_op.extensions_2d.points_2d[point_i + 1];
        const float2 p_vec = p1 - p0;

        immBegin(GPU_PRIM_LINES, 2);
        immVertex2fv(shdr_pos, p0);
        immVertex2fv(shdr_pos, p0 + p_vec * distance);
        immEnd();
      }
    }
  }

  /* Draw curve end radii. */
  if (fill_op.use_gap_close_radius) {

    /* Draw connected curve ends. */
    immUniformColor3fv(fill_op.gap_closed_color);
    for (const int2 &connection : fill_op.radius_connections) {
      const int point_i0 = fill_op.get_curve_point_by_end_index(connection[0]);
      const int point_i1 = fill_op.get_curve_point_by_end_index(connection[1]);
      immBegin(GPU_PRIM_LINES, 2);
      immVertex2fv(shdr_pos, fill_op.curves_2d.points_2d[point_i0]);
      immVertex2fv(shdr_pos, fill_op.curves_2d.points_2d[point_i1]);
      immEnd();
    }

    /* Draw unconnected curve end radii. */
    immUniformColor3fv(fill_op.gap_closure_color);
    GPU_line_width(2.0f);

    int curve_end_index = -1;
    for (const bool connected : fill_op.connected_by_radius) {
      curve_end_index++;
      if (connected) {
        continue;
      }

      /* Skip ends of cyclic curves. */
      const int curve_i = int(curve_end_index / 2);
      if (fill_op.curves_2d.is_cyclic[curve_i]) {
        continue;
      }

      /* Draw radius. */
      const int point_i = fill_op.get_curve_point_by_end_index(curve_end_index);
      imm_draw_circle_wire_2d(shdr_pos,
                              fill_op.curves_2d.points_2d[point_i][0],
                              fill_op.curves_2d.points_2d[point_i][1],
                              fill_op.gap_distance,
                              40);
    }
  }

  immUnbindProgram();

  GPU_line_width(1.0f);
  GPU_line_smooth(false);
  GPU_blend(GPU_BLEND_NONE);

#ifdef GP_FILL_DEBUG_MODE
  fill_op.debug_draw_curve_indices();
#endif
}

/**
 * Clean up the fill operator data.
 */
static void fill_exit(bContext *C, wmOperator *op)
{
  WM_cursor_modal_restore(CTX_wm_window(C));

  if (op->customdata == nullptr) {
    return;
  }

  const ToolSettings *ts = CTX_data_tool_settings(C);
  Brush *brush = BKE_paint_brush(&ts->gp_paint->paint);
  switch (brush->gpencil_settings->fill_mode) {
    case GP_FILL_MODE_FLOOD:
      flood_fill_exit(*op);
      break;
    case GP_FILL_MODE_GEOMETRY:
      geometry_fill_exit(*op);
      break;
  }

  op->customdata = nullptr;
}

/**
 * Modal handler for the fill operator:
 * - Change gap closure length/radius
 * - Perform the fill at second mouse click
 */
static int fill_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  int modal_state = OPERATOR_RUNNING_MODAL;

  const ToolSettings *ts = CTX_data_tool_settings(C);
  Brush *brush = BKE_paint_brush(&ts->gp_paint->paint);
  switch (brush->gpencil_settings->fill_mode) {
    case GP_FILL_MODE_FLOOD:
      modal_state = flood_fill_modal(*op, *event);
      break;
    case GP_FILL_MODE_GEOMETRY:
      modal_state = geometry_fill_modal(*op, *event);
      break;
  }

  switch (modal_state) {
    case OPERATOR_FINISHED:
      fill_exit(C, op);
      WM_event_add_notifier(C, NC_GPENCIL | NA_EDITED, nullptr);
      break;
    case OPERATOR_CANCELLED:
      fill_exit(C, op);
      break;
  }

  return modal_state;
}

/**
 * Invoke the fill operator at first mouse click in the viewport.
 */
static int fill_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  const Scene *scene = CTX_data_scene(C);
  const Object *object = CTX_data_active_object(C);
  const GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  /* Check active layer. */
  if (!grease_pencil.has_active_layer()) {
    BKE_report(op->reports, RPT_ERROR, "No active Grease Pencil layer");
    return OPERATOR_CANCELLED;
  }

  /* Check if layer is editable. */
  const bke::greasepencil::Layer *active_layer = grease_pencil.get_active_layer();
  if (!active_layer->is_editable()) {
    BKE_report(op->reports, RPT_ERROR, "Grease Pencil layer is not editable");
    return OPERATOR_CANCELLED;
  }

  /* TODO: Check active frame. */
  if (!grease_pencil.get_active_layer()->frames().contains(scene->r.cfra)) {
    if (!blender::animrig::is_autokey_on(scene)) {
      BKE_report(op->reports, RPT_ERROR, "No Grease Pencil frame to draw on");
      return OPERATOR_CANCELLED;
    }
  }

  /* Init tool data. */
  bool success = false;
  const ToolSettings *ts = CTX_data_tool_settings(C);
  Brush *brush = BKE_paint_brush(&ts->gp_paint->paint);
  switch (brush->gpencil_settings->fill_mode) {
    case GP_FILL_MODE_FLOOD:
      success = flood_fill_invoke(C, op);
      break;
    case GP_FILL_MODE_GEOMETRY:
      success = geometry_fill_invoke(C, op);
      break;
  }

  if (!success) {
    fill_exit(C, op);
    BKE_report(
        op->reports,
        RPT_ERROR,
        "No Grease Pencil layers with edge strokes found, see 'Layers' in Advanced options");
    return OPERATOR_CANCELLED;
  }

  /* Add modal handler. */
  WM_event_add_modal_handler(C, op);

  /* Set cursor. */
  WM_cursor_modal_set(CTX_wm_window(C), WM_CURSOR_PAINT_BRUSH);

  return OPERATOR_RUNNING_MODAL;
}

/**
 * Definition of the fill operator.
 */
static void GREASE_PENCIL_OT_fill(wmOperatorType *ot)
{
  ot->name = "Fill";
  ot->idname = __func__;
  ot->description = "Fill a shape formed by strokes";

  ot->poll = grease_pencil_painting_fill_poll;
  ot->invoke = fill_invoke;
  ot->modal = fill_modal;
  ot->cancel = fill_exit;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

}  // namespace blender::ed::greasepencil::fill

void ED_operatortypes_grease_pencil_fill()
{
  using namespace blender::ed::greasepencil::fill;

  WM_operatortype_append(GREASE_PENCIL_OT_fill);
}
