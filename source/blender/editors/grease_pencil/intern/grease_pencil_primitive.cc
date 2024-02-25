/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 * Operators for creating new Grease Pencil primitives (boxes, circles, ...).
 */

#include <fmt/format.h>

#include <cstring>

#include "BKE_attribute.hh"
#include "BKE_brush.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"
#include "BKE_paint.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "DEG_depsgraph_query.hh"

#include "ED_grease_pencil.hh"
#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_view3d.hh"

#include "BLI_array_utils.hh"
#include "BLI_string.h"
#include "BLI_vector.hh"

#include "BLT_translation.hh"

#include "GPU_immediate.h"
#include "GPU_state.h"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"

namespace blender::ed::greasepencil {

enum class PrimitiveType : int8_t {
  LINE = 0,
  POLYLINE = 1,
  ARC = 2,
  CURVE = 3,
  BOX = 4,
  CIRCLE = 5,
};

enum class OperatorMode : int8_t {
  IDLE = 0,
  EXTRUDING = 1,
  /* Set the active control point to the mouse. */
  GRAB = 2,
  /* Move the active control point. */
  DRAG = 3,
  /* Move all control points. */
  DRAG_ALL = 4,
  /* Rotate all control points. */
  ROTATE_ALL = 5,
  /* Scale all control points. */
  SCALE_ALL = 6,
};

enum class ControlPointType : int8_t {
  /* The points that are at the end of segments. */
  JOIN_POINT = 0,
  /* The points inside of the segments. */
  EXTRINSIC_POINT = 1,
};

enum class InterpolationMode : bool {
  Mode2D = false,
  Mode3D = true,
};

enum class ModelKeyMode : int8_t {
  CANCEL = 1,
  CONFIRM,
  LEFTCLICK,
  EXTRUDE,
  PANNING,
  GRAB,
  ROTATE,
  SCALE,
  INCREASE_SUBDIVISION,
  DECREASE_SUBDIVISION,
};

static constexpr float UI_PRIMARY_POINT_DRAW_SIZE_PX = 8.0f;
static constexpr float UI_SECONDARY_POINT_DRAW_SIZE_PX = 5.0f;
static constexpr float UI_TERTIARY_POINT_DRAW_SIZE_PX = 3.0f;
static constexpr float UI_POINT_HIT_SIZE_PX = 20.0f;
static constexpr float UI_POINT_MAX_HIT_SIZE_PX = 600.0f;

struct PrimitiveTool_OpData {
  ARegion *region;
  /* For drawing preview loop. */
  void *draw_handle;
  ViewContext vc;

  int segments;
  Vector<float3> control_points;
  /* Store the control points temporally. */
  Vector<float3> temp_control_points;

  PrimitiveType type;
  int subdivision;
  InterpolationMode interpolate_mode;
  float4x4 projection;
  /* Helper class to project screen space coordinates to 3D. */
  ed::greasepencil::DrawingPlacement placement_;

  bke::greasepencil::Drawing *drawing_;
  BrushGpencilSettings *settings_;
  float4 vertex_color_;
  int material_index;
  float hardness;
  Brush *brush_;

  OperatorMode mode;
  float2 start_position_2d;
  int active_control_point_index;

  ViewOpsData *vod;
};

static int control_points_per_segment(const PrimitiveTool_OpData &ptd)
{
  switch (ptd.type) {
    case PrimitiveType::POLYLINE:
    case PrimitiveType::LINE: {
      return 1;
    }
    case PrimitiveType::BOX:
    case PrimitiveType::CIRCLE:
    case PrimitiveType::ARC: {
      return 2;
    }
    case PrimitiveType::CURVE: {
      return 3;
    }
  }

  BLI_assert_unreachable();
  return 0;
}

static ControlPointType get_control_point_type(const PrimitiveTool_OpData &ptd, const int point_id)
{
  BLI_assert(point_id != -1);
  if (ELEM(ptd.type, PrimitiveType::CIRCLE, PrimitiveType::BOX)) {
    return ControlPointType::JOIN_POINT;
  }

  const int num_shared_points = control_points_per_segment(ptd);
  if (math::mod(point_id, num_shared_points) == 0) {
    return ControlPointType::JOIN_POINT;
  }
  return ControlPointType::EXTRINSIC_POINT;
}

static void control_point_colors_and_sizes(const PrimitiveTool_OpData &ptd,
                                           MutableSpan<ColorGeometry4f> colors,
                                           MutableSpan<float> sizes)
{
  ColorGeometry4f color_redalert;
  ColorGeometry4f color_gizmo_primary;
  ColorGeometry4f color_gizmo_secondary;
  ColorGeometry4f color_gizmo_a;
  UI_GetThemeColor4fv(TH_REDALERT, color_redalert);
  UI_GetThemeColor4fv(TH_GIZMO_PRIMARY, color_gizmo_primary);
  UI_GetThemeColor4fv(TH_GIZMO_SECONDARY, color_gizmo_secondary);
  UI_GetThemeColor4fv(TH_GIZMO_A, color_gizmo_a);

  const float size_primary = UI_PRIMARY_POINT_DRAW_SIZE_PX;
  const float size_secondary = UI_SECONDARY_POINT_DRAW_SIZE_PX;
  const float size_tertiary = UI_TERTIARY_POINT_DRAW_SIZE_PX;

  if (ELEM(ptd.type, PrimitiveType::BOX, PrimitiveType::CIRCLE)) {
    colors.fill(color_gizmo_primary);
    sizes.fill(size_primary);

    /* Set the center point's color. */
    colors[1] = color_redalert;
    sizes[1] = size_secondary;
  }
  else {
    colors.fill(color_gizmo_secondary);
    sizes.fill(size_secondary);

    for (const int i : colors.index_range()) {
      const ControlPointType control_point_type = get_control_point_type(ptd, i);

      if (control_point_type == ControlPointType::JOIN_POINT) {
        colors[i] = color_redalert;
        sizes[i] = size_tertiary;
      }
    }

    colors.last() = color_gizmo_primary;
    sizes.last() = size_primary;

    if (ELEM(ptd.type, PrimitiveType::LINE, PrimitiveType::POLYLINE)) {
      colors.last(1) = color_gizmo_secondary;
      sizes.last(1) = size_primary;
    }
  }

  const int active_index = ptd.active_control_point_index;
  if (active_index != -1) {
    sizes[active_index] *= 1.5;
    colors[active_index] = math::interpolate(colors[active_index], color_gizmo_a, 0.5f);
  }
}

static void draw_control_points(PrimitiveTool_OpData &ptd)
{
  GPUVertFormat *format3d = immVertexFormat();
  const uint pos3d = GPU_vertformat_attr_add(format3d, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  const uint col3d = GPU_vertformat_attr_add(format3d, "color", GPU_COMP_F32, 4, GPU_FETCH_FLOAT);
  const uint siz3d = GPU_vertformat_attr_add(format3d, "size", GPU_COMP_F32, 1, GPU_FETCH_FLOAT);
  immBindBuiltinProgram(GPU_SHADER_3D_POINT_VARYING_SIZE_VARYING_COLOR);

  GPU_program_point_size(true);
  immBegin(GPU_PRIM_POINTS, ptd.control_points.size());

  Array<ColorGeometry4f> colors(ptd.control_points.size());
  Array<float> sizes(ptd.control_points.size());
  control_point_colors_and_sizes(ptd, colors, sizes);

  for (const int point_id : ptd.control_points.index_range()) {
    const float3 point = ptd.control_points[point_id];
    const ColorGeometry4f color = colors[point_id];
    const float size = sizes[point_id];

    immAttr4f(col3d, color[0], color[1], color[2], color[3]);
    immAttr1f(siz3d, size * 2.0f);
    immVertex3fv(pos3d, point);
  }

  immEnd();
  immUnbindProgram();
  GPU_program_point_size(false);
}

static void grease_pencil_primitive_draw(const bContext * /*C*/, ARegion * /*region*/, void *arg)
{
  PrimitiveTool_OpData &ptd = *((PrimitiveTool_OpData *)arg);
  draw_control_points(ptd);
}

static void primitive_calulate_curve_positions_3d_exec(PrimitiveTool_OpData &ptd,
                                                       Span<float3> control_points,
                                                       MutableSpan<float3> new_positions)
{
  const int subdivision = ptd.subdivision;
  const int new_points_num = new_positions.size();

  switch (ptd.type) {
    case PrimitiveType::LINE:
    case PrimitiveType::POLYLINE: {
      for (const int i : new_positions.index_range().drop_back(1)) {
        const float t = math::mod(i / float(subdivision + 1), 1.0f);
        const int point_id = int(i / (subdivision + 1));
        const int point_next_id = point_id + 1;
        new_positions[i] = math::interpolate(
            control_points[point_id], control_points[point_next_id], t);
      }
      new_positions.last() = control_points.last();
      return;
    }
    case PrimitiveType::ARC: {
      const int num_shared_points = control_points_per_segment(ptd);
      const int num_segments = ptd.segments;
      for (const int segment_i : IndexRange(num_segments)) {
        const float3 A = control_points[num_shared_points * segment_i + 0];
        const float3 B = control_points[num_shared_points * segment_i + 1];
        const float3 C = control_points[num_shared_points * segment_i + 2];
        for (const int i : IndexRange(subdivision + 1)) {
          const float t = i / float(subdivision + 1);
          const float3 AB = math::interpolate(A, B, t);
          const float3 BC = math::interpolate(B, C, t);
          new_positions[i + segment_i * (subdivision + 1)] = math::interpolate(AB, BC, t);
        }
      }
      new_positions.last() = control_points.last();
      return;
    }
    case PrimitiveType::CURVE: {
      const int num_shared_points = control_points_per_segment(ptd);
      const int num_segments = ptd.segments;

      for (const int segment_i : IndexRange(num_segments)) {
        const float3 A = control_points[num_shared_points * segment_i + 0];
        const float3 B = control_points[num_shared_points * segment_i + 1];
        const float3 C = control_points[num_shared_points * segment_i + 2];
        const float3 D = control_points[num_shared_points * segment_i + 3];
        for (const int i : IndexRange(subdivision + 1)) {
          const float t = i / float(subdivision + 1);
          const float3 AB = math::interpolate(A, B, t);
          const float3 BC = math::interpolate(B, C, t);
          const float3 CD = math::interpolate(C, D, t);
          const float3 ABBC = math::interpolate(AB, BC, t);
          const float3 BCCD = math::interpolate(BC, CD, t);
          new_positions[i + segment_i * (subdivision + 1)] = math::interpolate(ABBC, BCCD, t);
        }
      }
      new_positions.last() = control_points.last();
      return;
    }
    case PrimitiveType::CIRCLE: {
      const float3 center = control_points[1];
      const float3 A = control_points.first() - center;
      const float3 B = control_points.last() - center;
      for (const int i : new_positions.index_range()) {
        const float t = i / float(new_points_num);
        const float a = t * math::numbers::pi * 2.0f;
        new_positions[i] = A * sinf(a) + B * cosf(a) + center;
      }
      return;
    }
    case PrimitiveType::BOX: {
      const float3 center = control_points[1];
      const float3 A = control_points.first();
      const float3 B = control_points.last();
      const float3 C = -(A - center) + center;
      const float3 D = -(B - center) + center;
      /*
       *
       * +-----------+
       * |A         B|
       * |           |
       * |   center  |
       * |           |
       * |D         C|
       * +-----------+
       *
       */
      const float3 corners[4] = {A, B, C, D};
      for (const int i : new_positions.index_range()) {
        const float t = math::mod(i / float(subdivision + 1), 1.0f);
        const int point_id = int(i / (subdivision + 1));
        const int point_next_id = math::mod(point_id + 1, 4);
        new_positions[i] = math::interpolate(corners[point_id], corners[point_next_id], t);
      }
      return;
    }
  }
}

static void primitive_calulate_curve_positions_3d(PrimitiveTool_OpData &ptd,
                                                  MutableSpan<float3> new_positions)
{
  primitive_calulate_curve_positions_3d_exec(ptd, ptd.control_points, new_positions);
}

static void primitive_calulate_curve_positions_2d(PrimitiveTool_OpData &ptd,
                                                  MutableSpan<float2> new_positions)
{
  Array<float3> control_points_2d(ptd.control_points.size());

  for (const int i : ptd.control_points.index_range()) {
    control_points_2d[i] = float3(
        ED_view3d_project_float_v2_m4(ptd.vc.region, ptd.control_points[i], ptd.projection));
  }

  Array<float3> new_positions_3d(new_positions.size());
  primitive_calulate_curve_positions_3d_exec(ptd, control_points_2d, new_positions_3d);

  /* TODO(FIXME): Currently we are projecting the 3D points into 2D space but then storing them in
   * float3 so that they can be past in to `primitive_calulate_curve_positions_3d_exec` before
   * being converting back to float2. */
  for (const int i : new_positions.index_range()) {
    new_positions[i] = float2(new_positions_3d[i]);
  }
}

static int grease_pencil_primitive_curve_points_number(PrimitiveTool_OpData &ptd)
{
  const int subdivision = ptd.subdivision;

  switch (ptd.type) {
    case PrimitiveType::POLYLINE:
    case PrimitiveType::CURVE:
    case PrimitiveType::LINE:
    case PrimitiveType::CIRCLE:
    case PrimitiveType::ARC: {
      const int join_points = ptd.segments + 1;
      return join_points + subdivision * ptd.segments;
      break;
    }
    case PrimitiveType::BOX: {
      return 4 + subdivision * 4;
      break;
    }
  }

  BLI_assert_unreachable();
  return 0;
}

static void grease_pencil_primitive_update_curves(PrimitiveTool_OpData &ptd)
{
  bke::CurvesGeometry &curves = ptd.drawing_->strokes_for_write();
  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();

  const int last_points = curves.points_by_curve()[curves.curves_range().last()].size();

  int new_points_num = grease_pencil_primitive_curve_points_number(ptd);

  curves.resize(curves.points_num() - last_points + new_points_num, curves.curves_num());
  curves.offsets_for_write().last() = curves.points_num();
  const IndexRange curve_points = curves.points_by_curve()[curves.curves_range().last()];

  MutableSpan<float3> positions_3d = curves.positions_for_write().slice(curve_points);
  Array<float2> positions_2d(new_points_num);

  if (ptd.interpolate_mode == InterpolationMode::Mode3D) {
    primitive_calulate_curve_positions_3d(ptd, positions_3d);

    for (const int point : positions_3d.index_range()) {
      positions_2d[point] = ED_view3d_project_float_v2_m4(
          ptd.vc.region, positions_3d[point], ptd.projection);
    }
    ptd.placement_.project(positions_2d, positions_3d);
  }
  else { /* 2D */
    primitive_calulate_curve_positions_2d(ptd, positions_2d);
    ptd.placement_.project(positions_2d, positions_3d);
  }

  MutableSpan<float> new_radii = ptd.drawing_->radii_for_write().slice(curve_points);
  MutableSpan<float> new_opacities = ptd.drawing_->opacities_for_write().slice(curve_points);
  MutableSpan<ColorGeometry4f> new_vertex_colors = ptd.drawing_->vertex_colors_for_write().slice(
      curve_points);

  new_vertex_colors.fill(ColorGeometry4f(ptd.vertex_color_));

  ToolSettings *ts = ptd.vc.scene->toolsettings;
  GP_Sculpt_Settings *gset = &ts->gp_sculpt;

  for (const int point_id : curve_points.index_range()) {
    float pressure = 1.0f;
    if (gset->flag & GP_SCULPT_SETT_FLAG_PRIMITIVE_CURVE) {
      const float t = point_id / float(new_points_num - 1);
      pressure = BKE_curvemapping_evaluateF(gset->cur_primitive, 0, t);
    }

    const float radius = ed::sculpt_paint::greasepencil::radius_from_input_sample(
        pressure, positions_3d[point_id], ptd.vc, ptd.brush_, ptd.vc.scene, ptd.settings_);
    const float opacity = ed::sculpt_paint::greasepencil::opacity_from_input_sample(
        pressure, ptd.brush_, ptd.vc.scene, ptd.settings_);

    new_radii[point_id] = radius;
    new_opacities[point_id] = opacity;
  }

  ptd.drawing_->tag_topology_changed();
}

static void grease_pencil_primitive_init_curves(PrimitiveTool_OpData &ptd)
{
  /* Resize the curves geometry so there is one more curve with a single point. */
  bke::CurvesGeometry &curves = ptd.drawing_->strokes_for_write();
  const int num_old_points = curves.points_num();
  curves.resize(curves.points_num() + 1, curves.curves_num() + 1);
  curves.offsets_for_write().last(1) = num_old_points;

  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  bke::SpanAttributeWriter<int> materials = attributes.lookup_or_add_for_write_span<int>(
      "material_index", bke::AttrDomain::Curve);
  bke::SpanAttributeWriter<bool> cyclic = attributes.lookup_or_add_for_write_span<bool>(
      "cyclic", bke::AttrDomain::Curve);
  bke::SpanAttributeWriter<float> hardnesses = attributes.lookup_or_add_for_write_span<float>(
      "hardness",
      bke::AttrDomain::Curve,
      bke::AttributeInitVArray(VArray<float>::ForSingle(1.0f, curves.curves_num())));

  cyclic.span.last() = ELEM(ptd.type, PrimitiveType::BOX, PrimitiveType::CIRCLE);
  materials.span.last() = ptd.material_index;
  hardnesses.span.last() = ptd.hardness;

  cyclic.finish();
  materials.finish();
  hardnesses.finish();

  curves.curve_types_for_write().last() = CURVE_TYPE_POLY;
  curves.update_curve_types();

  /* Initialize the rest of the attributes with default values. */
  bke::fill_attribute_range_default(attributes,
                                    bke::AttrDomain::Point,
                                    {"position", "radius", "opacity", "vertex_color"},
                                    curves.points_range().take_back(1));
  bke::fill_attribute_range_default(attributes,
                                    bke::AttrDomain::Curve,
                                    {"curve_type", "material_index", "cyclic", "hardness"},
                                    curves.curves_range().take_back(1));

  grease_pencil_primitive_update_curves(ptd);
}

static void grease_pencil_primitive_undo_curves(PrimitiveTool_OpData &ptd)
{
  bke::CurvesGeometry &curves = ptd.drawing_->strokes_for_write();
  curves.remove_curves(IndexMask({curves.curves_range().last(), 1}), {});
  ptd.drawing_->tag_topology_changed();
}

/* Helper: Draw status message while the user is running the operator */
static void grease_pencil_primitive_status_indicators(bContext *C,
                                                      wmOperator *op,
                                                      PrimitiveTool_OpData &ptd)
{
  std::string header;

  switch (ptd.type) {
    case PrimitiveType::LINE: {
      header += RPT_("Line: ");
      break;
    }
    case (PrimitiveType::POLYLINE): {
      header += RPT_("Polyline: ");
      break;
    }
    case (PrimitiveType::BOX): {
      header += RPT_("Rectangle: ");
      break;
    }
    case (PrimitiveType::CIRCLE): {
      header += RPT_("Circle: ");
      break;
    }
    case (PrimitiveType::ARC): {
      header += RPT_("Arc: ");
      break;
    }
    case (PrimitiveType::CURVE): {
      header += RPT_("Curve: ");
      break;
    }
  }

  auto get_modal_key_str = [&](ModelKeyMode id) {
    return WM_modalkeymap_operator_items_to_string(op->type, int(id), true).value_or("");
  };

  header += fmt::format(IFACE_("{}: confirm, {}: cancel, {}: panning, Shift: align"),
                        get_modal_key_str(ModelKeyMode::CONFIRM),
                        get_modal_key_str(ModelKeyMode::CANCEL),
                        get_modal_key_str(ModelKeyMode::PANNING));

  header += fmt::format(IFACE_(", {}/{}: adjust subdivisions: {}"),
                        get_modal_key_str(ModelKeyMode::INCREASE_SUBDIVISION),
                        get_modal_key_str(ModelKeyMode::DECREASE_SUBDIVISION),
                        int(ptd.subdivision));

  if (ptd.segments == 1) {
    header += IFACE_(", Alt: center");
  }

  if (ELEM(ptd.type,
           PrimitiveType::LINE,
           PrimitiveType::POLYLINE,
           PrimitiveType::ARC,
           PrimitiveType::CURVE))
  {
    header += fmt::format(IFACE_(", {}: extrude"), get_modal_key_str(ModelKeyMode::EXTRUDE));
  }

  header += fmt::format(IFACE_(", {}: grab, {}: rotate, {}: scale"),
                        get_modal_key_str(ModelKeyMode::GRAB),
                        get_modal_key_str(ModelKeyMode::ROTATE),
                        get_modal_key_str(ModelKeyMode::SCALE));

  ED_workspace_status_text(C, header.c_str());
}

static void grease_pencil_primitive_update_view(bContext *C, PrimitiveTool_OpData &ptd)
{
  GreasePencil *grease_pencil = static_cast<GreasePencil *>(ptd.vc.obact->data);

  DEG_id_tag_update(&grease_pencil->id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, grease_pencil);

  ED_region_tag_redraw(ptd.region);
}

/* Invoke handler: Initialize the operator */
static int grease_pencil_primitive_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  int return_value = ed::sculpt_paint::grease_pencil_draw_operator_invoke(C, op);
  if (return_value != OPERATOR_RUNNING_MODAL) {
    return return_value;
  }

  /* if in tools region, wait till we get to the main (3D-space)
   * region before allowing drawing to take place.
   */
  op->flag |= OP_IS_MODAL_CURSOR_REGION;

  wmWindow *win = CTX_wm_window(C);

  /* set cursor to indicate modal */
  WM_cursor_modal_set(win, WM_CURSOR_CROSS);

  ViewContext vc = ED_view3d_viewcontext_init(C, CTX_data_depsgraph_pointer(C));

  /* Allocate new data. */
  PrimitiveTool_OpData *ptd_pointer = MEM_new<PrimitiveTool_OpData>(__func__);
  op->customdata = ptd_pointer;

  PrimitiveTool_OpData &ptd = *ptd_pointer;

  ptd.vc = vc;
  ptd.region = vc.region;
  View3D *view3d = CTX_wm_view3d(C);
  const float2 start_coords = float2(event->mval);

  GreasePencil *grease_pencil = static_cast<GreasePencil *>(vc.obact->data);

  /* Initialize helper class for projecting screen space coordinates. */
  ed::greasepencil::DrawingPlacement placement_ = ed::greasepencil::DrawingPlacement(
      *vc.scene, *vc.region, *view3d, *vc.obact, *grease_pencil->get_active_layer());
  if (placement_.use_project_to_surface()) {
    placement_.cache_viewport_depths(CTX_data_depsgraph_pointer(C), vc.region, view3d);
  }
  else if (placement_.use_project_to_nearest_stroke()) {
    placement_.cache_viewport_depths(CTX_data_depsgraph_pointer(C), vc.region, view3d);
    placement_.set_origin_to_nearest_stroke(start_coords);
  }

  ptd.placement_ = placement_;

  wmWindowManager *wm = CTX_wm_manager(C);
  wmKeyMap *keymap = WM_keymap_active(wm, op->type->modalkeymap);
  const wmKeyMapItem *kmi_passthrough = nullptr;
  // printf("\n");
  LISTBASE_FOREACH (const wmKeyMapItem *, kmi, &keymap->items) {
    // printf("kmi->propvalue %i \n", kmi->propvalue);
    if (kmi->flag & KMI_INACTIVE) {
      continue;
    }

    // if (kmi->propvalue == TFM_MODAL_PASSTHROUGH_NAVIGATE) {
    // if (kmi->propvalue == int(ModelKeyMode::PANNING)) {
    //   kmi_passthrough = kmi;
    //   break;
    // }
  }
  // printf("\n");

  ptd.vod = ED_view3d_navigation_init(C, kmi_passthrough);

  ptd.subdivision = RNA_int_get(op->ptr, "subdivision");
  ptd.type = PrimitiveType(RNA_enum_get(op->ptr, "type"));
  ptd.interpolate_mode = InterpolationMode(RNA_boolean_get(op->ptr, "interpolate_mode"));
  ptd.control_points = Vector<float3>();
  ptd.temp_control_points = Vector<float3>();

  ptd.mode = OperatorMode::EXTRUDING;
  ptd.start_position_2d = start_coords;
  const float3 pos = ptd.placement_.project(ptd.start_position_2d);
  ptd.segments = 1;
  /* Add one point for the beginning. */
  ptd.control_points.append_n_times(pos, control_points_per_segment(ptd) + 1);
  ptd.active_control_point_index = -1;

  Paint *paint = &vc.scene->toolsettings->gp_paint->paint;
  ptd.brush_ = BKE_paint_brush(paint);
  ptd.settings_ = ptd.brush_->gpencil_settings;

  BKE_curvemapping_init(ptd.settings_->curve_sensitivity);
  BKE_curvemapping_init(ptd.settings_->curve_strength);
  BKE_curvemapping_init(ptd.settings_->curve_jitter);
  BKE_curvemapping_init(ptd.settings_->curve_rand_pressure);
  BKE_curvemapping_init(ptd.settings_->curve_rand_strength);
  BKE_curvemapping_init(ptd.settings_->curve_rand_uv);
  BKE_curvemapping_init(ptd.settings_->curve_rand_hue);
  BKE_curvemapping_init(ptd.settings_->curve_rand_saturation);
  BKE_curvemapping_init(ptd.settings_->curve_rand_value);

  ToolSettings *ts = vc.scene->toolsettings;
  GP_Sculpt_Settings *gset = &ts->gp_sculpt;
  if (gset->flag & GP_SCULPT_SETT_FLAG_PRIMITIVE_CURVE) {
    BKE_curvemapping_init(ts->gp_sculpt.cur_primitive);
  }

  Material *material = BKE_grease_pencil_object_material_ensure_from_active_input_brush(
      CTX_data_main(C), vc.obact, ptd.brush_);
  ptd.material_index = BKE_object_material_index_get(vc.obact, material);

  const bool use_vertex_color = (vc.scene->toolsettings->gp_paint->mode ==
                                 GPPAINT_FLAG_USE_VERTEXCOLOR);
  const bool use_vertex_color_stroke = use_vertex_color && ELEM(ptd.settings_->vertex_mode,
                                                                GPPAINT_MODE_STROKE,
                                                                GPPAINT_MODE_BOTH);
  ptd.vertex_color_ = use_vertex_color_stroke ? float4(ptd.brush_->rgb[0],
                                                       ptd.brush_->rgb[1],
                                                       ptd.brush_->rgb[2],
                                                       ptd.settings_->vertex_factor) :
                                                float4(0.0f);
  srgb_to_linearrgb_v4(ptd.vertex_color_, ptd.vertex_color_);

  /* TODO: Add UI. */
  ptd.hardness = 1.0f;

  BLI_assert(grease_pencil->has_active_layer());
  ptd.drawing_ = grease_pencil->get_editable_drawing_at(*grease_pencil->get_active_layer(),
                                                        vc.scene->r.cfra);

  grease_pencil_primitive_init_curves(ptd);

  grease_pencil_primitive_update_view(C, ptd);

  ptd.draw_handle = ED_region_draw_cb_activate(
      ptd.region->type, grease_pencil_primitive_draw, ptd_pointer, REGION_DRAW_POST_VIEW);

  /* Updates indicator in header. */
  grease_pencil_primitive_status_indicators(C, op, ptd);

  /* add a modal handler for this operator */
  WM_event_add_modal_handler(C, op);

  return OPERATOR_RUNNING_MODAL;
}

/* Exit and free memory */
static void grease_pencil_primitive_exit(bContext *C, wmOperator *op)
{
  PrimitiveTool_OpData *ptd = static_cast<PrimitiveTool_OpData *>(op->customdata);

  /* Clear status message area. */
  ED_workspace_status_text(C, nullptr);

  WM_cursor_modal_restore(ptd->vc.win);

  /* Deactivate the extra drawing stuff in 3D-View. */
  ED_region_draw_cb_exit(ptd->region->type, ptd->draw_handle);

  ED_view3d_navigation_free(C, ptd->vod);

  grease_pencil_primitive_update_view(C, *ptd);

  MEM_delete<PrimitiveTool_OpData>(ptd);
  /* Clear pointer. */
  op->customdata = nullptr;
}

static float2 snap_diagonals(float2 p)
{
  using namespace math;
  return sign(p) * float2(1.0f / numbers::sqrt2) * length(p);
}

/* Uses Chebychev distance instead of Euclidean. */
static float2 snap_diagonals_box(float2 p)
{
  using namespace math;
  return sign(p) * float2(std::max(abs(p[0]), abs(p[1])));
}

/* Snaps to diagonals, horizontal and vertical. */
static float2 snap_8_angles(float2 p)
{
  using namespace math;
  /* sin(pi/8) or sin of 22.5 degrees.*/
  const float sin225 = 0.3826834323650897717284599840304f;
  return sign(p) * length(p) * normalize(sign(normalize(abs(p)) - sin225) + 1.0f);
}

static void grease_pencil_primitive_extruding_update(PrimitiveTool_OpData &ptd,
                                                     const wmEvent *event)
{
  using namespace math;
  const float2 start = ptd.start_position_2d;
  const float2 end = float2(event->mval);

  const float2 dif = end - start;
  float2 offset = dif;

  if (event->modifier & KM_SHIFT) {
    if (ptd.type == PrimitiveType::BOX) {
      offset = snap_diagonals_box(dif);
    }
    else if (ptd.type == PrimitiveType::CIRCLE) {
      offset = snap_diagonals(dif);
    }
    else { /* Line, Polyline, Arc and Curve. */
      offset = snap_8_angles(dif);
    }
  }
  offset *= 0.5f;

  float2 center = start + offset;

  if (event->modifier & KM_ALT && ptd.segments == 1) {
    center = start;
    offset *= 2.0f;
  }

  const float3 start_pos = ptd.placement_.project(center - offset);
  const float3 end_pos = ptd.placement_.project(center + offset);

  switch (ptd.type) {
    case PrimitiveType::BOX: {

      ptd.control_points[0] = ptd.placement_.project(offset * float2(-1.0f, 1.0f) + center);
      ptd.control_points[1] = ptd.placement_.project(center);
      ptd.control_points[2] = ptd.placement_.project(offset * float2(1.0f, 1.0f) + center);
      return;
    }
    case PrimitiveType::CIRCLE: {

      ptd.control_points[0] = ptd.placement_.project(offset * float2(1.0f, 0.0f) + center);
      ptd.control_points[1] = ptd.placement_.project(center);
      ptd.control_points[2] = ptd.placement_.project(offset * float2(0.0f, 1.0f) + center);
      return;
    }
    case PrimitiveType::POLYLINE:
    case PrimitiveType::LINE: {
      ptd.control_points.last(1) = start_pos;
      ptd.control_points.last(0) = end_pos;
      return;
    }
    case PrimitiveType::ARC: {
      /* Linear interpolation. */
      ptd.control_points.last(2) = start_pos;
      ptd.control_points.last(1) = interpolate(start_pos, end_pos, 1.0f / 2.0f);
      ptd.control_points.last(0) = end_pos;
      return;
    }
    case PrimitiveType::CURVE: {
      /* Linear interpolation. */
      ptd.control_points.last(3) = start_pos;
      ptd.control_points.last(2) = interpolate(start_pos, end_pos, 1.0f / 3.0f);
      ptd.control_points.last(1) = interpolate(start_pos, end_pos, 2.0f / 3.0f);
      ptd.control_points.last(0) = end_pos;
      return;
    }
  }
}

static void grease_pencil_primitive_grab_update(PrimitiveTool_OpData &ptd, const wmEvent *event)
{
  BLI_assert(ptd.active_control_point_index != -1);
  float3 pos = ptd.placement_.project(float2(event->mval));
  ptd.control_points[ptd.active_control_point_index] = pos;
}

static void grease_pencil_primitive_drag_update(PrimitiveTool_OpData &ptd, const wmEvent *event)
{
  BLI_assert(ptd.active_control_point_index != -1);
  const float2 start = ptd.start_position_2d;
  const float2 end = float2(event->mval);
  const float2 dif = end - start;

  const float2 start_pos2 = ED_view3d_project_float_v2_m4(
      ptd.vc.region, ptd.temp_control_points[ptd.active_control_point_index], ptd.projection);

  float3 pos = ptd.placement_.project(start_pos2 + dif);
  ptd.control_points[ptd.active_control_point_index] = pos;
}

static void grease_pencil_primitive_drag_all_update(PrimitiveTool_OpData &ptd,
                                                    const wmEvent *event)
{
  const float2 start = ptd.start_position_2d;
  const float2 end = float2(event->mval);
  const float2 dif = end - start;

  for (const int point_index : ptd.control_points.index_range()) {
    const float2 start_pos2 = ED_view3d_project_float_v2_m4(
        ptd.vc.region, ptd.temp_control_points[point_index], ptd.projection);

    float3 pos = ptd.placement_.project(start_pos2 + dif);
    ptd.control_points[point_index] = pos;
  }
}

static float2 primitive_center_of_mass(const PrimitiveTool_OpData &ptd)
{
  if (ELEM(ptd.type, PrimitiveType::BOX, PrimitiveType::CIRCLE)) {
    return ED_view3d_project_float_v2_m4(
        ptd.vc.region, ptd.temp_control_points[1], ptd.projection);
  }
  float2 center_of_mass = float2(0.0f, 0.0f);

  for (const int point_index : ptd.control_points.index_range()) {
    center_of_mass += ED_view3d_project_float_v2_m4(
        ptd.vc.region, ptd.temp_control_points[point_index], ptd.projection);
  }
  center_of_mass /= ptd.control_points.size();
  return center_of_mass;
}

static void grease_pencil_primitive_rotate_all_update(PrimitiveTool_OpData &ptd,
                                                      const wmEvent *event)
{
  const float2 start = ptd.start_position_2d;
  const float2 end = float2(event->mval);

  const float2 center_of_mass = primitive_center_of_mass(ptd);

  const float2 end_ = end - center_of_mass;
  const float2 start_ = start - center_of_mass;
  const float rotation = math::atan2(start_[0], start_[1]) - math::atan2(end_[0], end_[1]);

  for (const int point_index : ptd.control_points.index_range()) {
    const float2 start_pos2 = ED_view3d_project_float_v2_m4(
        ptd.vc.region, ptd.temp_control_points[point_index], ptd.projection);

    const float2 dif = start_pos2 - center_of_mass;
    const float c = math::cos(rotation);
    const float s = math::sin(rotation);
    const float2 pos2 = float2(dif[0] * c - dif[1] * s, dif[0] * s + dif[1] * c) + center_of_mass;
    float3 pos = ptd.placement_.project(pos2);
    ptd.control_points[point_index] = pos;
  }
}

static void grease_pencil_primitive_scale_all_update(PrimitiveTool_OpData &ptd,
                                                     const wmEvent *event)
{
  const float2 start = ptd.start_position_2d;
  const float2 end = float2(event->mval);

  const float2 center_of_mass = primitive_center_of_mass(ptd);

  const float scale = math::length(end - center_of_mass) / math::length(start - center_of_mass);

  for (const int point_index : ptd.control_points.index_range()) {
    const float2 start_pos2 = ED_view3d_project_float_v2_m4(
        ptd.vc.region, ptd.temp_control_points[point_index], ptd.projection);

    const float2 pos2 = (start_pos2 - center_of_mass) * scale + center_of_mass;
    float3 pos = ptd.placement_.project(pos2);
    ptd.control_points[point_index] = pos;
  }
}

static int primitive_check_ui_hover(const PrimitiveTool_OpData &ptd, const wmEvent *event)
{
  float closest_distance_squared = std::numeric_limits<float>::max();
  int closest_point = -1;

  for (const int i : ptd.control_points.index_range()) {
    const int point_id = (ptd.control_points.size() - 1) - i;
    const float2 pos_proj = ED_view3d_project_float_v2_m4(
        ptd.vc.region, ptd.control_points[point_id], ptd.projection);
    const float radius_sq = UI_POINT_HIT_SIZE_PX * UI_POINT_HIT_SIZE_PX;
    const float distance_squared = math::distance_squared(pos_proj, float2(event->mval));
    if (distance_squared <= radius_sq) {
      return point_id;
    }

    const ControlPointType control_point_type = get_control_point_type(ptd, point_id);

    if (distance_squared < closest_distance_squared &&
        control_point_type == ControlPointType::EXTRINSIC_POINT &&
        distance_squared < UI_POINT_MAX_HIT_SIZE_PX * UI_POINT_MAX_HIT_SIZE_PX)
    {
      closest_point = point_id;
      closest_distance_squared = distance_squared;
    }
  }

  if (closest_point != -1) {
    return closest_point;
  }

  return -1;
}

static void grease_pencil_primitive_cursor_update(bContext *C,
                                                  PrimitiveTool_OpData &ptd,
                                                  const wmEvent *event)
{
  wmWindow *win = CTX_wm_window(C);

  if (ptd.mode != OperatorMode::IDLE) {
    WM_cursor_modal_set(win, WM_CURSOR_CROSS);
    return;
  }

  const int ui_id = primitive_check_ui_hover(ptd, event);
  ptd.active_control_point_index = ui_id;
  if (ui_id == -1) {
    if (ptd.type == PrimitiveType::POLYLINE) {
      WM_cursor_modal_set(win, WM_CURSOR_CROSS);
      return;
    }

    WM_cursor_modal_set(win, WM_CURSOR_HAND);
    return;
  }

  const ControlPointType control_point_type = get_control_point_type(ptd, ui_id);

  if (control_point_type == ControlPointType::JOIN_POINT) {
    WM_cursor_modal_set(win, WM_CURSOR_NSEW_SCROLL);
    return;
  }
  else if (control_point_type == ControlPointType::EXTRINSIC_POINT) {
    WM_cursor_modal_set(win, WM_CURSOR_NSEW_SCROLL);
    return;
  }
}

/* Modal handler: Events handling during interactive part. */
static int grease_pencil_primitive_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  PrimitiveTool_OpData &ptd = *((PrimitiveTool_OpData *)op->customdata);

  const float3 pos = ptd.control_points.first();
  if (ED_view3d_navigation_do(C, ptd.vod, event, pos)) {
    if (ptd.vc.rv3d->rflag & RV3D_NAVIGATING) {
      ptd.projection = ED_view3d_ob_project_mat_get(ptd.vc.rv3d, ptd.vc.obact);

      grease_pencil_primitive_update_curves(ptd);
      grease_pencil_primitive_update_view(C, ptd);

      return OPERATOR_RUNNING_MODAL;
    }
  }

  ptd.projection = ED_view3d_ob_project_mat_get(ptd.vc.rv3d, ptd.vc.obact);
  // printf("\naaaaaaaaaaaaaaaaaa-----------aaaaaaaaaaaaaaaaaa\n\n");

  // WM_event_print(event);

  grease_pencil_primitive_cursor_update(C, ptd, event);

  if (event->type == EVT_MODAL_MAP) {
    switch (event->val) {
      case int(ModelKeyMode::CANCEL): {
        grease_pencil_primitive_undo_curves(ptd);
        grease_pencil_primitive_exit(C, op);

        return OPERATOR_CANCELLED;
      }
      case int(ModelKeyMode::CONFIRM): {
        grease_pencil_primitive_exit(C, op);

        return OPERATOR_FINISHED;
      }
      case int(ModelKeyMode::EXTRUDE): {
        if (ptd.mode == OperatorMode::IDLE &&
            ELEM(ptd.type, PrimitiveType::LINE, PrimitiveType::ARC, PrimitiveType::CURVE))
        {
          ptd.mode = OperatorMode::EXTRUDING;
          ptd.start_position_2d = ED_view3d_project_float_v2_m4(
              ptd.vc.region, ptd.control_points.last(), ptd.projection);
          const float3 pos = ptd.placement_.project(ptd.start_position_2d);

          const int number_control_points = control_points_per_segment(ptd);
          ptd.control_points.append_n_times(pos, number_control_points);
          ptd.active_control_point_index = -1;
          ptd.segments++;
          break;
        }

        if (ptd.type == PrimitiveType::POLYLINE &&
            ELEM(ptd.mode, OperatorMode::IDLE, OperatorMode::EXTRUDING))
        {
          ptd.mode = OperatorMode::EXTRUDING;
          ptd.start_position_2d = ED_view3d_project_float_v2_m4(
              ptd.vc.region, ptd.control_points.last(), ptd.projection);
          ptd.control_points.append(ptd.placement_.project(float2(event->mval)));
          ptd.active_control_point_index = -1;
          ptd.segments++;

          break;
        }

        break;
      }
      // case int(ModelKeyMode::LEFTCLICK): {
      //   printf("LEFTCLICK\n");
      //   break;
      // }
      case int(ModelKeyMode::GRAB): {
        if (ptd.mode == OperatorMode::IDLE) {
          ptd.start_position_2d = float2(event->mval);
          ptd.mode = OperatorMode::DRAG_ALL;

          ptd.temp_control_points.resize(ptd.control_points.size());
          array_utils::copy(ptd.control_points.as_span(),
                            ptd.temp_control_points.as_mutable_span());
        }
        break;
      }
      case int(ModelKeyMode::ROTATE): {
        if (ptd.mode == OperatorMode::IDLE) {
          ptd.start_position_2d = float2(event->mval);
          ptd.mode = OperatorMode::ROTATE_ALL;

          ptd.temp_control_points.resize(ptd.control_points.size());
          array_utils::copy(ptd.control_points.as_span(),
                            ptd.temp_control_points.as_mutable_span());
        }
        break;
      }
      case int(ModelKeyMode::SCALE): {
        if (ptd.mode == OperatorMode::IDLE) {
          ptd.start_position_2d = float2(event->mval);
          ptd.mode = OperatorMode::SCALE_ALL;

          ptd.temp_control_points.resize(ptd.control_points.size());
          array_utils::copy(ptd.control_points.as_span(),
                            ptd.temp_control_points.as_mutable_span());
        }
        break;
      }
      case int(ModelKeyMode::INCREASE_SUBDIVISION): {
        if (event->val != KM_RELEASE) {
          ptd.subdivision++;
          RNA_int_set(op->ptr, "subdivision", ptd.subdivision);
        }
        break;
      }
      case int(ModelKeyMode::DECREASE_SUBDIVISION): {
        if (event->val != KM_RELEASE) {
          ptd.subdivision--;
          ptd.subdivision = std::max(ptd.subdivision, 0);
          RNA_int_set(op->ptr, "subdivision", ptd.subdivision);
        }
        break;
      }
    }
  }

  switch (event->type) {
    case LEFTMOUSE: {
      if (event->val == KM_RELEASE && ELEM(ptd.mode,
                                           OperatorMode::GRAB,
                                           OperatorMode::DRAG,
                                           OperatorMode::EXTRUDING,
                                           OperatorMode::DRAG_ALL,
                                           OperatorMode::ROTATE_ALL,
                                           OperatorMode::SCALE_ALL))
      {
        ptd.mode = OperatorMode::IDLE;
        break;
      }

      if (ptd.mode == OperatorMode::IDLE && event->val == KM_PRESS) {
        const int ui_id = primitive_check_ui_hover(ptd, event);
        ptd.active_control_point_index = ui_id;
        if (ui_id == -1) {
          if (ptd.type != PrimitiveType::POLYLINE) {
            ptd.start_position_2d = float2(event->mval);
            ptd.mode = OperatorMode::DRAG_ALL;

            ptd.temp_control_points.resize(ptd.control_points.size());
            array_utils::copy(ptd.control_points.as_span(),
                              ptd.temp_control_points.as_mutable_span());

            break;
          }
        }
        else {
          const ControlPointType control_point_type = get_control_point_type(ptd, ui_id);

          if (control_point_type == ControlPointType::JOIN_POINT) {
            ptd.start_position_2d = ED_view3d_project_float_v2_m4(
                ptd.vc.region, ptd.control_points[ptd.active_control_point_index], ptd.projection);
            ptd.mode = OperatorMode::GRAB;
            break;
          }
          else if (control_point_type == ControlPointType::EXTRINSIC_POINT) {
            ptd.start_position_2d = float2(event->mval);
            ptd.mode = OperatorMode::DRAG;

            ptd.temp_control_points.resize(ptd.control_points.size());
            array_utils::copy(ptd.control_points.as_span(),
                              ptd.temp_control_points.as_mutable_span());

            break;
          }
        }
      }

      if (ptd.type == PrimitiveType::POLYLINE && ptd.mode == OperatorMode::IDLE &&
          event->val == KM_PRESS)
      {
        ptd.mode = OperatorMode::EXTRUDING;
        ptd.start_position_2d = ED_view3d_project_float_v2_m4(
            ptd.vc.region, ptd.control_points.last(), ptd.projection);
        ptd.control_points.append(ptd.placement_.project(float2(event->mval)));
        ptd.segments++;
        break;
      }

      break;
    }
  }

  /* Updating is done every event not just MOUSEMOVE. */
  switch (ptd.mode) {
    case OperatorMode::EXTRUDING: {
      grease_pencil_primitive_extruding_update(ptd, event);
      break;
    }
    case OperatorMode::GRAB: {
      grease_pencil_primitive_grab_update(ptd, event);
      break;
    }
    case OperatorMode::DRAG: {
      grease_pencil_primitive_drag_update(ptd, event);
      break;
    }
    case OperatorMode::DRAG_ALL: {
      grease_pencil_primitive_drag_all_update(ptd, event);
      break;
    }
    case OperatorMode::SCALE_ALL: {
      grease_pencil_primitive_scale_all_update(ptd, event);
      break;
    }
    case OperatorMode::ROTATE_ALL: {
      grease_pencil_primitive_rotate_all_update(ptd, event);
      break;
    }
    case OperatorMode::IDLE: {
    }
  }

  grease_pencil_primitive_update_curves(ptd);

  /* Updates indicator in header. */
  grease_pencil_primitive_status_indicators(C, op, ptd);

  grease_pencil_primitive_update_view(C, ptd);

  /* Still running... */
  return OPERATOR_RUNNING_MODAL;
}

/* Cancel handler */
static void grease_pencil_primitive_cancel(bContext *C, wmOperator *op)
{
  /* This is just a wrapper around exit() */
  grease_pencil_primitive_exit(C, op);
}

static void grease_pencil_primitive_common_props(wmOperatorType *ot,
                                                 const int default_subdiv,
                                                 const PrimitiveType default_type)
{
  /* Callbacks. */
  ot->invoke = grease_pencil_primitive_invoke;
  ot->modal = grease_pencil_primitive_modal;
  ot->cancel = grease_pencil_primitive_cancel;

  /* Flags. */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;

  static const EnumPropertyItem grease_pencil_primitive_type[] = {
      {int(PrimitiveType::BOX), "BOX", 0, "Box", ""},
      {int(PrimitiveType::LINE), "LINE", 0, "Line", ""},
      {int(PrimitiveType::POLYLINE), "POLYLINE", 0, "Polyline", ""},
      {int(PrimitiveType::CIRCLE), "CIRCLE", 0, "Circle", ""},
      {int(PrimitiveType::ARC), "ARC", 0, "Arc", ""},
      {int(PrimitiveType::CURVE), "CURVE", 0, "Curve", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  prop = RNA_def_int(ot->srna,
                     "subdivision",
                     default_subdiv,
                     0,
                     INT_MAX,
                     "Subdivisions",
                     "Number of subdivisions per segment",
                     0,
                     INT_MAX);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);

  RNA_def_enum(
      ot->srna, "type", grease_pencil_primitive_type, int(default_type), "Type", "Type of shape");

  // /* Internal prop. */
  // prop = RNA_def_boolean(ot->srna, "wait_for_input", true, "Wait for Input", "");
  // RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_boolean(
      ot->srna,
      "interpolate_mode",
      false,
      "Interpolation Mode",
      "Whether the interpolation happens in 2D view space or in 3D world space");
}

void GREASE_PENCIL_OT_primitive_line(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Line Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_line";
  ot->description = "Create predefined grease pencil stroke lines";

  /* Properties, Callbacks and Flags. */
  grease_pencil_primitive_common_props(ot, 6, PrimitiveType::LINE);
}

void GREASE_PENCIL_OT_primitive_polyline(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Polyline Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_polyline";
  ot->description = "Create predefined grease pencil stroke polylines";

  /* Properties, Callbacks and Flags. */
  grease_pencil_primitive_common_props(ot, 6, PrimitiveType::POLYLINE);
}

void GREASE_PENCIL_OT_primitive_arc(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Arc Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_arc";
  ot->description = "Create predefined grease pencil stroke arcs";

  /* Properties, Callbacks and Flags. */
  grease_pencil_primitive_common_props(ot, 62, PrimitiveType::ARC);
}

void GREASE_PENCIL_OT_primitive_curve(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Curve Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_curve";
  ot->description = "Create predefined grease pencil stroke curve shapes";

  /* Properties, Callbacks and Flags. */
  grease_pencil_primitive_common_props(ot, 62, PrimitiveType::CURVE);
}

void GREASE_PENCIL_OT_primitive_box(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Box Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_box";
  ot->description = "Create predefined grease pencil stroke boxs";

  /* Properties, Callbacks and Flags. */
  grease_pencil_primitive_common_props(ot, 3, PrimitiveType::BOX);
}

void GREASE_PENCIL_OT_primitive_circle(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Grease Pencil Circle Shape";
  ot->idname = "GREASE_PENCIL_OT_primitive_circle";
  ot->description = "Create predefined grease pencil stroke circles";

  /* Properties, Callbacks and Flags. */
  grease_pencil_primitive_common_props(ot, 94, PrimitiveType::CIRCLE);
}

}  // namespace blender::ed::greasepencil

void ED_operatortypes_grease_pencil_primitives()
{
  using namespace blender::ed::greasepencil;
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_line);
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_polyline);
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_arc);
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_curve);
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_box);
  WM_operatortype_append(GREASE_PENCIL_OT_primitive_circle);
}

wmKeyMap *ED_primitivetool_modal_keymap(wmKeyConfig *keyconf)
{
  using namespace blender::ed::greasepencil;
  static const EnumPropertyItem modal_items[] = {
      {int(ModelKeyMode::CANCEL), "CANCEL", 0, "Cancel", ""},
      {int(ModelKeyMode::CONFIRM), "CONFIRM", 0, "Confirm", ""},
      {int(ModelKeyMode::PANNING), "PANNING", 0, "Panning", ""},
      {int(ModelKeyMode::LEFTCLICK), "LEFTCLICK", 0, "Left Click", ""},
      {int(ModelKeyMode::EXTRUDE), "EXTRUDE", 0, "Extrude", ""},
      {int(ModelKeyMode::GRAB), "GRAB", 0, "Grab", ""},
      {int(ModelKeyMode::ROTATE), "ROTATE", 0, "Rotate", ""},
      {int(ModelKeyMode::SCALE), "SCALE", 0, "Scale", ""},
      {int(ModelKeyMode::INCREASE_SUBDIVISION),
       "INCREASE_SUBDIVISION",
       0,
       "increase_subdivision",
       ""},
      {int(ModelKeyMode::DECREASE_SUBDIVISION),
       "DECREASE_SUBDIVISION",
       0,
       "decrease_subdivision",
       ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  wmKeyMap *keymap = WM_modalkeymap_find(keyconf, "Primitive Tool Modal Map");

  /* This function is called for each space-type, only needs to add map once. */
  if (keymap && keymap->modal_items) {
    return nullptr;
  }

  keymap = WM_modalkeymap_ensure(keyconf, "Primitive Tool Modal Map", modal_items);

  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_line");
  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_polyline");
  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_arc");
  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_curve");
  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_box");
  WM_modalkeymap_assign(keymap, "GREASE_PENCIL_OT_primitive_circle");

  return keymap;
}
