/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_brush.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"

#include "BLI_index_mask.hh"
#include "BLI_math_vector.hh"
#include "BLI_task.hh"
#include "DEG_depsgraph_query.hh"

#include "DNA_brush_types.h"

#include "DNA_screen_types.h"
#include "DNA_view3d_types.h"
#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "DNA_node_tree_interface_types.h"
#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

float3 mouse_delta_in_world_space(const bContext &C,
                                  const bke::greasepencil::Layer &layer,
                                  const float2 &mouse_delta_win)
{
  const ARegion &region = *CTX_wm_region(&C);
  const RegionView3D &rv3d = *CTX_wm_region_view3d(&C);
  const Depsgraph &depsgraph = *CTX_data_depsgraph_pointer(&C);
  Object &ob_orig = *CTX_data_active_object(&C);
  Object &ob_eval = *DEG_get_evaluated_object(&depsgraph, &ob_orig);

  const float3 layer_origin = layer.to_world_space(ob_eval).location();
  const float zfac = ED_view3d_calc_zfac(&rv3d, layer_origin);
  float3 mouse_delta;
  ED_view3d_win_to_delta(&region, mouse_delta_win, zfac, mouse_delta);

  return mouse_delta;
}

Vector<ed::greasepencil::MutableDrawingInfo> get_drawings_for_sculpt(const bContext &C)
{
  using namespace blender::bke::greasepencil;

  const Scene &scene = *CTX_data_scene(&C);
  Object &ob_orig = *CTX_data_active_object(&C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob_orig.data);
  Paint &paint = *BKE_paint_get_active_from_context(&C);
  const Brush &brush = *BKE_paint_brush(&paint);
  const bool active_layer_only = ((brush.gpencil_settings->flag & GP_BRUSH_ACTIVE_LAYER_ONLY) !=
                                  0);

  if (active_layer_only) {
    /* Apply only to the drawing at the current frame of the active layer. */
    if (!grease_pencil.has_active_layer()) {
      return {};
    }
    const Layer &active_layer = *grease_pencil.get_active_layer();
    Drawing *drawing = grease_pencil.get_editable_drawing_at(active_layer, scene.r.cfra);
    if (drawing == nullptr) {
      return {};
    }

    const int layer_index = *grease_pencil.get_layer_index(active_layer);
    const int frame_number = scene.r.cfra;
    const float multi_frame_falloff = 1.0f;
    return {ed::greasepencil::MutableDrawingInfo{
        *drawing, layer_index, frame_number, multi_frame_falloff}};
  }

  /* Apply to all editable drawings. */
  return ed::greasepencil::retrieve_editable_drawings(scene, grease_pencil);
}

void init_brush(Brush &brush)
{
  if (brush.gpencil_settings == nullptr) {
    BKE_brush_init_gpencil_settings(&brush);
  }
  BLI_assert(brush.gpencil_settings != nullptr);
  BKE_curvemapping_init(brush.gpencil_settings->curve_strength);
  BKE_curvemapping_init(brush.gpencil_settings->curve_sensitivity);
  BKE_curvemapping_init(brush.gpencil_settings->curve_jitter);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_pressure);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_strength);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_uv);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_hue);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_saturation);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_value);
}

static float brush_radius(const Scene &scene, const Brush &brush, const float pressure = 1.0f)
{
  float radius = BKE_brush_size_get(&scene, &brush);
  if (BKE_brush_use_size_pressure(&brush)) {
    radius *= BKE_curvemapping_evaluateF(brush.gpencil_settings->curve_sensitivity, 0, pressure);
  }
  return radius;
}

float brush_influence(const Scene &scene,
                      const Brush &brush,
                      const float2 &co,
                      const InputSample &sample,
                      const float multi_frame_falloff)
{
  const float radius = brush_radius(scene, brush, sample.pressure);
  /* Basic strength factor from brush settings. */
  const bool use_pressure = (brush.gpencil_settings->flag & GP_BRUSH_USE_PRESSURE);
  const float influence_base = brush.alpha * multi_frame_falloff *
                               (use_pressure ? sample.pressure : 1.0f);

  /* Distance falloff. */
  const int2 mval_i = int2(math::round(sample.mouse_position));
  const float distance = math::distance(mval_i, int2(co));
  /* Apply Brush curve. */
  const float brush_falloff = BKE_brush_curve_strength(&brush, distance, radius);

  return influence_base * brush_falloff;
}

IndexMask brush_influence_mask(const Scene &scene,
                               const Brush &brush,
                               const float2 &mouse_position,
                               const float pressure,
                               const float multi_frame_falloff,
                               const IndexMask &selection,
                               const Span<float2> view_positions,
                               Vector<float> &influences,
                               IndexMaskMemory &memory)
{
  if (selection.is_empty()) {
    return {};
  }

  const float radius = brush_radius(scene, brush, pressure);
  const float radius_squared = radius * radius;
  const bool use_pressure = (brush.gpencil_settings->flag & GP_BRUSH_USE_PRESSURE);
  const float influence_base = brush.alpha * multi_frame_falloff *
                               (use_pressure ? pressure : 1.0f);
  const int2 mval_i = int2(math::round(mouse_position));

  Vector<int> indices;
  selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
    /* Distance falloff. */
    const int2 delta = int2(view_positions[point_i]) - mval_i;
    const float distance_squared = math::length_squared(delta);
    if (distance_squared > radius_squared) {
      return;
    }

    /* Apply Brush curve. */
    const float brush_falloff = BKE_brush_curve_strength(
        &brush, math::sqrt(distance_squared), radius);
    const float influence = influence_base * brush_falloff;

    if (influence > 0.0f) {
      indices.append(point_i);
      influences.append(influence);
    }
  });

  return IndexMask::from_indices(indices.as_span(), memory);
}

bool is_brush_inverted(const Brush &brush, const BrushStrokeMode stroke_mode)
{
  /* The basic setting is the brush's setting. */
  bool invert = ((brush.gpencil_settings->sculpt_flag & GP_SCULPT_FLAG_INVERT) != 0) ||
                (brush.gpencil_settings->sculpt_flag & BRUSH_DIR_IN);
  /* During runtime, the user can hold down the Ctrl key to invert the basic behavior. */
  if (stroke_mode == BrushStrokeMode::BRUSH_STROKE_INVERT) {
    invert ^= true;
  }
  /* Set temporary status */
  SET_FLAG_FROM_TEST(brush.gpencil_settings->sculpt_flag, invert, GP_SCULPT_FLAG_TMP_INVERT);
  return invert;
}

GreasePencilStrokeParams GreasePencilStrokeParams::from_context(
    const bContext &C,
    const ARegion &region,
    const int layer_index,
    const int frame_number,
    bke::greasepencil::Drawing &drawing)
{
  const Scene &scene = *CTX_data_scene(&C);
  const Depsgraph &depsgraph = *CTX_data_depsgraph_pointer(&C);
  const View3D &view3d = *CTX_wm_view3d(&C);
  Object &ob_orig = *CTX_data_active_object(&C);
  Object &ob_eval = *DEG_get_evaluated_object(&depsgraph, &ob_orig);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob_orig.data);

  const bke::greasepencil::Layer &layer = *grease_pencil.layers()[layer_index];
  ed::greasepencil::DrawingPlacement placement(scene, region, view3d, ob_eval, layer);

  return {C,
          region,
          ob_eval,
          ob_orig,
          layer,
          layer_index,
          frame_number,
          std::move(placement),
          drawing};
}

IndexMask point_selection_mask(const GreasePencilStrokeParams &params, IndexMaskMemory &memory)
{
  const Scene &scene = *CTX_data_scene(&params.context);
  const bool is_masking = GPENCIL_ANY_SCULPT_MASK(
      eGP_Sculpt_SelectMaskFlag(scene.toolsettings->gpencil_selectmode_sculpt));
  return (is_masking ? ed::greasepencil::retrieve_editable_and_selected_points(
                           params.ob_eval, params.drawing, memory) :
                       params.drawing.strokes().points_range());
}

bke::crazyspace::GeometryDeformation get_drawing_deformation(
    const GreasePencilStrokeParams &params)
{
  return bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
      &params.ob_eval, params.ob_orig, params.layer_index, params.frame_number);
}

Array<float2> calculate_view_positions(const GreasePencilStrokeParams &params,
                                       const IndexMask &selection)
{
  bke::crazyspace::GeometryDeformation deformation = get_drawing_deformation(params);

  Array<float2> view_positions(deformation.positions.size());

  /* Compute screen space positions. */
  const float4x4 transform = params.layer.to_world_space(params.ob_eval);
  selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
    ED_view3d_project_float_global(
        &params.region,
        math::transform_point(transform, deformation.positions[point_i]),
        view_positions[point_i],
        V3D_PROJ_TEST_NOP);
  });

  return view_positions;
}

float2 GreasePencilStrokeOperationCommon::mouse_delta(const InputSample &input_sample) const
{
  return input_sample.mouse_position - this->prev_mouse_position;
}

void GreasePencilStrokeOperationCommon::on_stroke_begin(const bContext &C,
                                                        const InputSample &start_sample)
{
  using namespace blender::bke::greasepencil;

  const ARegion &region = *CTX_wm_region(&C);
  Object &ob_orig = *CTX_data_active_object(&C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob_orig.data);
  Paint &paint = *BKE_paint_get_active_from_context(&C);
  Brush &brush = *BKE_paint_brush(&paint);

  init_brush(brush);

  this->prev_mouse_position = start_sample.mouse_position;

  std::atomic<bool> changed = false;
  const Vector<MutableDrawingInfo> drawings = get_drawings_for_sculpt(C);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    GreasePencilStrokeParams params = GreasePencilStrokeParams::from_context(
        C, region, info.layer_index, info.frame_number, info.drawing);
    if (this->on_stroke_begin_drawing(params, start_sample)) {
      changed = true;
    }
  });

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(&C, NC_GEOM | ND_DATA, &grease_pencil);
  }
}

void GreasePencilStrokeOperationCommon::on_stroke_extended(const bContext &C,
                                                           const InputSample &extension_sample)
{
  using namespace blender::bke::greasepencil;

  const ARegion &region = *CTX_wm_region(&C);
  Object &ob_orig = *CTX_data_active_object(&C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob_orig.data);

  std::atomic<bool> changed = false;
  const Vector<MutableDrawingInfo> drawings = get_drawings_for_sculpt(C);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    GreasePencilStrokeParams params = GreasePencilStrokeParams::from_context(
        C, region, info.layer_index, info.frame_number, info.drawing);
    if (this->on_stroke_extended_drawing(params, extension_sample)) {
      changed = true;
    }
  });

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(&C, NC_GEOM | ND_DATA, &grease_pencil);
  }

  this->prev_mouse_position = extension_sample.mouse_position;
}

void GreasePencilStrokeOperationCommon::on_stroke_done(const bContext & /*C*/) {}

}  // namespace blender::ed::sculpt_paint::greasepencil
