/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_brush.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_brush_types.h"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "DNA_node_tree_interface_types.h"
#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

static float radius_from_input_sample(const Scene &scene,
                                      const Brush &brush,
                                      const InputSample &sample)
{
  float radius = BKE_brush_size_get(&scene, &brush);
  if (BKE_brush_use_size_pressure(&brush)) {
    radius *= BKE_curvemapping_evaluateF(
        brush.gpencil_settings->curve_sensitivity, 0, sample.pressure);
  }
  return radius;
}

float brush_influence(const Scene &scene,
                      const Brush &brush,
                      const int2 &co,
                      const InputSample &sample,
                      const float multi_frame_falloff)
{
  const float radius = radius_from_input_sample(scene, brush, sample);
  /* Basic strength factor from brush settings. */
  const bool use_pressure = (brush.gpencil_settings->flag & GP_BRUSH_USE_PRESSURE);
  const float influence_base = brush.alpha * multi_frame_falloff *
                               (use_pressure ? sample.pressure : 1.0f);

  /* Distance falloff. */
  int2 mval_i;
  round_v2i_v2fl(mval_i, sample.mouse_position);
  const float distance = float(len_v2v2_int(mval_i, co));
  /* Apply Brush curve. */
  const float brush_falloff = BKE_brush_curve_strength(&brush, distance, radius);

  return influence_base * brush_falloff;
}

void GreasePencilStrokeOperationCommon::on_stroke_begin(const bContext &C,
                                                        const InputSample & /*start_sample*/)
{
  Paint *paint = BKE_paint_get_active_from_context(&C);
  Brush *brush = BKE_paint_brush(paint);

  if (brush->gpencil_settings == nullptr) {
    BKE_brush_init_gpencil_settings(brush);
  }
  BLI_assert(brush->gpencil_settings != nullptr);
  BKE_curvemapping_init(brush->gpencil_settings->curve_strength);
  BKE_curvemapping_init(brush->gpencil_settings->curve_sensitivity);
  BKE_curvemapping_init(brush->gpencil_settings->curve_jitter);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_pressure);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_strength);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_uv);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_hue);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_saturation);
  BKE_curvemapping_init(brush->gpencil_settings->curve_rand_value);
}

static bool apply_to_drawing(GreasePencilStrokeOperationCommon &op,
                             const bContext &C,
                             const ARegion &region,
                             const Object &ob_orig,
                             const Object &ob_eval,
                             const bke::greasepencil::Layer &layer,
                             const int layer_index,
                             const int frame_number,
                             bke::greasepencil::Drawing &drawing,
                             const InputSample &extension_sample)
{
  bke::CurvesGeometry &curves = drawing.strokes_for_write();

  /* Evaluated geometry. */
  bke::crazyspace::GeometryDeformation deformation =
      bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
          &ob_eval, ob_orig, layer_index, frame_number);

  /* Compute screen space positions. */
  const float4x4 transform = layer.to_world_space(ob_eval);
  Array<float2> screen_space_positions(curves.points_num());
  threading::parallel_for(curves.points_range(), 4096, [&](const IndexRange points) {
    for (const int point_i : points) {
      ED_view3d_project_float_global(
          &region,
          math::transform_point(transform, deformation.positions[point_i]),
          screen_space_positions[point_i],
          V3D_PROJ_TEST_NOP);
    }
  });

  return op.on_stroke_extended_drawing_view(C, drawing, screen_space_positions, extension_sample);
}

void GreasePencilStrokeOperationCommon::on_stroke_extended(const bContext &C,
                                                           const InputSample &extension_sample)
{
  using namespace blender::bke::greasepencil;

  /* Important: accessing region in worker threads will return null,
   * this has to be done on the main thread. */
  const ARegion &region = *CTX_wm_region(&C);
  const Scene &scene = *CTX_data_scene(&C);
  const Depsgraph &depsgraph = *CTX_data_depsgraph_pointer(&C);
  Object &ob_orig = *CTX_data_active_object(&C);
  const Object &ob_eval = *DEG_get_evaluated_object(&depsgraph, &ob_orig);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob_orig.data);

  Paint &paint = *BKE_paint_get_active_from_context(&C);
  const Brush &brush = *BKE_paint_brush(&paint);
  const bool active_layer_only = ((brush.gpencil_settings->flag & GP_BRUSH_ACTIVE_LAYER_ONLY) !=
                                  0);

  std::atomic<bool> changed = false;
  if (active_layer_only) {
    /* Apply only to the drawing at the current frame of the active layer. */
    if (!grease_pencil.has_active_layer()) {
      return;
    }
    const Layer &active_layer = *grease_pencil.get_active_layer();
    Drawing *drawing = grease_pencil.get_editable_drawing_at(active_layer, scene.r.cfra);

    if (drawing == nullptr) {
      return;
    }

    const int layer_index = *grease_pencil.get_layer_index(active_layer);
    const int frame_number = scene.r.cfra;
    changed = apply_to_drawing(*this,
                               C,
                               region,
                               ob_orig,
                               ob_eval,
                               active_layer,
                               layer_index,
                               frame_number,
                               *drawing,
                               extension_sample);
  }
  else {
    /* Apply to all editable drawings. */
    const Vector<ed::greasepencil::MutableDrawingInfo> drawings =
        ed::greasepencil::retrieve_editable_drawings(scene, grease_pencil);
    threading::parallel_for_each(drawings, [&](const ed::greasepencil::MutableDrawingInfo &info) {
      const Layer &layer = *grease_pencil.layers()[info.layer_index];
      if (apply_to_drawing(*this,
                           C,
                           region,
                           ob_orig,
                           ob_eval,
                           layer,
                           info.layer_index,
                           info.frame_number,
                           info.drawing,
                           extension_sample))
      {
        changed = true;
      }
    });
  }

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(&C, NC_GEOM | ND_DATA, &grease_pencil);
  }
}

void GreasePencilStrokeOperationCommon::on_stroke_done(const bContext & /*C*/) {}

}  // namespace blender::ed::sculpt_paint::greasepencil
