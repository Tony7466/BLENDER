/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <atomic>

#include "BLI_array.hh"
#include "BLI_task.hh"
#include "BLI_utildefines.h"

#include "BKE_brush.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_paint.hh"

#include "DNA_brush_enums.h"

#include "DEG_depsgraph_query.hh"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

struct Params {
  Params(const bContext &C,
         const float pressure,
         const float multi_frame_falloff,
         const bool invert)
  {
    this->context = &C;
    Scene *scene = CTX_data_scene(&C);
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
    this->ob_orig = CTX_data_active_object(&C);
    this->ob_eval = DEG_get_evaluated_object(depsgraph, this->ob_orig);

    Paint *paint = &scene->toolsettings->gp_sculptpaint->paint;
    this->brush = BKE_paint_brush(paint);

    this->pressure = pressure;
    this->multi_frame_falloff = multi_frame_falloff;
    this->invert = invert;
  }

  const bContext *context;
  Object *ob_eval;
  Object *ob_orig;
  const Brush *brush;

  float pressure;
  float multi_frame_falloff;
  bool invert;
};

static bool get_brush_invert(const Params &params)
{
  /* The basic setting is the brush's setting. */
  bool invert = ((params.brush->gpencil_settings->sculpt_flag & GP_SCULPT_FLAG_INVERT) != 0) ||
                (params.brush->gpencil_settings->sculpt_flag & BRUSH_DIR_IN);
  /* During runtime, the user can hold down the Ctrl key to invert the basic behavior. */
  if (params.invert) {
    invert ^= true;
  }
  /* Set temporary status */
  SET_FLAG_FROM_TEST(
      params.brush->gpencil_settings->sculpt_flag, invert, GP_SCULPT_FLAG_TMP_INVERT);
  return invert;
}

static float radius_from_input_sample(const Params &params, const InputSample &sample)
{
  const Scene *scene = CTX_data_scene(params.context);
  float radius = BKE_brush_size_get(scene, params.brush);
  if (BKE_brush_use_size_pressure(params.brush)) {
    radius *= BKE_curvemapping_evaluateF(
        params.brush->gpencil_settings->curve_sensitivity, 0, sample.pressure);
  }
  return radius;
}

static float brush_influence(const Params &params, const int2 &co, const InputSample &sample)
{
  const bool use_pressure = (params.brush->gpencil_settings->flag & GP_BRUSH_USE_PRESSURE);
  const float brush_radius = radius_from_input_sample(params, sample);
  const float radius = (use_pressure ? brush_radius * params.pressure : brush_radius);

  /* Basic strength factor from brush settings. */
  const float influence_base = (use_pressure ? params.brush->alpha * params.pressure :
                                               params.brush->alpha);

  /* Distance falloff. */
  int2 mval_i;
  round_v2i_v2fl(mval_i, sample.mouse_position);
  const float distance = float(len_v2v2_int(mval_i, co));

  /* Apply Brush curve. */
  const float brush_falloff = BKE_brush_curve_strength(params.brush, distance, radius);

  /* Arbitrary scaling factor to turn brush strength into radius offset. */
  return influence_base * brush_falloff * params.multi_frame_falloff * 0.001f;
}

static bool apply_thickness(const Params &params,
                            bke::CurvesGeometry &curves,
                            const Span<float2> screen_space_positions,
                            MutableSpan<float> radii,
                            const InputSample &sample)
{
  BLI_assert(screen_space_positions.size() == curves.points_num());

  threading::parallel_for(curves.points_range(), 4096, [&](const IndexRange range) {
    for (const int point_i : range) {
      const int2 &co = int2(screen_space_positions[point_i]);
      float &radius = radii[point_i];

      const float influence = brush_influence(params, co, sample);
      const bool invert = get_brush_invert(params);

      const float new_radius = (invert ? radius - influence : radius + influence);

      radius = std::max(new_radius, 0.0f);
    }
  });
  return true;
}

class ThicknessOperation : public GreasePencilStrokeOperation {
 public:
  bool active_layer_only = false;
  BrushStrokeMode stroke_mode = BRUSH_STROKE_NORMAL;
  float strength = 1.0f;

  ThicknessOperation(const BrushStrokeMode stroke_mode, const float strength)
      : stroke_mode(stroke_mode), strength(strength)
  {
  }
  ~ThicknessOperation() override {}

  bool apply(const Params &params, const InputSample &sample) const;
  bool apply_on_drawing(const Params &params,
                        const bke::greasepencil::Layer &layer,
                        const int layer_index,
                        const int frame_number,
                        bke::greasepencil::Drawing &drawing,
                        const InputSample &sample) const;

  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;
};

bool ThicknessOperation::apply(const Params &params, const InputSample &sample) const
{
  using namespace blender::bke::greasepencil;

  const Scene *scene = CTX_data_scene(params.context);

  /* Get the grease pencil drawing. */
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(params.ob_orig->data);

  std::atomic<bool> changed = false;
  if (this->active_layer_only) {
    /* Erase only on the drawing at the current frame of the active layer. */
    if (!grease_pencil.has_active_layer()) {
      return false;
    }
    const Layer &active_layer = *grease_pencil.get_active_layer();
    Drawing *drawing = grease_pencil.get_editable_drawing_at(active_layer, scene->r.cfra);

    if (drawing == nullptr) {
      return false;
    }

    const int layer_index = *grease_pencil.get_layer_index(active_layer);
    changed = apply_on_drawing(params, active_layer, layer_index, scene->r.cfra, *drawing, sample);
  }
  else {
    /* Erase on all editable drawings. */
    const Vector<ed::greasepencil::MutableDrawingInfo> drawings =
        ed::greasepencil::retrieve_editable_drawings(*scene, grease_pencil);
    threading::parallel_for_each(drawings, [&](const ed::greasepencil::MutableDrawingInfo &info) {
      const Layer &layer = *grease_pencil.layers()[info.layer_index];
      if (apply_on_drawing(
              params, layer, info.layer_index, info.frame_number, info.drawing, sample))
      {
        changed = true;
      }
    });
  }

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(params.context, NC_GEOM | ND_DATA, &grease_pencil);
  }
  return changed;
}

bool ThicknessOperation::apply_on_drawing(const Params &params,
                                          const bke::greasepencil::Layer &layer,
                                          const int layer_index,
                                          const int frame_number,
                                          bke::greasepencil::Drawing &drawing,
                                          const InputSample &sample) const
{
  const ARegion *region = CTX_wm_region(params.context);
  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  MutableSpan<float> radii = drawing.radii_for_write();

  bool changed = false;

  /* Evaluated geometry. */
  bke::crazyspace::GeometryDeformation deformation =
      bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
          params.ob_eval, *params.ob_orig, layer_index, frame_number);

  /* Compute screen space positions. */
  const float4x4 transform = layer.to_world_space(*params.ob_eval);
  Array<float2> screen_space_positions(curves.points_num());
  threading::parallel_for(curves.points_range(), 4096, [&](const IndexRange points) {
    for (const int point_i : points) {
      ED_view3d_project_float_global(
          region,
          math::transform_point(transform, deformation.positions[point_i]),
          screen_space_positions[point_i],
          V3D_PROJ_TEST_NOP);
    }
  });

  changed = apply_thickness(params, curves, screen_space_positions, radii, sample);
  if (changed) {
    curves.tag_radii_changed();
  }

  return changed;
}

void ThicknessOperation::on_stroke_begin(const bContext &C, const InputSample & /*start_sample*/)
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

  this->active_layer_only = ((brush->gpencil_settings->flag & GP_BRUSH_ACTIVE_LAYER_ONLY) != 0);
}

void ThicknessOperation::on_stroke_extended(const bContext &C, const InputSample &extension_sample)
{
  // TODO
  const float multi_frame_falloff = 1.0f;
  const bool invert = (this->stroke_mode == BrushStrokeMode::BRUSH_STROKE_INVERT);

  const Params params(C, this->strength, multi_frame_falloff, invert);
  apply(params, extension_sample);
}

void ThicknessOperation::on_stroke_done(const bContext & /*C*/) {}

std::unique_ptr<GreasePencilStrokeOperation> new_thickness_operation(
    const BrushStrokeMode stroke_mode, const float strength)
{
  return std::make_unique<ThicknessOperation>(stroke_mode, strength);
}

}  // namespace blender::ed::sculpt_paint::greasepencil
