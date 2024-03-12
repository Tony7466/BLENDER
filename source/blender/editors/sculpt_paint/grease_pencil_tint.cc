/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_brush.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"

#include "BLI_length_parameterize.hh"
#include "BLI_math_color.h"
#include "BLI_math_geom.h"

#include "DEG_depsgraph_query.hh"

#include "ED_curves.hh"
#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "GEO_smooth_curves.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

static constexpr float POINT_OVERRIDE_THRESHOLD_PX = 3.0f;
static constexpr float POINT_RESAMPLE_MIN_DISTANCE_PX = 10.0f;

class TintOperation : public GreasePencilStrokeOperation {
 public:
  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;

  float radius;
  float strength;
  bool active_layer_only;
  ColorGeometry4f color;
};

void TintOperation::on_stroke_begin(const bContext &C, const InputSample &start_sample)
{
  Scene *scene = CTX_data_scene(&C);
  Paint *paint = BKE_paint_get_active_from_context(&C);
  Brush *brush = BKE_paint_brush(paint);

  if (brush->gpencil_settings == nullptr) {
    BKE_brush_init_gpencil_settings(brush);
  }
  BLI_assert(brush->gpencil_settings != nullptr);

  BKE_curvemapping_init(brush->gpencil_settings->curve_strength);

  this->radius = BKE_brush_size_get(scene, brush);
  this->strength = BKE_brush_alpha_get(scene, brush);
  this->active_layer_only = ((brush->gpencil_settings->flag & GP_BRUSH_ACTIVE_LAYER_ONLY) != 0);

  float4 color_linear;
  color_linear[3] = 1.0f;
  srgb_to_linearrgb_v3_v3(color_linear, brush->rgb);

  this->color = ColorGeometry4f(color_linear);
}

/* From BKE_gpencil_legacy.h.*/

#define TINT_VERTEX_COLOR_STROKE(brush) \
  (((brush)->gpencil_settings->vertex_mode == GPPAINT_MODE_STROKE) || \
   ((brush)->gpencil_settings->vertex_mode == GPPAINT_MODE_BOTH))
#define TINT_VERTEX_COLOR_FILL(brush) \
  (((brush)->gpencil_settings->vertex_mode == GPPAINT_MODE_FILL) || \
   ((brush)->gpencil_settings->vertex_mode == GPPAINT_MODE_BOTH))

void tint_perform_dab(TintOperation &self, const bContext &C, const InputSample &extension_sample)
{
  using namespace blender::bke::greasepencil;
  Scene *scene = CTX_data_scene(&C);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
  ARegion *region = CTX_wm_region(&C);
  Object *obact = CTX_data_active_object(&C);
  Object *ob_eval = DEG_get_evaluated_object(depsgraph, obact);

  Paint *paint = &scene->toolsettings->gp_paint->paint;
  Brush *brush = BKE_paint_brush(paint);

  /* Get the tool's data. */
  float2 mouse_position = extension_sample.mouse_position;
  float radius = self.radius;
  float strength = self.strength;
  if (BKE_brush_use_size_pressure(brush)) {
    radius *= extension_sample.pressure;
  }
  if (BKE_brush_use_alpha_pressure(brush)) {
    strength *= BKE_curvemapping_evaluateF(
        brush->gpencil_settings->curve_strength, 0, extension_sample.pressure);
  }
  // strength /= 100.0f; /* Attenuate factor to get a smoother tinting. */
  float fill_strength = strength;  // / 10.0f;

  strength = math::clamp(strength, 0.0f, 1.0f);
  fill_strength = math::clamp(fill_strength, 0.0f, 1.0f);

  /* Get the grease pencil drawing. */
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(obact->data);

  const auto execute_tint_on_drawing =
      [&](const int layer_index, const int frame_number, Drawing &drawing) {
        const Layer &layer = *grease_pencil.layers()[layer_index];
        bke::CurvesGeometry strokes = drawing.strokes_for_write();

        /* Evaluated geometry. */
        bke::crazyspace::GeometryDeformation deformation =
            bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
                ob_eval, *obact, layer_index, frame_number);

        /* Compute screen space positions. */
        Array<float2> screen_space_positions(strokes.points_num());
        threading::parallel_for(strokes.points_range(), 4096, [&](const IndexRange range) {
          for (const int point : range) {
            ED_view3d_project_float_global(region,
                                           math::transform_point(layer.to_world_space(*ob_eval),
                                                                 deformation.positions[point]),
                                           screen_space_positions[point],
                                           V3D_PROJ_TEST_NOP);
          }
        });

        MutableSpan<ColorGeometry4f> vertex_colors = drawing.vertex_colors_for_write();
        bke::MutableAttributeAccessor stroke_attributes = strokes.attributes_for_write();
        bke::SpanAttributeWriter<ColorGeometry4f> fill_colors =
            stroke_attributes.lookup_or_add_for_write_span<ColorGeometry4f>(
                "fill_color", bke::AttrDomain::Curve);
        OffsetIndices<int> points_by_curve = strokes.points_by_curve();

        bool changed = false;
        changed = true;
        threading::parallel_for(strokes.curves_range(), 128, [&](const IndexRange range) {
          for (const int curve : range) {
            bool stroke_changed = false;
            for (const int curve_point : points_by_curve[curve].index_range()) {
              const int point = curve_point + points_by_curve[curve].first();
              const float distance = math::distance(screen_space_positions[point], mouse_position);
              const float influence = strength * BKE_brush_curve_strength(brush, distance, radius);
              if (influence <= 0.0f) {
                continue;
              }
              if (TINT_VERTEX_COLOR_STROKE(brush)) {
                vertex_colors[point].premultiply_alpha();
                float4 rgba = float4(
                    math::interpolate(float3(vertex_colors[point]), float3(self.color), influence),
                    vertex_colors[point][3]);
                rgba[3] = rgba[3] * (1.0f - influence) + influence;
                vertex_colors[point] = ColorGeometry4f(rgba);
                vertex_colors[point].unpremultiply_alpha();
                stroke_changed = true;
              }
            }
            if (!fill_colors.span.is_empty() && stroke_changed && TINT_VERTEX_COLOR_FILL(brush)) {
              fill_colors.span[curve].premultiply_alpha();
              float4 rgba = float4(math::interpolate(float3(fill_colors.span[curve]),
                                                     float3(self.color),
                                                     fill_strength),
                                   fill_colors.span[curve][3]);
              rgba[3] = rgba[3] * (1.0f - fill_strength) + fill_strength;
              fill_colors.span[curve] = ColorGeometry4f(rgba);
              fill_colors.span[curve].unpremultiply_alpha();
            }
          }
        });
        fill_colors.finish();

        if (changed) {
          DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
          WM_event_add_notifier(&C, NC_GEOM | ND_DATA, &grease_pencil);
        }
      };

  if (self.active_layer_only) {
    /* Tint only on the drawing at the current frame of the active layer. */
    const Layer &active_layer = *grease_pencil.get_active_layer();
    Drawing *drawing = grease_pencil.get_editable_drawing_at(active_layer, scene->r.cfra);

    if (drawing == nullptr) {
      return;
    }

    execute_tint_on_drawing(*grease_pencil.get_layer_index(active_layer), scene->r.cfra, *drawing);
  }
  else {
    /* Tint on all editable drawings. */
    const Vector<ed::greasepencil::MutableDrawingInfo> drawings =
        ed::greasepencil::retrieve_editable_drawings(*scene, grease_pencil);
    threading::parallel_for_each(drawings, [&](const ed::greasepencil::MutableDrawingInfo &info) {
      execute_tint_on_drawing(info.layer_index, info.frame_number, info.drawing);
    });
  }
}

void TintOperation::on_stroke_extended(const bContext &C, const InputSample &extension_sample)
{
  tint_perform_dab(*this, C, extension_sample);
}

void TintOperation::on_stroke_done(const bContext & /*C*/) {}

std::unique_ptr<GreasePencilStrokeOperation> new_tint_operation()
{
  return std::make_unique<TintOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
