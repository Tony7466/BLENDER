/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <atomic>

#include "BKE_attribute.hh"
#include "BLI_array.hh"
#include "BLI_task.hh"

#include "BKE_brush.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_paint.hh"

#include "DNA_brush_enums.h"

#include "DEG_depsgraph_query.hh"

#include "GEO_smooth_curves.hh"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"

#include <iostream>

namespace blender::ed::sculpt_paint::greasepencil {

struct Params {
  Params(const bContext &C, const float multi_frame_falloff)
  {
    this->context = &C;
    this->region = CTX_wm_region(&C);
    Scene *scene = CTX_data_scene(&C);
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
    this->ob_orig = CTX_data_active_object(&C);
    this->ob_eval = DEG_get_evaluated_object(depsgraph, this->ob_orig);

    Paint *paint = &scene->toolsettings->gp_sculptpaint->paint;
    this->brush = BKE_paint_brush(paint);

    this->multi_frame_falloff = multi_frame_falloff;
  }

  const bContext *context;
  /* Region can only be accessed from context on the main thread,
   * have to store the pointer for threaded point conversion. */
  const ARegion *region;
  Object *ob_eval;
  Object *ob_orig;
  const Brush *brush;

  float multi_frame_falloff;
};

class SmoothOperation : public GreasePencilStrokeOperation {
 public:
  bool active_layer_only = false;

  SmoothOperation() {}
  ~SmoothOperation() override {}

  bool apply(const Params &params, const InputSample &sample) const;
  bool apply_to_drawing(const Params &params,
                        const bke::greasepencil::Layer &layer,
                        const int layer_index,
                        const int frame_number,
                        bke::greasepencil::Drawing &drawing,
                        const InputSample &sample) const;
  bool apply_with_2d_positions(const Params &params,
                               bke::greasepencil::Drawing &drawing,
                               const Span<float2> screen_space_positions,
                               const InputSample &sample) const;

  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;
};

bool SmoothOperation::apply(const Params &params, const InputSample &sample) const
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
    changed = apply_to_drawing(params, active_layer, layer_index, scene->r.cfra, *drawing, sample);
  }
  else {
    /* Erase on all editable drawings. */
    const Vector<ed::greasepencil::MutableDrawingInfo> drawings =
        ed::greasepencil::retrieve_editable_drawings(*scene, grease_pencil);
    threading::parallel_for_each(drawings, [&](const ed::greasepencil::MutableDrawingInfo &info) {
      const Layer &layer = *grease_pencil.layers()[info.layer_index];
      if (apply_to_drawing(
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

bool SmoothOperation::apply_to_drawing(const Params &params,
                                       const bke::greasepencil::Layer &layer,
                                       const int layer_index,
                                       const int frame_number,
                                       bke::greasepencil::Drawing &drawing,
                                       const InputSample &sample) const
{
  const bke::CurvesGeometry &curves = drawing.strokes_for_write();

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
          params.region,
          math::transform_point(transform, deformation.positions[point_i]),
          screen_space_positions[point_i],
          V3D_PROJ_TEST_NOP);
    }
  });

  return apply_with_2d_positions(params, drawing, screen_space_positions, sample);
}

bool SmoothOperation::apply_with_2d_positions(const Params &params,
                                              bke::greasepencil::Drawing &drawing,
                                              const Span<float2> screen_space_positions,
                                              const InputSample &sample) const
{
  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  const OffsetIndices points_by_curve = curves.points_by_curve();
  const VArray<bool> point_selection = VArray<bool>::ForSingle(true, curves.points_num());
  const VArray<bool> cyclic = curves.cyclic();
  const int iterations = 2;

  const VArray<float> influences = VArray<float>::ForFunc(
      screen_space_positions.size(), [&](const int64_t index) {
        return greasepencil::sculpt_brush_influence(*CTX_data_scene(params.context),
                                                    *params.brush,
                                                    int2(screen_space_positions[index]),
                                                    sample);
      });

  bool changed = false;
  if (params.brush->gpencil_settings->sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_POSITION) {
    MutableSpan<float3> positions = curves.positions_for_write();
    geometry::smooth_curve_attribute(curves.curves_range(),
                                     points_by_curve,
                                     point_selection,
                                     cyclic,
                                     iterations,
                                     influences,
                                     false,
                                     false,
                                     positions);
    drawing.tag_positions_changed();
    changed = true;
  }
  if (params.brush->gpencil_settings->sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_STRENGTH) {
    MutableSpan<float> opacities = drawing.opacities_for_write();
    geometry::smooth_curve_attribute(curves.curves_range(),
                                     points_by_curve,
                                     point_selection,
                                     cyclic,
                                     iterations,
                                     influences,
                                     true,
                                     false,
                                     opacities);
    changed = true;
  }
  if (params.brush->gpencil_settings->sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_THICKNESS) {
    const MutableSpan<float> radii = drawing.radii_for_write();
    geometry::smooth_curve_attribute(curves.curves_range(),
                                     points_by_curve,
                                     point_selection,
                                     cyclic,
                                     iterations,
                                     influences,
                                     true,
                                     false,
                                     radii);
    curves.tag_radii_changed();
    changed = true;
  }
  if (params.brush->gpencil_settings->sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_UV) {
    /* TODO stroke_u attribute not used yet. */
    bke::SpanAttributeWriter<float> rotations = attributes.lookup_or_add_for_write_span<float>(
        "rotation", bke::AttrDomain::Point);
    geometry::smooth_curve_attribute(curves.curves_range(),
                                     points_by_curve,
                                     point_selection,
                                     cyclic,
                                     iterations,
                                     influences,
                                     true,
                                     false,
                                     rotations.span);
    rotations.finish();
    changed = true;
  }
  return changed;
}

void SmoothOperation::on_stroke_begin(const bContext &C, const InputSample & /*start_sample*/)
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

void SmoothOperation::on_stroke_extended(const bContext &C, const InputSample &extension_sample)
{
  // TODO
  const float multi_frame_falloff = 1.0f;

  const Params params(C, multi_frame_falloff);
  this->apply(params, extension_sample);
}

void SmoothOperation::on_stroke_done(const bContext & /*C*/) {}

std::unique_ptr<GreasePencilStrokeOperation> new_smooth_operation()
{
  return std::make_unique<SmoothOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
