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
  Params(const bContext &C, const float multi_frame_falloff, const bool invert)
  {
    this->context = &C;
    Scene *scene = CTX_data_scene(&C);
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
    this->ob_orig = CTX_data_active_object(&C);
    this->ob_eval = DEG_get_evaluated_object(depsgraph, this->ob_orig);

    Paint *paint = &scene->toolsettings->gp_sculptpaint->paint;
    this->brush = BKE_paint_brush(paint);

    this->multi_frame_falloff = multi_frame_falloff;
    this->invert = invert;
  }

  const bContext *context;
  /* Region can only be accessed from context on the main thread,
   * have to store the pointer for threaded point conversion. */
  const ARegion *region;
  Object *ob_eval;
  Object *ob_orig;
  const Brush *brush;

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

class ThicknessOperation : public GreasePencilStrokeOperation {
 public:
  bool active_layer_only = false;
  BrushStrokeMode stroke_mode = BRUSH_STROKE_NORMAL;

  ThicknessOperation(const BrushStrokeMode stroke_mode) : stroke_mode(stroke_mode) {}
  ~ThicknessOperation() override {}

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

bool ThicknessOperation::apply_to_drawing(const Params &params,
                                          const bke::greasepencil::Layer &layer,
                                          const int layer_index,
                                          const int frame_number,
                                          bke::greasepencil::Drawing &drawing,
                                          const InputSample &sample) const
{
  const ARegion *region = CTX_wm_region(params.context);
  bke::CurvesGeometry &curves = drawing.strokes_for_write();

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

  return apply_with_2d_positions(params, drawing, screen_space_positions, sample);
}

bool ThicknessOperation::apply_with_2d_positions(const Params &params,
                                                 bke::greasepencil::Drawing &drawing,
                                                 const Span<float2> screen_space_positions,
                                                 const InputSample &sample) const
{
  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  BLI_assert(screen_space_positions.size() == curves.points_num());
  MutableSpan<float> radii = drawing.radii_for_write();

  threading::parallel_for(curves.points_range(), 4096, [&](const IndexRange range) {
    for (const int point_i : range) {
      const int2 &co = int2(screen_space_positions[point_i]);
      float &radius = radii[point_i];
      const float influence = brush_influence(
          *CTX_data_scene(params.context), *params.brush, co, sample);
      const bool invert = get_brush_invert(params);

      const float new_radius = (invert ? radius - influence : radius + influence);

      radius = std::max(new_radius, 0.0f);
    }
  });
  curves.tag_radii_changed();
  return true;
}

void ThicknessOperation::on_stroke_begin(const bContext &C, const InputSample & /*start_sample*/)
{
  Paint *paint = BKE_paint_get_active_from_context(&C);
  Brush *brush = BKE_paint_brush(paint);
  init_brush(*brush);

  this->active_layer_only = ((brush->gpencil_settings->flag & GP_BRUSH_ACTIVE_LAYER_ONLY) != 0);
}

void ThicknessOperation::on_stroke_extended(const bContext &C, const InputSample &extension_sample)
{
  // TODO
  const float multi_frame_falloff = 1.0f;
  const bool invert = (this->stroke_mode == BrushStrokeMode::BRUSH_STROKE_INVERT);

  const Params params(C, multi_frame_falloff, invert);
  apply(params, extension_sample);
}

void ThicknessOperation::on_stroke_done(const bContext & /*C*/) {}

std::unique_ptr<GreasePencilStrokeOperation> new_thickness_operation(
    const BrushStrokeMode stroke_mode)
{
  return std::make_unique<ThicknessOperation>(stroke_mode);
}

}  // namespace blender::ed::sculpt_paint::greasepencil
