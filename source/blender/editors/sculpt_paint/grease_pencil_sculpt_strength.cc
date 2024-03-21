/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_task.hh"

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_paint.hh"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

class StrengthOperation : public GreasePencilStrokeOperationCommon {
 public:
  BrushStrokeMode stroke_mode = BRUSH_STROKE_NORMAL;

  StrengthOperation(const BrushStrokeMode stroke_mode) : stroke_mode(stroke_mode) {}

  bool on_stroke_extended_drawing(const bContext &C,
                                  bke::greasepencil::Drawing &drawing,
                                  int frame_number,
                                  const ed::greasepencil::DrawingPlacement &placement,
                                  const IndexMask &point_selection,
                                  Span<float2> view_positions,
                                  const InputSample &extension_sample) override;
};

bool StrengthOperation::on_stroke_extended_drawing(
    const bContext &C,
    bke::greasepencil::Drawing &drawing,
    int /*frame_number*/,
    const ed::greasepencil::DrawingPlacement & /*placement*/,
    const IndexMask &point_selection,
    const Span<float2> view_positions,
    const InputSample &extension_sample)
{
  Paint &paint = *BKE_paint_get_active_from_context(&C);
  const Brush &brush = *BKE_paint_brush(&paint);
  const bool invert = brush_inverted(brush, this->stroke_mode);

  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  BLI_assert(view_positions.size() == curves.points_num());
  MutableSpan<float> opacities = drawing.opacities_for_write();

  point_selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
    float &opacity = opacities[point_i];
    const float influence = brush_influence(
        *CTX_data_scene(&C), brush, view_positions[point_i], extension_sample);
    /* Brush influence mapped to opacity by a factor of 0.125. */
    const float delta_opacity = (invert ? -influence : influence) * 0.125f;
    opacity = math::clamp(opacity + delta_opacity, 0.0f, 1.0f);
  });

  return true;
}

std::unique_ptr<GreasePencilStrokeOperation> new_strength_operation(
    const BrushStrokeMode stroke_mode)
{
  return std::make_unique<StrengthOperation>(stroke_mode);
}

}  // namespace blender::ed::sculpt_paint::greasepencil
