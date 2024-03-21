/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

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

class ThicknessOperation : public GreasePencilStrokeOperationCommon {
 public:
  BrushStrokeMode stroke_mode = BRUSH_STROKE_NORMAL;

  ThicknessOperation(const BrushStrokeMode stroke_mode) : stroke_mode(stroke_mode) {}

  bool on_stroke_extended_drawing(const bContext &C,
                                  bke::greasepencil::Drawing &drawing,
                                  int frame_number,
                                  const ed::greasepencil::DrawingPlacement &placement,
                                  const IndexMask &point_selection,
                                  Span<float2> view_positions,
                                  const InputSample &extension_sample) override;
};

bool ThicknessOperation::on_stroke_extended_drawing(
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
  const bool invert = is_brush_inverted(brush, this->stroke_mode);

  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  BLI_assert(view_positions.size() == curves.points_num());
  MutableSpan<float> radii = drawing.radii_for_write();

  point_selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
    float &radius = radii[point_i];
    const float influence = brush_influence(
        *CTX_data_scene(&C), brush, view_positions[point_i], extension_sample);
    /* Factor 1/1000 is used to map arbitrary influence value to a sensible radius. */
    const float delta_radius = (invert ? -influence : influence) * 0.001f;
    radius = std::max(radius + delta_radius, 0.0f);
  });

  curves.tag_radii_changed();
  return true;
}

std::unique_ptr<GreasePencilStrokeOperation> new_thickness_operation(
    const BrushStrokeMode stroke_mode)
{
  return std::make_unique<ThicknessOperation>(stroke_mode);
}

}  // namespace blender::ed::sculpt_paint::greasepencil
