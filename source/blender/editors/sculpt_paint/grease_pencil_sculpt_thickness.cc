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
  using GreasePencilStrokeOperationCommon::GreasePencilStrokeOperationCommon;

  bool on_stroke_extended_drawing(const GreasePencilStrokeParams &params,
                                  const InputSample &extension_sample) override;
};

bool ThicknessOperation::on_stroke_extended_drawing(const GreasePencilStrokeParams &params,
    const InputSample &extension_sample)
{
  Paint &paint = *BKE_paint_get_active_from_context(&params.context);
  const Brush &brush = *BKE_paint_brush(&paint);
  const bool invert = this->is_inverted(brush);

  IndexMaskMemory selection_memory;
  const IndexMask selection = point_selection_mask(params, selection_memory);
  if (selection.is_empty()) {
    return false;
  }

  Array<float2> view_positions = calculate_view_positions(params, selection);
  bke::CurvesGeometry &curves = params.drawing.strokes_for_write();
  BLI_assert(view_positions.size() == curves.points_num());
  MutableSpan<float> radii = params.drawing.radii_for_write();

  selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
    float &radius = radii[point_i];
    const float influence = brush_influence(
        *CTX_data_scene(&params.context), brush, view_positions[point_i], extension_sample);
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
