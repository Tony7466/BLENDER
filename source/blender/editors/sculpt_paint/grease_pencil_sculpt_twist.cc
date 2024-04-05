/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_paint.hh"

#include "BLI_math_rotation.hh"
#include "BLI_math_rotation_legacy.hh"
#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

class TwistOperation : public GreasePencilStrokeOperationCommon {
 public:
  using GreasePencilStrokeOperationCommon::GreasePencilStrokeOperationCommon;

  bool on_stroke_extended_drawing(const GreasePencilStrokeParams &params,
                                  const InputSample &extension_sample) override;
};

static float2 rotate_by_angle(const float2 &vec, const float angle)
{
  const float cos_angle = math::cos(angle);
  const float sin_angle = math::sin(angle);
  return float2(vec.x * cos_angle - vec.y * sin_angle, vec.x * sin_angle + vec.y * cos_angle);
}

bool TwistOperation::on_stroke_extended_drawing(const GreasePencilStrokeParams &params,
                                                const InputSample &extension_sample)
{
  const Scene &scene = *CTX_data_scene(&params.context);
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
  MutableSpan<float3> positions = curves.positions_for_write();

  const float2 mouse_pos = extension_sample.mouse_position;

  selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
    const float2 &co = view_positions[point_i];
    const float influence = brush_influence(scene, brush, co, extension_sample);
    if (influence <= 0.0f) {
      return;
    }

    const float angle = DEG2RADF(invert ? -1.0f : 1.0f) * influence;
    positions[point_i] = params.placement.project(rotate_by_angle(co - mouse_pos, angle) +
                                                  mouse_pos);
  });

  params.drawing.tag_positions_changed();
  return true;
}

std::unique_ptr<GreasePencilStrokeOperation> new_twist_operation(const BrushStrokeMode stroke_mode)
{
  return std::make_unique<TwistOperation>(stroke_mode);
}

}  // namespace blender::ed::sculpt_paint::greasepencil
