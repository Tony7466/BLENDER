/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

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

class PushOperation : public GreasePencilStrokeOperationCommon {
 public:
  using GreasePencilStrokeOperationCommon::GreasePencilStrokeOperationCommon;

  bool on_stroke_extended_drawing(const GreasePencilStrokeParams &params,
                                  const IndexMask &point_selection,
                                  Span<float2> view_positions,
                                  const InputSample &extension_sample) override;
};

bool PushOperation::on_stroke_extended_drawing(const GreasePencilStrokeParams &params,
                                               const IndexMask &point_selection,
                                               Span<float2> view_positions,
                                               const InputSample &extension_sample)
{
  const Scene &scene = *CTX_data_scene(&params.context);
  Paint &paint = *BKE_paint_get_active_from_context(&params.context);
  const Brush &brush = *BKE_paint_brush(&paint);

  bke::CurvesGeometry &curves = params.drawing.strokes_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();

  const float2 mouse_delta = this->mouse_delta(extension_sample);

  point_selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
    const float2 &co = view_positions[point_i];
    const float influence = brush_influence(scene, brush, co, extension_sample);
    if (influence <= 0.0f) {
      return;
    }

    positions[point_i] = params.placement.project(co + mouse_delta * influence);
  });

  params.drawing.tag_positions_changed();
  return true;
}

std::unique_ptr<GreasePencilStrokeOperation> new_push_operation(const BrushStrokeMode stroke_mode)
{
  return std::make_unique<PushOperation>(stroke_mode);
}

}  // namespace blender::ed::sculpt_paint::greasepencil
