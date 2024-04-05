/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_paint.hh"

#include "GEO_join_geometries.hh"

#include "ED_curves.hh"
#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"

#include <iostream>

namespace blender::ed::sculpt_paint::greasepencil {

class CloneOperation : public GreasePencilStrokeOperationCommon {
 public:
  using GreasePencilStrokeOperationCommon::GreasePencilStrokeOperationCommon;

  bool on_stroke_extended_drawing(const GreasePencilStrokeParams &params,
                                  const IndexMask &point_selection,
                                  Span<float2> view_positions,
                                  const InputSample &extension_sample) override;
};

bool CloneOperation::on_stroke_extended_drawing(const GreasePencilStrokeParams &params,
                                                const IndexMask & /*point_selection*/,
                                                Span<float2> /*view_positions*/,
                                                const InputSample &extension_sample)
{
  Main &bmain = *CTX_data_main(&params.context);
  Object &object = *CTX_data_active_object(&params.context);
  float2 ob_center;
  ED_view3d_project_float_object(&params.region, float3(0), ob_center, V3D_PROJ_TEST_NOP);
  const float2 &mouse_delta = extension_sample.mouse_position - ob_center;

  const IndexRange pasted_curves = ed::greasepencil::clipboard_paste_strokes(
      bmain, object, params.drawing, false);
  if (pasted_curves.is_empty()) {
    return false;
  }

  bke::CurvesGeometry &curves = params.drawing.strokes_for_write();
  const OffsetIndices<int> pasted_points_by_curve = curves.points_by_curve().slice(pasted_curves);
  const IndexRange pasted_points = IndexRange::from_begin_size(
      pasted_points_by_curve[0].start(),
      pasted_points_by_curve.total_size() - pasted_points_by_curve[0].start());

  Array<float2> view_positions = calculate_view_positions(params, pasted_points);
  MutableSpan<float3> positions = curves.positions_for_write();
  threading::parallel_for(pasted_points, 4096, [&](const IndexRange range) {
    for (const int point_i : range) {
      positions[point_i] = params.placement.project(view_positions[point_i] + mouse_delta);
    }
  });
  params.drawing.tag_positions_changed();

  return true;
}

std::unique_ptr<GreasePencilStrokeOperation> new_clone_operation(const BrushStrokeMode stroke_mode)
{
  return std::make_unique<CloneOperation>(stroke_mode);
}

}  // namespace blender::ed::sculpt_paint::greasepencil
