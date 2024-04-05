/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_paint.hh"

#include "BLI_virtual_array.hh"
#include "DNA_brush_enums.h"

#include "GEO_smooth_curves.hh"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

class SmoothOperation : public GreasePencilStrokeOperationCommon {
 public:
  using GreasePencilStrokeOperationCommon::GreasePencilStrokeOperationCommon;

  bool on_stroke_extended_drawing(const GreasePencilStrokeParams &params,
                                  const InputSample &extension_sample) override;
};

bool SmoothOperation::on_stroke_extended_drawing(const GreasePencilStrokeParams &params,
    const InputSample &extension_sample)
{
  Paint &paint = *BKE_paint_get_active_from_context(&params.context);
  const Brush &brush = *BKE_paint_brush(&paint);
  const int sculpt_mode_flag = brush.gpencil_settings->sculpt_mode_flag;

  IndexMaskMemory selection_memory;
  const IndexMask selection = point_selection_mask(params, selection_memory);
  if (selection.is_empty()) {
    return false;
  }

  Array<float2> view_positions = calculate_view_positions(params, selection);
  bke::CurvesGeometry &curves = params.drawing.strokes_for_write();
  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  const OffsetIndices points_by_curve = curves.points_by_curve();
  const VArray<bool> cyclic = curves.cyclic();
  const int iterations = 2;

  const VArray<float> influences = VArray<float>::ForFunc(
      view_positions.size(), [&](const int64_t point_) {
        return brush_influence(
            *CTX_data_scene(&params.context), brush, view_positions[point_], extension_sample);
      });
  Array<bool> selection_array(curves.points_num());
  selection.to_bools(selection_array);
  const VArray<bool> selection_varray = VArray<bool>::ForSpan(selection_array);

  bool changed = false;
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_POSITION) {
    MutableSpan<float3> positions = curves.positions_for_write();
    geometry::smooth_curve_attribute(curves.curves_range(),
                                     points_by_curve,
                                     selection_varray,
                                     cyclic,
                                     iterations,
                                     influences,
                                     false,
                                     false,
                                     positions);
    params.drawing.tag_positions_changed();
    changed = true;
  }
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_STRENGTH) {
    MutableSpan<float> opacities = params.drawing.opacities_for_write();
    geometry::smooth_curve_attribute(curves.curves_range(),
                                     points_by_curve,
                                     selection_varray,
                                     cyclic,
                                     iterations,
                                     influences,
                                     true,
                                     false,
                                     opacities);
    changed = true;
  }
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_THICKNESS) {
    const MutableSpan<float> radii = params.drawing.radii_for_write();
    geometry::smooth_curve_attribute(curves.curves_range(),
                                     points_by_curve,
                                     selection_varray,
                                     cyclic,
                                     iterations,
                                     influences,
                                     true,
                                     false,
                                     radii);
    curves.tag_radii_changed();
    changed = true;
  }
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_UV) {
    /* TODO stroke_u attribute not used yet. */
    bke::SpanAttributeWriter<float> rotations = attributes.lookup_or_add_for_write_span<float>(
        "rotation", bke::AttrDomain::Point);
    geometry::smooth_curve_attribute(curves.curves_range(),
                                     points_by_curve,
                                     selection_varray,
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

std::unique_ptr<GreasePencilStrokeOperation> new_smooth_operation(
    const BrushStrokeMode stroke_mode)
{
  return std::make_unique<SmoothOperation>(stroke_mode);
}

}  // namespace blender::ed::sculpt_paint::greasepencil
