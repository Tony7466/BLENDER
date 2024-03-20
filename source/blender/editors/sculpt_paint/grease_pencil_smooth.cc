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

namespace blender::ed::sculpt_paint::greasepencil {

class SmoothOperation : public GreasePencilStrokeOperationCommon {
 public:
  bool on_stroke_extended_drawing_view(const bContext &C,
                                       bke::greasepencil::Drawing &drawing,
                                       Span<float2> view_positions,
                                       const InputSample &extension_sample) override;
};

bool SmoothOperation::on_stroke_extended_drawing_view(const bContext &C,
                                                      bke::greasepencil::Drawing &drawing,
                                                      Span<float2> view_positions,
                                                      const InputSample &extension_sample)
{
  Paint &paint = *BKE_paint_get_active_from_context(&C);
  const Brush &brush = *BKE_paint_brush(&paint);
  const int sculpt_mode_flag = brush.gpencil_settings->sculpt_mode_flag;

  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  const OffsetIndices points_by_curve = curves.points_by_curve();
  const VArray<bool> point_selection = VArray<bool>::ForSingle(true, curves.points_num());
  const VArray<bool> cyclic = curves.cyclic();
  const int iterations = 2;

  const VArray<float> influences = VArray<float>::ForFunc(
      view_positions.size(), [&](const int64_t index) {
        return greasepencil::brush_influence(
            *CTX_data_scene(&C), brush, int2(view_positions[index]), extension_sample);
      });

  bool changed = false;
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_POSITION) {
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
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_STRENGTH) {
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
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_THICKNESS) {
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
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_UV) {
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

std::unique_ptr<GreasePencilStrokeOperation> new_smooth_operation()
{
  return std::make_unique<SmoothOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
