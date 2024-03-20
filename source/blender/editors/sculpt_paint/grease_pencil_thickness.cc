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

static bool get_brush_invert(const Brush &brush, const bool user_invert)
{
  /* The basic setting is the brush's setting. */
  bool invert = ((brush.gpencil_settings->sculpt_flag & GP_SCULPT_FLAG_INVERT) != 0) ||
                (brush.gpencil_settings->sculpt_flag & BRUSH_DIR_IN);
  /* During runtime, the user can hold down the Ctrl key to invert the basic behavior. */
  if (user_invert) {
    invert ^= true;
  }
  /* Set temporary status */
  SET_FLAG_FROM_TEST(brush.gpencil_settings->sculpt_flag, invert, GP_SCULPT_FLAG_TMP_INVERT);
  return invert;
}

class ThicknessOperation : public GreasePencilStrokeOperationCommon {
 public:
  BrushStrokeMode stroke_mode = BRUSH_STROKE_NORMAL;

  ThicknessOperation(const BrushStrokeMode stroke_mode) : stroke_mode(stroke_mode) {}

  bool on_stroke_extended_drawing(const bContext &C,
                                  bke::greasepencil::Drawing &drawing,
                                  Span<float2> view_positions,
                                  const InputSample &extension_sample) override;
};

bool ThicknessOperation::on_stroke_extended_drawing(const bContext &C,
                                                    bke::greasepencil::Drawing &drawing,
                                                    const Span<float2> view_positions,
                                                    const InputSample &extension_sample)
{
  Paint &paint = *BKE_paint_get_active_from_context(&C);
  const Brush &brush = *BKE_paint_brush(&paint);
  const bool user_invert = (this->stroke_mode == BrushStrokeMode::BRUSH_STROKE_INVERT);
  const bool invert = get_brush_invert(brush, user_invert);

  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  BLI_assert(view_positions.size() == curves.points_num());
  MutableSpan<float> radii = drawing.radii_for_write();

  threading::parallel_for(curves.points_range(), 4096, [&](const IndexRange range) {
    for (const int point_i : range) {
      const int2 &co = int2(view_positions[point_i]);
      float &radius = radii[point_i];
      const float influence = brush_influence(*CTX_data_scene(&C), brush, co, extension_sample);
      /* Factor 1/1000 is used to map arbitrary influence value to a sensible radius. */
      const float delta_radius = (invert ? -influence : influence) * 0.001f;
      radius = std::max(radius + delta_radius, 0.0f);
    }
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
