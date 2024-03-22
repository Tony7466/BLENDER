/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_paint.hh"

#include "BLI_virtual_array.hh"
#include "DNA_brush_enums.h"

#include "DNA_view3d_types.h"
#include "GEO_smooth_curves.hh"

#include "DEG_depsgraph_query.hh"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

class PushOperation : public GreasePencilStrokeOperationCommon {
 public:
  BrushStrokeMode stroke_mode;

  PushOperation(const BrushStrokeMode stroke_mode) : stroke_mode(stroke_mode) {}

  bool on_stroke_extended_drawing(const bContext &C,
                                  const bke::greasepencil::Layer &layer,
                                  bke::greasepencil::Drawing &drawing,
                                  int frame_number,
                                  const ed::greasepencil::DrawingPlacement &placement,
                                  const IndexMask &point_selection,
                                  Span<float2> view_positions,
                                  const InputSample &extension_sample) override;
};

bool PushOperation::on_stroke_extended_drawing(const bContext &C,
                                               const bke::greasepencil::Layer &layer,
                                               bke::greasepencil::Drawing &drawing,
                                               int /*frame_number*/,
                                               const ed::greasepencil::DrawingPlacement &placement,
                                               const IndexMask &point_selection,
                                               Span<float2> view_positions,
                                               const InputSample &extension_sample)
{
  const ARegion &region = *CTX_wm_region(&C);
  const RegionView3D &rv3d = *CTX_wm_region_view3d(&C);
  const Scene &scene = *CTX_data_scene(&C);
  const Depsgraph &depsgraph = *CTX_data_depsgraph_pointer(&C);
  Object &ob_orig = *CTX_data_active_object(&C);
  Object &ob_eval = *DEG_get_evaluated_object(&depsgraph, &ob_orig);
  Paint &paint = *BKE_paint_get_active_from_context(&C);
  const Brush &brush = *BKE_paint_brush(&paint);

  /* Crazyspace deformation. */
  bke::crazyspace::GeometryDeformation deformation =
      bke::crazyspace::get_evaluated_grease_pencil_drawing_deformation(
          &ob_eval, ob_orig, data.layer_index, data.frame_number);

  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();

  /* Transform mouse delta into layer space. */
  //   const float3 mouse_delta = mouse_delta_in_world_space(
  //       C, layer, this->mouse_delta(extension_sample));
  const float2 mouse_delta = this->mouse_delta(extension_sample);

  point_selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
    const float2 &co = view_positions[point_i];
    const float influence = brush_influence(scene, brush, co, extension_sample);
    if (influence <= 0.0f) {
      return;
    }

    // positions[point_i] = placement.project(co + sideways * influence * noise);

    const float3 new_pos_layer = deformation.positions[point_i] +
                                 mouse_delta * data.weights[index];
    // const float3 new_pos_world = math::transform_point(layer.to_world_space(ob_eval),
    //                                                    new_pos_layer);
    // float2 new_pos_view;
    // ED_view3d_project_float_global(&region, new_pos_world, new_pos_view, V3D_PROJ_TEST_NOP);
    // positions[point_i] = placement.project(new_pos_view);
  });

  drawing.tag_positions_changed();
  return true;
}

std::unique_ptr<GreasePencilStrokeOperation> new_push_operation(const BrushStrokeMode stroke_mode)
{
  return std::make_unique<PushOperation>(stroke_mode);
}

}  // namespace blender::ed::sculpt_paint::greasepencil
