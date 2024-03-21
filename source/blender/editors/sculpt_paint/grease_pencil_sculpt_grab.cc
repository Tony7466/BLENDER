/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_paint.hh"

#include "BLI_index_mask.hh"
#include "BLI_math_matrix.hh"
#include "BLI_task.hh"
#include "BLI_utildefines.h"
#include "BLI_virtual_array.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_grease_pencil_types.h"
#include "DNA_view3d_types.h"
#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"

#include <iostream>

namespace blender::ed::sculpt_paint::greasepencil {

class GrabOperation : public GreasePencilStrokeOperation {
 public:
  using MutableDrawingInfo = blender::ed::greasepencil::MutableDrawingInfo;
  using DrawingPlacement = ed::greasepencil::DrawingPlacement;

  float2 initial_mouse_position;

  /* Cached point mask and influence for a particular drawing. */
  struct PointWeights {
    int layer_index;
    int frame_number;
    float multi_frame_falloff;

    /* Layer space to view space projection at the start of the stroke. */
    float4x4 layer_to_view;
    /* Points that are grabbed at the beginning of the stroke. */
    IndexMask point_mask;
    /* Influence weights for grabbed points. */
    Vector<float> weights;
    /* Rotation of the original point. */
    Vector<float> rotations;

    IndexMaskMemory memory;
  };
  /* Cached point data for each affected drawing. */
  Array<PointWeights> drawing_data;

  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;
};

void GrabOperation::on_stroke_begin(const bContext &C, const InputSample &start_sample)
{
  const ARegion &region = *CTX_wm_region(&C);
  const RegionView3D &rv3d = *CTX_wm_region_view3d(&C);
  const Scene &scene = *CTX_data_scene(&C);
  Paint &paint = *BKE_paint_get_active_from_context(&C);
  Brush &brush = *BKE_paint_brush(&paint);
  const Depsgraph &depsgraph = *CTX_data_depsgraph_pointer(&C);
  Object &ob_orig = *CTX_data_active_object(&C);
  Object &ob_eval = *DEG_get_evaluated_object(&depsgraph, &ob_orig);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob_orig.data);

  init_brush(brush);

  this->initial_mouse_position = start_sample.mouse_position;

  const Vector<MutableDrawingInfo> drawings = get_drawings_for_sculpt(C);
  this->drawing_data.reinitialize(drawings.size());
  threading::parallel_for_each(drawings.index_range(), [&](const int i) {
    const MutableDrawingInfo &info = drawings[i];
    BLI_assert(info.layer_index >= 0);
    PointWeights &data = this->drawing_data[i];

    const bke::greasepencil::Layer &layer = *grease_pencil.layers()[info.layer_index];
    const bke::CurvesGeometry &curves = info.drawing.strokes();
    const int drawing_index = layer.drawing_index_at(info.frame_number);
    BLI_assert(drawing_index >= 0);
    BLI_assert(grease_pencil.get_drawing_at(layer, info.frame_number) == &info.drawing);

    IndexMaskMemory selection_memory;
    IndexMask selection = selection_mask(scene, ob_eval, info.drawing, selection_memory);

    Array<float2> view_positions(curves.points_num());
    calculate_view_positions(region,
                             ob_eval,
                             ob_orig,
                             layer,
                             info.layer_index,
                             info.frame_number,
                             selection,
                             view_positions);

    /* Cache points under brush influence. */
    Vector<float> weights;
    IndexMask point_mask = brush_influence_mask(scene,
                                                brush,
                                                start_sample.mouse_position,
                                                start_sample.pressure,
                                                info.multi_frame_falloff,
                                                selection,
                                                view_positions,
                                                weights,
                                                data.memory);
    /* TODO */
    Vector<float> rotations(point_mask.size(), 0.0f);

    if (point_mask.is_empty()) {
      /* Set empty point mask to skip. */
      data.point_mask = {};
      return;
    }

    data.layer_index = info.layer_index;
    data.frame_number = info.frame_number;
    data.multi_frame_falloff = info.multi_frame_falloff;
    data.layer_to_view = ED_view3d_ob_project_mat_get(&rv3d, &ob_eval) *
                         layer.to_object_space(ob_eval);
    data.point_mask = std::move(point_mask);
    std::cout << "Drawing " << drawing_index << ": " << std::endl;
    for (const int i : data.point_mask.index_range()) {
      std::cout << " " << i << " " << data.point_mask[i] << " w=" << weights[i] << std::endl;
    }
    std::flush(std::cout);
    data.weights = std::move(weights);
    data.rotations = std::move(rotations);
  });
}

void GrabOperation::on_stroke_extended(const bContext &C, const InputSample &extension_sample)
{
  const ARegion &region = *CTX_wm_region(&C);
  const RegionView3D &rv3d = *CTX_wm_region_view3d(&C);
  const View3D &view3d = *CTX_wm_view3d(&C);
  const Scene &scene = *CTX_data_scene(&C);
  const Depsgraph &depsgraph = *CTX_data_depsgraph_pointer(&C);
  Object &ob_orig = *CTX_data_active_object(&C);
  Object &ob_eval = *DEG_get_evaluated_object(&depsgraph, &ob_orig);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(ob_orig.data);

  /* Mouse translation in view space. */
  const float2 mouse_delta = extension_sample.mouse_position - this->initial_mouse_position;
  std::cout << "mouse delta = " << mouse_delta << std::endl;

  bool changed = false;

  threading::parallel_for_each(this->drawing_data.index_range(), [&](const int i) {
    const PointWeights &data = this->drawing_data[i];
    if (data.point_mask.is_empty()) {
      return;
    }
    const bke::greasepencil::Layer &layer = *grease_pencil.layers()[data.layer_index];
    /* If a new frame is created, could be impossible find the stroke. */
    const int drawing_index = layer.drawing_index_at(data.frame_number);
    if (drawing_index < 0) {
      return;
    }
    GreasePencilDrawingBase &drawing_base = *grease_pencil.drawing(drawing_index);
    if (drawing_base.type != GP_DRAWING) {
      return;
    }
    bke::greasepencil::Drawing &drawing =
        reinterpret_cast<GreasePencilDrawing &>(drawing_base).wrap();
    const ed::greasepencil::DrawingPlacement placement(scene, region, view3d, ob_eval, layer);
    bke::CurvesGeometry &curves = drawing.strokes_for_write();
    /* Current view space to layer space transform. */
    const float4x4 view_to_layer_current = math::invert(
        ED_view3d_ob_project_mat_get(&rv3d, &ob_eval) * layer.to_object_space(ob_eval));

    MutableSpan<float3> positions = curves.positions_for_write();
    std::cout << "Apply " << i << ": " << std::endl;
    data.point_mask.foreach_index(GrainSize(1024), [&](const int point_i, const int index) {
      /* Translate the point in view space with the influence factor. */
      const float3 pos_view_orig = math::transform_point(data.layer_to_view, positions[point_i]);
      const float3 pos_view = pos_view_orig + float3(mouse_delta, 0.0f) * data.weights[index];
      /* Project on drawing plane. */
      positions[point_i] = placement.project(
          math::transform_point(view_to_layer_current, pos_view).xy());
      std::cout << " " << point_i << " " << pos_view_orig << " -> " << pos_view << std::endl;
    });
    std::flush(std::cout);

    drawing.tag_positions_changed();
    changed = true;
  });

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(&C, NC_GEOM | ND_DATA, &grease_pencil);
  }
}

void GrabOperation::on_stroke_done(const bContext & /*C*/) {}

std::unique_ptr<GreasePencilStrokeOperation> new_grab_operation()
{
  return std::make_unique<GrabOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
