/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "ED_grease_pencil.hh"

#include "paint_intern.hh"

namespace blender::bke::greasepencil {
class Drawing;
class Layer;
}  // namespace blender::bke::greasepencil
namespace blender::bke::crazyspace {
struct GeometryDeformation;
}

namespace blender::ed::sculpt_paint {

struct InputSample {
  float2 mouse_position;
  float pressure;
};

class GreasePencilStrokeOperation {
 public:
  virtual ~GreasePencilStrokeOperation() = default;
  virtual void on_stroke_begin(const bContext &C, const InputSample &start_sample) = 0;
  virtual void on_stroke_extended(const bContext &C, const InputSample &extension_sample) = 0;
  virtual void on_stroke_done(const bContext &C) = 0;
};

namespace greasepencil {

float3 mouse_delta_in_world_space(const bContext &C,
                                  const bke::greasepencil::Layer &layer,
                                  const float2 &mouse_delta_win);

/* Get list of drawings the tool should be operating on. */
Vector<ed::greasepencil::MutableDrawingInfo> get_drawings_for_sculpt(const bContext &C);

/* Point index mask for a drawing based on selection tool settings. */
IndexMask selection_mask(const Scene &scene,
                         Object &object,
                         const bke::greasepencil::Drawing &drawing,
                         IndexMaskMemory &memory);

/* Make sure the brush has all necessary grease pencil settings. */
void init_brush(Brush &brush);

/* Index mask of all points within the brush radius. */
IndexMask brush_influence_mask(const Scene &scene,
                               const Brush &brush,
                               const float2 &mouse_position,
                               float pressure,
                               float multi_frame_falloff,
                               const IndexMask &selection,
                               Span<float2> view_positions,
                               Vector<float> &influences,
                               IndexMaskMemory &memory);

/* Influence value at point co for the brush. */
float brush_influence(const Scene &scene,
                      const Brush &brush,
                      const float2 &co,
                      const InputSample &sample,
                      float multi_frame_falloff = 1.0f);

/* True if influence of the brush should be inverted. */
bool is_brush_inverted(const Brush &brush, BrushStrokeMode stroke_mode);

/* Common parameters for stroke callbacks that can be passed to utility functions. */
struct GreasePencilStrokeParams {
  const bContext &context;
  const ARegion &region;
  Object &ob_eval;
  Object &ob_orig;
  const bke::greasepencil::Layer &layer;
  int layer_index;
  int frame_number;
  ed::greasepencil::DrawingPlacement placement;
  bke::greasepencil::Drawing &drawing;
};

/* Project points from layer space into 2D view space. */
Array<float2> calculate_view_positions(const GreasePencilStrokeParams &params,
                                       const IndexMask &selection);

/* Stroke operation base class that performs various common initializations. */
class GreasePencilStrokeOperationCommon : public GreasePencilStrokeOperation {
 public:
  using MutableDrawingInfo = blender::ed::greasepencil::MutableDrawingInfo;
  using DrawingPlacement = ed::greasepencil::DrawingPlacement;

  BrushStrokeMode stroke_mode;

  /* Previous mouse position for computing the direction. */
  float2 prev_mouse_position;

  GreasePencilStrokeOperationCommon(const BrushStrokeMode stroke_mode) : stroke_mode(stroke_mode)
  {
  }

  bool is_inverted(const Brush &brush) const
  {
    return is_brush_inverted(brush, this->stroke_mode);
  }

  float2 mouse_delta(const InputSample &input_sample) const;

  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;

  /* Extend the stroke in a drawing using projected 2D coordinates.
   * This is an optional function that has all common data set up in advance. */
  virtual bool on_stroke_extended_drawing(const GreasePencilStrokeParams& /*params*/,
                                          const IndexMask & /*point_selection*/,
                                          Span<float2> /*view_positions*/,
                                          const InputSample & /*extension_sample*/)
  {
    return false;
  }
};

std::unique_ptr<GreasePencilStrokeOperation> new_paint_operation();
std::unique_ptr<GreasePencilStrokeOperation> new_erase_operation();
std::unique_ptr<GreasePencilStrokeOperation> new_tint_operation();
std::unique_ptr<GreasePencilStrokeOperation> new_smooth_operation(BrushStrokeMode stroke_mode);
std::unique_ptr<GreasePencilStrokeOperation> new_thickness_operation(BrushStrokeMode stroke_mode);
std::unique_ptr<GreasePencilStrokeOperation> new_strength_operation(BrushStrokeMode stroke_mode);
std::unique_ptr<GreasePencilStrokeOperation> new_randomize_operation(BrushStrokeMode stroke_mode);
std::unique_ptr<GreasePencilStrokeOperation> new_grab_operation(BrushStrokeMode stroke_mode);
std::unique_ptr<GreasePencilStrokeOperation> new_push_operation(BrushStrokeMode stroke_mode);
std::unique_ptr<GreasePencilStrokeOperation> new_pinch_operation(BrushStrokeMode stroke_mode);
std::unique_ptr<GreasePencilStrokeOperation> new_twist_operation(BrushStrokeMode stroke_mode);
std::unique_ptr<GreasePencilStrokeOperation> new_clone_operation(BrushStrokeMode stroke_mode);

}  // namespace greasepencil

}  // namespace blender::ed::sculpt_paint
