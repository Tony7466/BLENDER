/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

class VertexBlurOperation : public GreasePencilStrokeOperation {
 public:
  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;
};

void VertexBlurOperation::on_stroke_begin(const bContext &C, const InputSample & /*start_sample*/)
{
  UNUSED_VARS(C);
}

void VertexBlurOperation::on_stroke_extended(const bContext &C,
                                             const InputSample &extension_sample)
{
  UNUSED_VARS(C, extension_sample);
}

void VertexBlurOperation::on_stroke_done(const bContext & /*C*/) {}

std::unique_ptr<GreasePencilStrokeOperation> new_vertex_blur_operation()
{
  return std::make_unique<VertexBlurOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
