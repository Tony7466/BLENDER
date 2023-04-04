/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::gpencil {

class PaintOperation : public GreasePencilStrokeOperation {

 public:
  ~PaintOperation() override {}

  void on_stroke_extended(const bContext &C, const StrokeExtension &stroke_extension) override;
  void on_stroke_done(const bContext &C) override;
};

/**
 * Utility class that actually executes the update when the stroke is updated. That's useful
 * because it avoids passing a very large number of parameters between functions.
 */
struct PaintOperationExecutor {

  PaintOperationExecutor(const bContext &C) {}

  void execute(PaintOperation &self, const bContext &C, const StrokeExtension &stroke_extension)
  {
    std::cout << "pos: (" << stroke_extension.mouse_position.x << ", "
              << stroke_extension.mouse_position.x << "), pressure: " << stroke_extension.pressure
              << std::endl;
  }
};

void PaintOperation::on_stroke_extended(const bContext &C, const StrokeExtension &stroke_extension)
{
  PaintOperationExecutor executor{C};
  executor.execute(*this, C, stroke_extension);
}

void PaintOperation::on_stroke_done(const bContext &C)
{
  std::cout << "Done!" << std::endl;
}

std::unique_ptr<GreasePencilStrokeOperation> new_paint_operation()
{
  return std::make_unique<PaintOperation>();
}

}  // namespace blender::ed::sculpt_paint::gpencil