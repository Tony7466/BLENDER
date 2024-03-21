/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "grease_pencil_fill.hh"

namespace blender::ed::greasepencil::fill {

class FloodFillOperation : public FillOperation {
 public:
  /* Unused in flood fill. */
  void store_overlapping_segment(const int, const int, const bool, const int, const int) override
  {
  }

  bool execute_fill() override
  {
    return true;
  }
};

void flood_fill_exit(const wmOperator &op)
{
  FloodFillOperation *fill_op = static_cast<FloodFillOperation *>(op.customdata);
  fill_op->fill_exit();
  MEM_delete(fill_op);
}

bool flood_fill_exec(const wmOperator &op)
{
  FloodFillOperation &fill_op = *static_cast<FloodFillOperation *>(op.customdata);

  /* TODO... */
  printf("Perform flood fill algorithm on %lld strokes at frame %d \n",
         fill_op.curves_2d.point_offset.size(),
         fill_op.frame_number);

  return true;
}

int flood_fill_modal(const wmOperator &op, const wmEvent &event)
{
  FloodFillOperation *fill_op = static_cast<FloodFillOperation *>(op.customdata);
  return fill_op->fill_modal(op, event);
}

bool flood_fill_invoke(bContext *C, wmOperator *op)
{
  FloodFillOperation *fill_op = MEM_new<FloodFillOperation>(__func__);
  op->customdata = fill_op;
  return fill_op->fill_init(C);
}

}  // namespace blender::ed::greasepencil::fill
