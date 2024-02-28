/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_MultiThreadedOperation.h"
#include "COM_QualityStepHelper.h"

namespace blender::compositor {

class BokehBlurOperation : public MultiThreadedOperation, public QualityStepHelper {
 private:
  SocketReader *input_program_;
  SocketReader *input_bokeh_program_;
  SocketReader *input_bounding_box_reader_;
  void update_size();
  float size_;
  bool sizeavailable_;

  bool extend_bounds_;

 public:
  BokehBlurOperation();

  void init_data() override;

  /**
   * The inner loop of this operation.
   */
  void execute_pixel(float output[4], int x, int y, void *data) override;

  /**
   * Initialize the execution
   */
  void init_execution() override;

  /**
   * Deinitialize the execution
   */
  void deinit_execution() override;

  void set_size(float size)
  {
    size_ = size;
    sizeavailable_ = true;
  }

  void set_extend_bounds(bool extend_bounds)
  {
    extend_bounds_ = extend_bounds;
  }

  void determine_canvas(const rcti &preferred_area, rcti &r_area) override;

  void get_area_of_interest(int input_idx, const rcti &output_area, rcti &r_input_area) override;
  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) override;
};

}  // namespace blender::compositor
