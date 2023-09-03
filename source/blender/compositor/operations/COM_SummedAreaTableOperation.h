/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_SingleThreadedOperation.h"

namespace blender::compositor {

/**
 * \brief base class of CalculateMean, implementing the simple CalculateMean
 * \ingroup operation
 */
class SummedAreaTableOperation : public SingleThreadedOperation {

 public:
  SummedAreaTableOperation();

  enum eMode { Identity = 1, Squared };

  void set_mode(const eMode mode);
  eMode get_mode();

  /**
   * Initialize the execution
   */
  void init_execution() override;

  /**
   * Deinitialize the execution
   */
  void deinit_execution() override;

  bool determine_depending_area_of_interest(rcti *input,
                                            ReadBufferOperation *read_operation,
                                            rcti *output) override;

  void get_area_of_interest(int input_idx, const rcti &output_area, rcti &r_input_area) override;

  MemoryBuffer *create_memory_buffer(rcti *rect) override;

  void update_memory_buffer(MemoryBuffer *output,
                            const rcti &area,
                            Span<MemoryBuffer *> inputs) override;

 private:
  SocketReader *image_reader_;
  eMode mode_;
};

float4 summed_area_table_sum(MemoryBuffer *buffer, const rcti &area);

}  // namespace blender::compositor
