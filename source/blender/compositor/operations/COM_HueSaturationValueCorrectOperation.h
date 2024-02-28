/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_CurveBaseOperation.h"
#include "COM_NodeOperation.h"

namespace blender::compositor {

class HueSaturationValueCorrectOperation : public CurveBaseOperation {
 private:
  /**
   * Cached reference to the input_program
   */
  SocketReader *input_program_;

 public:
  HueSaturationValueCorrectOperation();

  /**
   * Initialize the execution
   */
  void init_execution() override;

  /**
   * Deinitialize the execution
   */
  void deinit_execution() override;

  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) override;
};

}  // namespace blender::compositor
