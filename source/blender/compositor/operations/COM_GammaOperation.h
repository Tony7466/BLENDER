/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_MultiThreadedRowOperation.h"

namespace blender::compositor {

class GammaOperation : public MultiThreadedRowOperation {
 private:
  /**
   * Cached reference to the input_program
   */
  SocketReader *input_program_;
  SocketReader *input_gamma_program_;

 public:
  GammaOperation();

  /**
   * Initialize the execution
   */
  void init_execution() override;

  /**
   * Deinitialize the execution
   */
  void deinit_execution() override;

  void update_memory_buffer_row(PixelCursor &p) override;
};

}  // namespace blender::compositor
