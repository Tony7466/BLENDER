/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_MultiThreadedOperation.h"

namespace blender::compositor {

class FlipOperation : public MultiThreadedOperation {
 private:
  SocketReader *input_operation_;
  bool flip_x_;
  bool flip_y_;

 public:
  FlipOperation();

  void init_execution() override;
  void deinit_execution() override;
  void setFlipX(bool flipX)
  {
    flip_x_ = flipX;
  }
  void setFlipY(bool flipY)
  {
    flip_y_ = flipY;
  }

  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) override;
};

}  // namespace blender::compositor
