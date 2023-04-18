/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2011 Blender Foundation. */

#pragma once

#include "COM_MultiThreadedOperation.h"

namespace blender::compositor {

class KuwaharaOperation : public MultiThreadedOperation {
  SocketReader *image_reader_;

  int kernel_size_;
  int variation_;

 public:
  KuwaharaOperation();

  void init_execution() override;
  void deinit_execution() override;
  void execute_pixel_sampled(float output[4], float x, float y, PixelSampler sampler) override;

  void set_kernel_size(int kernel_size);
  int get_kernel_size();

  void set_variation(int variation);
  int get_variation();

  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) override;
};

}  // namespace blender::compositor
