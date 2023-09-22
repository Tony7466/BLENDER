/* SPDX-FileCopyrightText: 2032 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_ConstantOperation.h"

namespace blender::compositor {

class TextureNoiseOperation : public MultiThreadedOperation {
  SocketReader *vector_reader_ = nullptr;
  SocketReader *scale_reader_ = nullptr;
  SocketReader *detail_reader_ = nullptr;
  SocketReader *roughness_reader_ = nullptr;
  SocketReader *lacunarity_reader_ = nullptr;
  SocketReader *distortion_reader_ = nullptr;

 public:
  TextureNoiseOperation();

  void init_execution() override;
  void deinit_execution() override;

  void execute_pixel_sampled(float output[4], float x, float y, PixelSampler sampler) override;
  void update_memory_buffer_partial(MemoryBuffer *output,
                                    const rcti &area,
                                    Span<MemoryBuffer *> inputs) override;

  void determine_canvas(const rcti &preferred_area, rcti &r_area) override;

 protected:
};

}  // namespace blender::compositor
