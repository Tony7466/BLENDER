/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#include "COM_KuwaharaClassicOperation.h"

namespace blender::compositor {

KuwaharaClassicOperation::KuwaharaClassicOperation()
{
  this->add_input_socket(DataType::Color);
  this->add_output_socket(DataType::Color);
  this->set_kernel_size(4);

  this->flags_.is_fullframe_operation = true;
}

void KuwaharaClassicOperation::init_execution()
{
  image_reader_ = this->get_input_socket_reader(0);
}

void KuwaharaClassicOperation::deinit_execution()
{
  image_reader_ = nullptr;
}

void KuwaharaClassicOperation::execute_pixel_sampled(float output[4],
                                                     float x,
                                                     float y,
                                                     PixelSampler sampler)
{
  /* Not implemented */
}

void KuwaharaClassicOperation::set_kernel_size(int kernel_size)
{
  kernel_size_ = kernel_size;
}

int KuwaharaClassicOperation::get_kernel_size()
{
  return kernel_size_;
}

void KuwaharaClassicOperation::update_memory_buffer_partial(MemoryBuffer *output,
                                                            const rcti &area,
                                                            Span<MemoryBuffer *> inputs)
{
  MemoryBuffer *image = inputs[0];

  for (BuffersIterator<float> it = output->iterate_with(inputs, area); !it.is_end(); ++it) {
    const int x = it.x;
    const int y = it.y;
    it.out[3] = image->get_value(x, y, 3);

    for (int ch = 0; ch < 3; ch++) {
      float sum[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      float var[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      int cnt[4] = {0, 0, 0, 0};

      /* Split surroundings of pixel into 4 overlapping regions */
      for (int dy = -kernel_size_; dy <= kernel_size_; dy++) {
        for (int dx = -kernel_size_; dx <= kernel_size_; dx++) {

          int xx = x + dx;
          int yy = y + dy;
          if (xx >= 0 && yy >= 0 && xx < image->get_width() && yy < image->get_height()) {
            float v;
            v = image->get_value(xx, yy, ch);

            if (dx <= 0 && dy <= 0) {
              sum[0] += v;
              var[0] += v * v;
              cnt[0]++;
            }

            if (dx >= 0 && dy <= 0) {
              sum[1] += v;
              var[1] += v * v;
              cnt[1]++;
            }

            if (dx <= 0 && dy >= 0) {
              sum[2] += v;
              var[2] += v * v;
              cnt[2]++;
            }

            if (dx >= 0 && dy >= 0) {
              sum[3] += v;
              var[3] += v * v;
              cnt[3]++;
            }
          }
        }
      }

      /* Compute region variances */
      for (int i = 0; i < 4; i++) {
        sum[i] = cnt[i] != 0 ? sum[i] / cnt[i] : 0.0f;
        var[i] = cnt[i] != 0 ? var[i] / cnt[i] : 0.0f;
        const float temp = sum[i] * sum[i];
        var[i] = var[i] > temp ? sqrt(var[i] - temp) : 0.0f;
      }

      /* Choose the region with lowest variance */
      float min_var = FLT_MAX;
      int min_index = 0;
      for (int i = 0; i < 4; i++) {
        if (var[i] < min_var) {
          min_var = var[i];
          min_index = i;
        }
      }
      output->get_value(x, y, ch) = sum[min_index];
    }
  }
}

}  // namespace blender::compositor
