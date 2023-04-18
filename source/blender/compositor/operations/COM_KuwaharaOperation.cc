/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2011 Blender Foundation. */

#include "COM_KuwaharaOperation.h"

namespace blender::compositor {

KuwaharaOperation::KuwaharaOperation()
{
  this->add_input_socket(DataType::Color);
  this->add_output_socket(DataType::Color);
  this->set_kernel_size(4.4f);

  this->flags_.is_fullframe_operation = true;
}

void KuwaharaOperation::init_execution()
{
  image_reader_ = this->get_input_socket_reader(0);
}

void KuwaharaOperation::deinit_execution()
{
  image_reader_ = nullptr;
}

void KuwaharaOperation::execute_pixel_sampled(float output[4], float x, float y, PixelSampler sampler)
{
  float input_value[4];
  image_reader_->read_sampled(input_value, x, y, sampler);

  output[0] = input_value[0] + 1.0;
  output[1] = input_value[1] + 2.0;
  output[2] = input_value[2] + 3.0;
  output[3] = input_value[3] + 4.0;
}

void KuwaharaOperation::set_kernel_size(int kernel_size)
{
  kernel_size_ = kernel_size;
}

int KuwaharaOperation::get_kernel_size()
{
  return kernel_size_;
}

void KuwaharaOperation::set_variation(int variation)
{
  variation_ = variation;
}

int KuwaharaOperation::get_variation()
{
  return variation_;
}

void KuwaharaOperation::update_memory_buffer_partial(MemoryBuffer *output,
                                                     const rcti &area,
                                                     Span<MemoryBuffer *> inputs)
{
  MemoryBuffer *image = inputs[0];

  for (BuffersIterator<float> it = output->iterate_with(inputs, area); !it.is_end(); ++it) {
    const int x = it.x;
    const int y = it.y;
    it.out[3] = image->get_value(x, y, 3);


    for(int ch = 0; ch < 3; ch++)
    {
      float sum[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      float var[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      int cnt[4] = {0, 0, 0, 0};

      for (int dy = -kernel_size_; dy <= kernel_size_; dy++)
      {
        for (int dx = -kernel_size_; dx <= kernel_size_; dx++)
        {

          int xx = x + dx;
          int yy = y + dy;
          if (xx >= 0 && yy >= 0 && xx < area.xmax && yy < area.ymax)
          {
            float v;
            v = image->get_value(xx, yy, ch);

            if (dx <= 0 && dy <= 0)
            {
              sum[0] += v;
              var[0] += v * v;
              cnt[0]++;
            }

            if (dx >= 0 && dy <= 0)
            {
              sum[1] += v;
              var[1] += v * v;
              cnt[1]++;
            }

            if (dx <= 0 && dy >= 0)
            {
              sum[2] += v;
              var[2] += v * v;
              cnt[2]++;
            }

            if (dx >= 0 && dy >= 0)
            {
              sum[3] += v;
              var[3] += v * v;
              cnt[3]++;
            }
          }
        }
      }

      std::vector<std::pair<double, int>> vec;
      for (int i = 0; i < 4; i++)
      {
        sum[i] = cnt[i] != 0 ? sum[i] / cnt[i] : 0.0f;
        var[i] = cnt[i] != 0 ? var[i] / cnt[i] : 0.0f;
        var[i] = sqrt(var[i] - sum[i] * sum[i]);
        vec.push_back(std::make_pair(var[i], i));
      }

      sort(vec.begin(), vec.end());
      float res = static_cast<float>(sum[vec[0].second]);

      output->get_value(x, y, ch) = res;
    }
  }
}

}  // namespace blender::compositor
