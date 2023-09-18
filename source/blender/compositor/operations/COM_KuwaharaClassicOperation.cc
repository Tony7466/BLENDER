/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"

#include "IMB_colormanagement.h"

#include "COM_KuwaharaClassicOperation.h"

namespace blender::compositor {

KuwaharaClassicOperation::KuwaharaClassicOperation()
{
  this->add_input_socket(DataType::Color);
  this->add_input_socket(DataType::Color);
  this->add_input_socket(DataType::Color);
  this->add_input_socket(DataType::Value);
  this->add_output_socket(DataType::Color);
  this->set_kernel_size(4);
  this->set_use_sat(true);

  this->flags_.is_fullframe_operation = true;
}

void KuwaharaClassicOperation::init_execution()
{
  image_reader_ = this->get_input_socket_reader(0);
  sat_reader_ = this->get_input_socket_reader(1);
  sat_squared_reader_ = this->get_input_socket_reader(2);
}

void KuwaharaClassicOperation::deinit_execution()
{
  image_reader_ = nullptr;
  sat_reader_ = nullptr;
  sat_squared_reader_ = nullptr;
}

void KuwaharaClassicOperation::execute_pixel_sampled(float output[4],
                                                     float x,
                                                     float y,
                                                     PixelSampler sampler)
{
  float3 mean_of_color[] = {float3(0.0f), float3(0.0f), float3(0.0f), float3(0.0f)};
  float3 mean_of_squared_color[] = {float3(0.0f), float3(0.0f), float3(0.0f), float3(0.0f)};
  int quadrant_pixel_count[] = {0, 0, 0, 0};

  if (use_sat_) {
    for (int q = 0; q < 4; q++) {
      /* A fancy expression to compute the sign of the quadrant q. */
      int2 sign = int2((q % 2) * 2 - 1, ((q / 2) * 2 - 1));

      int2 lower_bound = int2(x, y) -
                         int2(sign.x > 0 ? 0 : kernel_size_, sign.y > 0 ? 0 : kernel_size_);
      int2 upper_bound = int2(x, y) +
                         int2(sign.x < 0 ? 0 : kernel_size_, sign.y < 0 ? 0 : kernel_size_);

      /* Limit the quadrants to the image bounds. */
      int2 image_bound = int2(this->get_width(), this->get_height()) - int2(1);
      int2 corrected_lower_bound = math::min(image_bound, math::max(int2(0, 0), lower_bound));
      int2 corrected_upper_bound = math::min(image_bound, math::max(int2(0, 0), upper_bound));
      int2 region_size = corrected_upper_bound - corrected_lower_bound + int2(1, 1);
      quadrant_pixel_count[q] = region_size.x * region_size.y;

      rcti kernel_area;
      kernel_area.xmin = corrected_lower_bound[0];
      kernel_area.ymin = corrected_lower_bound[1];
      kernel_area.xmax = corrected_upper_bound[0];
      kernel_area.ymax = corrected_upper_bound[1];

      mean_of_color[q] = summed_area_table_sum_tiled(sat_reader_, kernel_area).xyz();
      mean_of_squared_color[q] =
          summed_area_table_sum_tiled(sat_squared_reader_, kernel_area).xyz();
    }
  }
  else {
    /* Split surroundings of pixel into 4 overlapping regions. */
    for (int dy = -kernel_size_; dy <= kernel_size_; dy++) {
      for (int dx = -kernel_size_; dx <= kernel_size_; dx++) {

        int xx = x + dx;
        int yy = y + dy;
        if (xx >= 0 && yy >= 0 && xx < this->get_width() && yy < this->get_height()) {

          float4 color;
          image_reader_->read_sampled(color, xx, yy, sampler);
          const float3 v = color.xyz();

          if (dx >= 0 && dy >= 0) {
            const int quadrant_index = 0;
            mean_of_color[quadrant_index] += v;
            mean_of_squared_color[quadrant_index] += v * v;
            quadrant_pixel_count[quadrant_index]++;
          }

          if (dx <= 0 && dy >= 0) {
            const int quadrant_index = 1;
            mean_of_color[quadrant_index] += v;
            mean_of_squared_color[quadrant_index] += v * v;
            quadrant_pixel_count[quadrant_index]++;
          }

          if (dx <= 0 && dy <= 0) {
            const int quadrant_index = 2;
            mean_of_color[quadrant_index] += v;
            mean_of_squared_color[quadrant_index] += v * v;
            quadrant_pixel_count[quadrant_index]++;
          }

          if (dx >= 0 && dy <= 0) {
            const int quadrant_index = 3;
            mean_of_color[quadrant_index] += v;
            mean_of_squared_color[quadrant_index] += v * v;
            quadrant_pixel_count[quadrant_index]++;
          }
        }
      }
    }
  }

  /* Choose the region with lowest variance. */
  float min_var = FLT_MAX;
  int min_index = 0;
  for (int i = 0; i < 4; i++) {
    mean_of_color[i] /= quadrant_pixel_count[i];
    mean_of_squared_color[i] /= quadrant_pixel_count[i];
    float3 color_variance = mean_of_squared_color[i] - mean_of_color[i] * mean_of_color[i];

    float variance = math::dot(color_variance, float3(1.0f));
    if (variance < min_var) {
      min_var = variance;
      min_index = i;
    }
  }

  output[0] = mean_of_color[min_index].x;
  output[1] = mean_of_color[min_index].y;
  output[2] = mean_of_color[min_index].z;

  /* No changes for alpha channel. */
  float tmp[4];
  image_reader_->read_sampled(tmp, x, y, sampler);
  output[3] = tmp[3];
}

void KuwaharaClassicOperation::set_kernel_size(int kernel_size)
{
  kernel_size_ = kernel_size;
}

int KuwaharaClassicOperation::get_kernel_size()
{
  return kernel_size_;
}

void KuwaharaClassicOperation::set_use_sat(bool use_sat)
{
  use_sat_ = use_sat;
}

bool KuwaharaClassicOperation::get_use_sat()
{
  return use_sat_;
}

void KuwaharaClassicOperation::update_memory_buffer_partial(MemoryBuffer *output,
                                                            const rcti &area,
                                                            Span<MemoryBuffer *> inputs)
{
  MemoryBuffer *image = inputs[0];
  MemoryBuffer *sat = inputs[1];
  MemoryBuffer *sat_squared = inputs[2];

  int width = image->get_width();
  int height = image->get_height();

  for (BuffersIterator<float> it = output->iterate_with(inputs, area); !it.is_end(); ++it) {
    const int x = it.x;
    const int y = it.y;

    float3 mean_of_color[4] = {float3(0.0f), float3(0.0f), float3(0.0f), float3(0.0f)};
    float3 mean_of_squared_color[4] = {float3(0.0f), float3(0.0f), float3(0.0f), float3(0.0f)};
    int quadrant_pixel_count[4] = {0, 0, 0, 0};

    if (use_sat_) {
      for (int q = 0; q < 4; q++) {
        /* A fancy expression to compute the sign of the quadrant q. */
        int2 sign = int2((q % 2) * 2 - 1, ((q / 2) * 2 - 1));

        int2 lower_bound = int2(x, y) -
                           int2(sign.x > 0 ? 0 : kernel_size_, sign.y > 0 ? 0 : kernel_size_);
        int2 upper_bound = int2(x, y) +
                           int2(sign.x < 0 ? 0 : kernel_size_, sign.y < 0 ? 0 : kernel_size_);

        /* Limit the quadrants to the image bounds. */
        int2 image_bound = int2(width, height) - int2(1);
        int2 corrected_lower_bound = math::min(image_bound, math::max(int2(0, 0), lower_bound));
        int2 corrected_upper_bound = math::min(image_bound, math::max(int2(0, 0), upper_bound));
        int2 region_size = corrected_upper_bound - corrected_lower_bound + int2(1, 1);
        quadrant_pixel_count[q] = region_size.x * region_size.y;

        rcti kernel_area;
        kernel_area.xmin = corrected_lower_bound[0];
        kernel_area.ymin = corrected_lower_bound[1];
        kernel_area.xmax = corrected_upper_bound[0];
        kernel_area.ymax = corrected_upper_bound[1];

        mean_of_color[q] = summed_area_table_sum(sat, kernel_area).xyz();
        mean_of_squared_color[q] = summed_area_table_sum(sat_squared, kernel_area).xyz();
      }
    }
    else {
      /* Split surroundings of pixel into 4 overlapping regions. */
      for (int dy = -kernel_size_; dy <= kernel_size_; dy++) {
        for (int dx = -kernel_size_; dx <= kernel_size_; dx++) {

          int xx = x + dx;
          int yy = y + dy;
          if (xx >= 0 && yy >= 0 && xx < image->get_width() && yy < image->get_height()) {

            float4 color;
            image->read_elem(xx, yy, &color.x);
            const float3 v = color.xyz();

            if (dx >= 0 && dy >= 0) {
              const int quadrant_index = 0;
              mean_of_color[quadrant_index] += v;
              mean_of_squared_color[quadrant_index] += v * v;
              quadrant_pixel_count[quadrant_index]++;
            }

            if (dx <= 0 && dy >= 0) {
              const int quadrant_index = 1;
              mean_of_color[quadrant_index] += v;
              mean_of_squared_color[quadrant_index] += v * v;
              quadrant_pixel_count[quadrant_index]++;
            }

            if (dx <= 0 && dy <= 0) {
              const int quadrant_index = 2;
              mean_of_color[quadrant_index] += v;
              mean_of_squared_color[quadrant_index] += v * v;
              quadrant_pixel_count[quadrant_index]++;
            }

            if (dx >= 0 && dy <= 0) {
              const int quadrant_index = 3;
              mean_of_color[quadrant_index] += v;
              mean_of_squared_color[quadrant_index] += v * v;
              quadrant_pixel_count[quadrant_index]++;
            }
          }
        }
      }
    }

    /* Choose the region with lowest variance. */
    float min_var = FLT_MAX;
    int min_index = 0;
    for (int i = 0; i < 4; i++) {
      mean_of_color[i] /= quadrant_pixel_count[i];
      mean_of_squared_color[i] /= quadrant_pixel_count[i];
      float3 color_variance = mean_of_squared_color[i] - mean_of_color[i] * mean_of_color[i];

      float variance = math::dot(color_variance, float3(1.0f));
      if (variance < min_var) {
        min_var = variance;
        min_index = i;
      }
    }

    it.out[0] = mean_of_color[min_index].x;
    it.out[1] = mean_of_color[min_index].y;
    it.out[2] = mean_of_color[min_index].z;

    /* No changes for alpha channel. */
    it.out[3] = image->get_value(x, y, 3);
  }
}

}  // namespace blender::compositor
