/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_TextureNoiseOperation.h"

#include "BLI_math_vector.hh"
#include "BLI_noise.hh"

namespace blender::compositor {

TextureNoiseOperation::TextureNoiseOperation()
{
  this->add_input_socket(DataType::Vector);  // Vector.

  this->add_input_socket(DataType::Value);  // Scale.
  this->add_input_socket(DataType::Value);  // Detail.
  this->add_input_socket(DataType::Value);  // Roughness.
  this->add_input_socket(DataType::Value);  // Lacunarity.
  this->add_input_socket(DataType::Value);  // Distortion.

  this->add_output_socket(DataType::Color);

  this->set_canvas_input_index(0);
}

void TextureNoiseOperation::init_execution()
{
  vector_reader_ = this->get_input_socket_reader(0);
  scale_reader_ = this->get_input_socket_reader(1);
  detail_reader_ = this->get_input_socket_reader(2);
  roughness_reader_ = this->get_input_socket_reader(3);
  lacunarity_reader_ = this->get_input_socket_reader(4);
  distortion_reader_ = this->get_input_socket_reader(5);
}

void TextureNoiseOperation::deinit_execution()
{
  vector_reader_ = nullptr;
  scale_reader_ = nullptr;
  detail_reader_ = nullptr;
  roughness_reader_ = nullptr;
  lacunarity_reader_ = nullptr;
  distortion_reader_ = nullptr;
}

void TextureNoiseOperation::execute_pixel_sampled(float output[4],
                                                  const float x,
                                                  const float y,
                                                  const PixelSampler sampler)
{
  const float scale = scale_reader_->read_sampled_float(x, y, sampler);
  const float detail = detail_reader_->read_sampled_float(x, y, sampler);
  const float roughness = roughness_reader_->read_sampled_float(x, y, sampler);
  const float lacunarity = lacunarity_reader_->read_sampled_float(x, y, sampler);
  const float distortion = distortion_reader_->read_sampled_float(x, y, sampler);

  const float3 vector = vector_reader_->read_sampled_float3(x, y, sampler);

  const float3 position_vector = vector * scale;
  const float4 position{position_vector[0], position_vector[1], position_vector[2], 0.0f};
  const float3 c = noise::perlin_float3_fractal_distorted(
      position, detail, roughness, lacunarity, distortion, false);

  output[0] = c[0];
  output[1] = c[1];
  output[2] = c[2];
  output[3] = 1.0f;
}

void TextureNoiseOperation::update_memory_buffer_partial(MemoryBuffer *output,
                                                         const rcti &area,
                                                         Span<MemoryBuffer *> inputs)
{
  // XXX
}

void TextureNoiseOperation::determine_canvas(const rcti &preferred_area, rcti &r_area)
{
  NodeOperation::determine_canvas(preferred_area, r_area);
}

}  // namespace blender::compositor
