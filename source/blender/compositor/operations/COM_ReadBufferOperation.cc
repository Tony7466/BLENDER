/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_ReadBufferOperation.h"

#include "COM_WriteBufferOperation.h"

namespace blender::compositor {

ReadBufferOperation::ReadBufferOperation(DataType datatype)
{
  this->add_output_socket(datatype);
  single_value_ = false;
  offset_ = 0;
  buffer_ = nullptr;
  flags_.is_read_buffer_operation = true;
}

void ReadBufferOperation::determine_canvas(const rcti &preferred_area, rcti &r_area)
{
  if (memory_proxy_ != nullptr) {
    WriteBufferOperation *operation = memory_proxy_->get_write_buffer_operation();
    operation->determine_canvas(preferred_area, r_area);
    operation->set_canvas(r_area);
    single_value_ = operation->is_single_value();
  }
}

void ReadBufferOperation::execute_pixel_extend(float output[4],
                                               float x,
                                               float y,
                                               PixelSampler sampler,
                                               MemoryBufferExtend extend_x,
                                               MemoryBufferExtend extend_y)
{
  if (single_value_) {
    /* write buffer has a single value stored at (0,0) */
    buffer_->read(output, 0, 0);
  }
  else if (sampler == PixelSampler::Nearest) {
    buffer_->read(output, x, y, extend_x, extend_y);
  }
  else {
    buffer_->read_bilinear(output, x, y, extend_x, extend_y);
  }
}

void ReadBufferOperation::execute_pixel_filtered(
    float output[4], float x, float y, float dx[2], float dy[2])
{
  if (single_value_) {
    /* write buffer has a single value stored at (0,0) */
    buffer_->read(output, 0, 0);
  }
  else {
    const float uv[2] = {x, y};
    const float deriv[2][2] = {{dx[0], dx[1]}, {dy[0], dy[1]}};
    buffer_->readEWA(output, uv, deriv);
  }
}

void ReadBufferOperation::read_resolution_from_write_buffer()
{
  if (memory_proxy_ != nullptr) {
    WriteBufferOperation *operation = memory_proxy_->get_write_buffer_operation();
    this->set_width(operation->get_width());
    this->set_height(operation->get_height());
  }
}

void ReadBufferOperation::update_memory_buffer()
{
  buffer_ = this->get_memory_proxy()->get_buffer();
}

}  // namespace blender::compositor
