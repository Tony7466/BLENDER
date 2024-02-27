/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_WriteBufferOperation.h"

namespace blender::compositor {

WriteBufferOperation::WriteBufferOperation(DataType datatype)
{
  this->add_input_socket(datatype);
  memory_proxy_ = new MemoryProxy(datatype);
  memory_proxy_->set_write_buffer_operation(this);
  flags_.is_write_buffer_operation = true;
}
WriteBufferOperation::~WriteBufferOperation()
{
  if (memory_proxy_) {
    delete memory_proxy_;
    memory_proxy_ = nullptr;
  }
}

void WriteBufferOperation::execute_pixel_sampled(float output[4],
                                                 float x,
                                                 float y,
                                                 PixelSampler sampler)
{
  input_->read_sampled(output, x, y, sampler);
}

void WriteBufferOperation::init_execution()
{
  input_ = this->get_input_operation(0);
  memory_proxy_->allocate(this->get_width(), this->get_height());
}

void WriteBufferOperation::deinit_execution()
{
  input_ = nullptr;
  memory_proxy_->free();
}

void WriteBufferOperation::execute_region(rcti *rect, uint /*tile_number*/)
{
  MemoryBuffer *memory_buffer = memory_proxy_->get_buffer();
  float *buffer = memory_buffer->get_buffer();
  const uint8_t num_channels = memory_buffer->get_num_channels();
  if (input_->get_flags().complex) {
    void *data = input_->initialize_tile_data(rect);
    int x1 = rect->xmin;
    int y1 = rect->ymin;
    int x2 = rect->xmax;
    int y2 = rect->ymax;
    int x;
    int y;
    bool breaked = false;
    for (y = y1; y < y2 && (!breaked); y++) {
      int offset4 = (y * memory_buffer->get_width() + x1) * num_channels;
      for (x = x1; x < x2; x++) {
        input_->read(&(buffer[offset4]), x, y, data);
        offset4 += num_channels;
      }
      if (is_braked()) {
        breaked = true;
      }
    }
    if (data) {
      input_->deinitialize_tile_data(rect, data);
      data = nullptr;
    }
  }
  else {
    int x1 = rect->xmin;
    int y1 = rect->ymin;
    int x2 = rect->xmax;
    int y2 = rect->ymax;

    int x;
    int y;
    bool breaked = false;
    for (y = y1; y < y2 && (!breaked); y++) {
      int offset4 = (y * memory_buffer->get_width() + x1) * num_channels;
      for (x = x1; x < x2; x++) {
        input_->read_sampled(&(buffer[offset4]), x, y, PixelSampler::Nearest);
        offset4 += num_channels;
      }
      if (is_braked()) {
        breaked = true;
      }
    }
  }
}

void WriteBufferOperation::determine_canvas(const rcti &preferred_area, rcti &r_area)
{
  NodeOperation::determine_canvas(preferred_area, r_area);
  /* make sure there is at least one pixel stored in case the input is a single value */
  single_value_ = false;
  if (BLI_rcti_size_x(&r_area) == 0) {
    r_area.xmax += 1;
    single_value_ = true;
  }
  if (BLI_rcti_size_y(&r_area) == 0) {
    r_area.ymax += 1;
    single_value_ = true;
  }
}

void WriteBufferOperation::read_resolution_from_input_socket()
{
  NodeOperation *input_operation = this->get_input_operation(0);
  this->set_width(input_operation->get_width());
  this->set_height(input_operation->get_height());
}

}  // namespace blender::compositor
