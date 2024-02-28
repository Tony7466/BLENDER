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
