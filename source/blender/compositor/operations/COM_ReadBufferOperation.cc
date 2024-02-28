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
