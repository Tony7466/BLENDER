/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_ConvertColorProfileOperation.h"

#include "IMB_imbuf.hh"

namespace blender::compositor {

ConvertColorProfileOperation::ConvertColorProfileOperation()
{
  this->add_input_socket(DataType::Color);
  this->add_output_socket(DataType::Color);
  input_operation_ = nullptr;
  predivided_ = false;
}

void ConvertColorProfileOperation::init_execution()
{
  input_operation_ = this->get_input_socket_reader(0);
}

void ConvertColorProfileOperation::deinit_execution()
{
  input_operation_ = nullptr;
}

}  // namespace blender::compositor
