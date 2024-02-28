/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_SetAlphaMultiplyOperation.h"

namespace blender::compositor {

SetAlphaMultiplyOperation::SetAlphaMultiplyOperation()
{
  this->add_input_socket(DataType::Color);
  this->add_input_socket(DataType::Value);
  this->add_output_socket(DataType::Color);

  input_color_ = nullptr;
  input_alpha_ = nullptr;
  flags_.can_be_constant = true;
}

void SetAlphaMultiplyOperation::init_execution()
{
  input_color_ = get_input_socket_reader(0);
  input_alpha_ = get_input_socket_reader(1);
}

void SetAlphaMultiplyOperation::deinit_execution()
{
  input_color_ = nullptr;
  input_alpha_ = nullptr;
}

void SetAlphaMultiplyOperation::update_memory_buffer_partial(MemoryBuffer *output,
                                                             const rcti &area,
                                                             Span<MemoryBuffer *> inputs)
{
  for (BuffersIterator<float> it = output->iterate_with(inputs, area); !it.is_end(); ++it) {
    const float *color = it.in(0);
    const float alpha = *it.in(1);
    mul_v4_v4fl(it.out, color, alpha);
  }
}

}  // namespace blender::compositor
