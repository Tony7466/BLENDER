/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_InvertOperation.h"

namespace blender::compositor {

InvertOperation::InvertOperation()
{
  this->add_input_socket(DataType::Value);
  this->add_input_socket(DataType::Color);
  this->add_output_socket(DataType::Color);
  input_value_program_ = nullptr;
  input_color_program_ = nullptr;
  color_ = true;
  alpha_ = false;
  set_canvas_input_index(1);
  flags_.can_be_constant = true;
}
void InvertOperation::init_execution()
{
  input_value_program_ = this->get_input_socket_reader(0);
  input_color_program_ = this->get_input_socket_reader(1);
}

void InvertOperation::deinit_execution()
{
  input_value_program_ = nullptr;
  input_color_program_ = nullptr;
}

void InvertOperation::update_memory_buffer_partial(MemoryBuffer *output,
                                                   const rcti &area,
                                                   Span<MemoryBuffer *> inputs)
{
  for (BuffersIterator<float> it = output->iterate_with(inputs, area); !it.is_end(); ++it) {
    const float value = *it.in(0);
    const float inverted_value = 1.0f - value;
    const float *color = it.in(1);

    if (color_) {
      it.out[0] = (1.0f - color[0]) * value + color[0] * inverted_value;
      it.out[1] = (1.0f - color[1]) * value + color[1] * inverted_value;
      it.out[2] = (1.0f - color[2]) * value + color[2] * inverted_value;
    }
    else {
      copy_v3_v3(it.out, color);
    }

    if (alpha_) {
      it.out[3] = (1.0f - color[3]) * value + color[3] * inverted_value;
    }
    else {
      it.out[3] = color[3];
    }
  }
}

}  // namespace blender::compositor
