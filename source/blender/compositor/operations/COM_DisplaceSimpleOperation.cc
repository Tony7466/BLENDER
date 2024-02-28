/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_DisplaceSimpleOperation.h"

namespace blender::compositor {

DisplaceSimpleOperation::DisplaceSimpleOperation()
{
  this->add_input_socket(DataType::Color);
  this->add_input_socket(DataType::Vector);
  this->add_input_socket(DataType::Value);
  this->add_input_socket(DataType::Value);
  this->add_output_socket(DataType::Color);

  input_color_program_ = nullptr;
  input_vector_program_ = nullptr;
  input_scale_xprogram_ = nullptr;
  input_scale_yprogram_ = nullptr;

  flags_.can_be_constant = true;
}

void DisplaceSimpleOperation::init_execution()
{
  input_color_program_ = this->get_input_socket_reader(0);
  input_vector_program_ = this->get_input_socket_reader(1);
  input_scale_xprogram_ = this->get_input_socket_reader(2);
  input_scale_yprogram_ = this->get_input_socket_reader(3);

  width_x4_ = this->get_width() * 4;
  height_x4_ = this->get_height() * 4;
}

void DisplaceSimpleOperation::deinit_execution()
{
  input_color_program_ = nullptr;
  input_vector_program_ = nullptr;
  input_scale_xprogram_ = nullptr;
  input_scale_yprogram_ = nullptr;
}

void DisplaceSimpleOperation::get_area_of_interest(const int input_idx,
                                                   const rcti &output_area,
                                                   rcti &r_input_area)
{
  switch (input_idx) {
    case 0: {
      r_input_area = get_input_operation(input_idx)->get_canvas();
      break;
    }
    default: {
      r_input_area = output_area;
      break;
    }
  }
}

void DisplaceSimpleOperation::update_memory_buffer_partial(MemoryBuffer *output,
                                                           const rcti &area,
                                                           Span<MemoryBuffer *> inputs)
{
  const float width = this->get_width();
  const float height = this->get_height();
  const MemoryBuffer *input_color = inputs[0];
  for (BuffersIterator<float> it = output->iterate_with(inputs.drop_front(1), area); !it.is_end();
       ++it)
  {
    float scale_x = *it.in(1);
    float scale_y = *it.in(2);

    /* Clamp x and y displacement to triple image resolution -
     * to prevent hangs from huge values mistakenly plugged in eg. z buffers. */
    CLAMP(scale_x, -width_x4_, width_x4_);
    CLAMP(scale_y, -height_x4_, height_x4_);

    /* Main displacement in pixel space. */
    const float *vector = it.in(0);
    const float p_dx = vector[0] * scale_x;
    const float p_dy = vector[1] * scale_y;

    /* Displaced pixel in uv coords, for image sampling. */
    /* Clamp nodes to avoid glitches. */
    float u = it.x - p_dx + 0.5f;
    float v = it.y - p_dy + 0.5f;
    CLAMP(u, 0.0f, width - 1.0f);
    CLAMP(v, 0.0f, height - 1.0f);

    input_color->read_elem_checked(u, v, it.out);
  }
}

}  // namespace blender::compositor
