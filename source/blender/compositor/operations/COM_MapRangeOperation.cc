/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_MapRangeOperation.h"

namespace blender::compositor {

MapRangeOperation::MapRangeOperation()
{
  this->add_input_socket(DataType::Value);
  this->add_input_socket(DataType::Value);
  this->add_input_socket(DataType::Value);
  this->add_input_socket(DataType::Value);
  this->add_input_socket(DataType::Value);
  this->add_output_socket(DataType::Value);
  input_operation_ = nullptr;
  use_clamp_ = false;
  flags_.can_be_constant = true;
}

void MapRangeOperation::init_execution()
{
  input_operation_ = this->get_input_socket_reader(0);
  source_min_operation_ = this->get_input_socket_reader(1);
  source_max_operation_ = this->get_input_socket_reader(2);
  dest_min_operation_ = this->get_input_socket_reader(3);
  dest_max_operation_ = this->get_input_socket_reader(4);
}

/* The code below assumes all data is inside range +- this, and that input buffer is single channel
 */
#define BLENDER_ZMAX 10000.0f

void MapRangeOperation::deinit_execution()
{
  input_operation_ = nullptr;
  source_min_operation_ = nullptr;
  source_max_operation_ = nullptr;
  dest_min_operation_ = nullptr;
  dest_max_operation_ = nullptr;
}

void MapRangeOperation::update_memory_buffer_partial(MemoryBuffer *output,
                                                     const rcti &area,
                                                     Span<MemoryBuffer *> inputs)
{
  for (BuffersIterator<float> it = output->iterate_with(inputs, area); !it.is_end(); ++it) {
    const float source_min = *it.in(1);
    const float source_max = *it.in(2);
    if (fabsf(source_max - source_min) < 1e-6f) {
      it.out[0] = 0.0f;
      continue;
    }

    float value = *it.in(0);
    const float dest_min = *it.in(3);
    const float dest_max = *it.in(4);
    if (value >= -BLENDER_ZMAX && value <= BLENDER_ZMAX) {
      value = (value - source_min) / (source_max - source_min);
      value = dest_min + value * (dest_max - dest_min);
    }
    else if (value > BLENDER_ZMAX) {
      value = dest_max;
    }
    else {
      value = dest_min;
    }

    if (use_clamp_) {
      if (dest_max > dest_min) {
        CLAMP(value, dest_min, dest_max);
      }
      else {
        CLAMP(value, dest_max, dest_min);
      }
    }

    it.out[0] = value;
  }
}

}  // namespace blender::compositor
