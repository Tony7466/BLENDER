/* SPDX-FileCopyrightText: 2011 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_SplitOperation.h"

namespace blender::compositor {

SplitOperation::SplitOperation()
{
  this->add_input_socket(DataType::Color);
  this->add_input_socket(DataType::Color);
  this->add_output_socket(DataType::Color);
  image1Input_ = nullptr;
  image2Input_ = nullptr;

  flags_.can_be_constant = true;
}

void SplitOperation::init_execution()
{
  /* When initializing the tree during initial load the width and height can be zero. */
  image1Input_ = get_input_socket_reader(0);
  image2Input_ = get_input_socket_reader(1);
}

void SplitOperation::deinit_execution()
{
  image1Input_ = nullptr;
  image2Input_ = nullptr;
}

void SplitOperation::execute_pixel_sampled(float output[4],
                                           float x,
                                           float y,
                                           PixelSampler /*sampler*/)
{
  int perc = x_split_ ? split_percentage_ * this->get_width() / 100.0f :
                        split_percentage_ * this->get_height() / 100.0f;
  bool image1 = x_split_ ? x > perc : y > perc;
  if (image1) {
    image1Input_->read_sampled(output, x, y, PixelSampler::Nearest);
  }
  else {
    image2Input_->read_sampled(output, x, y, PixelSampler::Nearest);
  }
}

void SplitOperation::determine_canvas(const rcti &preferred_area, rcti &r_area)
{
  /* The larger image decides the canvas size.
   * This makes the result more predictable and more consistent with the viewport compositor. */
  rcti area_1, area_2;
  const bool determined_1 = this->get_input_socket(0)->determine_canvas(COM_AREA_NONE, area_1);
  const bool determined_2 = this->get_input_socket(1)->determine_canvas(COM_AREA_NONE, area_2);

  if (determined_1 && determined_2) {
    const int size_1 = BLI_rcti_size_x(&area_1) * BLI_rcti_size_y(&area_1);
    const int size_2 = BLI_rcti_size_x(&area_2) * BLI_rcti_size_y(&area_2);
    this->set_canvas_input_index(size_1 > size_2 ? 0 : 1);
  }
  else {
    this->set_canvas_input_index(determined_1 ? 0 : 1);
  }

  NodeOperation::determine_canvas(preferred_area, r_area);
}

void SplitOperation::update_memory_buffer_partial(MemoryBuffer *output,
                                                  const rcti &area,
                                                  Span<MemoryBuffer *> inputs)
{
  const int percent = x_split_ ? split_percentage_ * this->get_width() / 100.0f :
                                 split_percentage_ * this->get_height() / 100.0f;
  const size_t elem_bytes = COM_data_type_bytes_len(get_output_socket()->get_data_type());
  for (BuffersIterator<float> it = output->iterate_with(inputs, area); !it.is_end(); ++it) {
    const bool is_image1 = x_split_ ? it.x > percent : it.y > percent;
    memcpy(it.out, it.in(is_image1 ? 0 : 1), elem_bytes);
  }
}

}  // namespace blender::compositor
