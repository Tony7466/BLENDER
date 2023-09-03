/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"

#include "COM_SummedAreaTableOperation.h"

namespace blender::compositor {

SummedAreaTableOperation::SummedAreaTableOperation()
{
  this->add_input_socket(DataType::Color);
  this->add_output_socket(DataType::Color);

  mode_ = eMode::Identity;

  this->flags_.is_fullframe_operation = true;
}
void SummedAreaTableOperation::init_execution()
{
  SingleThreadedOperation::init_execution();
  image_reader_ = this->get_input_socket_reader(0);
}

void SummedAreaTableOperation::deinit_execution()
{
  image_reader_ = nullptr;
  SingleThreadedOperation::deinit_execution();
}

bool SummedAreaTableOperation::determine_depending_area_of_interest(
    rcti * /*input*/, ReadBufferOperation *read_operation, rcti *output)
{
  rcti image_input;

  NodeOperation *operation = get_input_operation(0);
  image_input.xmax = operation->get_width();
  image_input.xmin = 0;
  image_input.ymax = operation->get_height();
  image_input.ymin = 0;
  if (operation->determine_depending_area_of_interest(&image_input, read_operation, output)) {
    return true;
  }
  return false;
}

void SummedAreaTableOperation::get_area_of_interest(int input_idx,
                                                    const rcti & /*output_area*/,
                                                    rcti &r_input_area)
{
  BLI_assert(input_idx == 0);
  r_input_area = get_input_operation(input_idx)->get_canvas();
}

void SummedAreaTableOperation::update_memory_buffer(MemoryBuffer *output,
                                                    const rcti &area,
                                                    Span<MemoryBuffer *> inputs)
{
  MemoryBuffer *image = inputs[0];

  for (BuffersIterator<float> it = output->iterate_with({}, area); !it.is_end(); ++it) {
    const int x = it.x;
    const int y = it.y;

    float4 color, upper, left, upper_left;
    image->read_elem(x, y, &color.x);

    float tmp[4];
    image->read_elem(x, y, tmp);

    output->read_elem_checked(x, y - 1, &upper.x);
    output->read_elem_checked(x - 1, y, &left.x);
    output->read_elem_checked(x - 1, y - 1, &upper_left.x);

    float4 sum = upper + left - upper_left;

    switch (mode_) {
      case eMode::Squared: {
        sum += color * color;
        break;
      }
      case eMode::Identity: {
        sum += color;
        break;
      }
      default: {
        BLI_assert_msg(0, "Mode not implemented");
        break;
      }
    }

    it.out[0] = sum.x;
    it.out[1] = sum.y;
    it.out[2] = sum.z;
    it.out[3] = sum.w;
  }
}

MemoryBuffer *SummedAreaTableOperation::create_memory_buffer(rcti *rect)
{
  MemoryBuffer *result = new MemoryBuffer(DataType::Color, *rect);
  PixelSampler sampler = PixelSampler::Nearest;

  for (BuffersIterator<float> it = result->iterate_with({}, *rect); !it.is_end(); ++it) {
    const int x = it.x;
    const int y = it.y;

    float4 color, upper, left, upper_left;
    image_reader_->read_sampled(color, x, y, sampler);

    result->read_elem_checked(x, y - 1, &upper.x);
    result->read_elem_checked(x - 1, y, &left.x);
    result->read_elem_checked(x - 1, y - 1, &upper_left.x);

    float4 sum = upper + left - upper_left;

    switch (mode_) {
      case eMode::Squared: {
        sum += color * color;
        break;
      }
      case eMode::Identity: {
        sum += color;
        break;
      }
      default: {
        BLI_assert_msg(0, "Mode not implemented");
        break;
      }
    }

    it.out[0] = sum.x;
    it.out[1] = sum.y;
    it.out[2] = sum.z;
    it.out[3] = sum.w;
  }

  return result;
}

void SummedAreaTableOperation::set_mode(eMode mode)
{
  mode_ = mode;
}

SummedAreaTableOperation::eMode SummedAreaTableOperation::get_mode()
{
  return mode_;
}

float4 summed_area_table_sum_tiled(SocketReader *buffer, const rcti &area)
{
  /*
   * a, b, c and d are the bounding box of the given area. They are defined as follows:
   *
   * y
   * ▲
   * │
   * │
   * │
   * ├────────x────────────────x
   * │        │c              d│
   * │        │                │
   * │        │                │
   * │        │                │
   * │        │                │
   * ├────────x────────────────x
   * │        │a              b│
   * │        │                │
   * │        │                │
   * └────────┴────────────────┴─────────► x
   *
   * Note: this is the same definition as in https://en.wikipedia.org/wiki/Summed-area_table
   * but using the blender convention with the origin being at the lower left.
   *
   */

  BLI_assert(area.xmin <= area.xmax && area.ymin <= area.ymax);

  int2 lower_bound(area.xmin, area.ymin);
  int2 upper_bound(area.xmax, area.ymax);

  int2 corrected_lower_bound = lower_bound - int2(1, 1);
  int2 corrected_upper_bound;
  corrected_upper_bound[0] = math::min((int)buffer->get_width() - 1, upper_bound[0]);
  corrected_upper_bound[1] = math::min((int)buffer->get_height() - 1, upper_bound[1]);

  float4 a, b, c, d, addend, substrahend;
  buffer->read_sampled(
      &a.x, corrected_upper_bound[0], corrected_upper_bound[1], PixelSampler::Nearest);
  buffer->read_sampled(
      &d.x, corrected_lower_bound[0], corrected_lower_bound[1], PixelSampler::Nearest);
  addend = a + d;

  buffer->read_sampled(
      &b.x, corrected_lower_bound[0], corrected_upper_bound[1], PixelSampler::Nearest);
  buffer->read_sampled(
      &c.x, corrected_upper_bound[0], corrected_lower_bound[1], PixelSampler::Nearest);
  substrahend = b + c;

  float4 sum = addend - substrahend;

  return sum;
}

float4 summed_area_table_sum(MemoryBuffer *buffer, const rcti &area)
{
  /*
   * a, b, c and d are the bounding box of the given area. They are defined as follows:
   *
   * y
   * ▲
   * │
   * │
   * │
   * ├────────x────────────────x
   * │        │c              d│
   * │        │                │
   * │        │                │
   * │        │                │
   * │        │                │
   * ├────────x────────────────x
   * │        │a              b│
   * │        │                │
   * │        │                │
   * └────────┴────────────────┴─────────► x
   *
   * Note: this is the same definition as in https://en.wikipedia.org/wiki/Summed-area_table
   * but using the blender convention with the origin being at the lower left.
   *
   */

  BLI_assert(area.xmin <= area.xmax && area.ymin <= area.ymax);

  int2 lower_bound(area.xmin, area.ymin);
  int2 upper_bound(area.xmax, area.ymax);

  int2 corrected_lower_bound = lower_bound - int2(1, 1);
  int2 corrected_upper_bound;
  corrected_upper_bound[0] = math::min(buffer->get_width() - 1, upper_bound[0]);
  corrected_upper_bound[1] = math::min(buffer->get_height() - 1, upper_bound[1]);

  float4 a, b, c, d, addend, substrahend;
  buffer->read_elem_checked(corrected_upper_bound[0], corrected_upper_bound[1], a);
  buffer->read_elem_checked(corrected_lower_bound[0], corrected_lower_bound[1], d);
  addend = a + d;

  buffer->read_elem_checked(corrected_lower_bound[0], corrected_upper_bound[1], b);
  buffer->read_elem_checked(corrected_upper_bound[0], corrected_lower_bound[1], c);
  substrahend = b + c;

  float4 sum = addend - substrahend;

  return sum;
}

}  // namespace blender::compositor
