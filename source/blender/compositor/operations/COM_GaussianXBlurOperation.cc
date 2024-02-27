/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_GaussianXBlurOperation.h"

namespace blender::compositor {

GaussianXBlurOperation::GaussianXBlurOperation() : GaussianBlurBaseOperation(eDimension::X) {}

void *GaussianXBlurOperation::initialize_tile_data(rcti * /*rect*/)
{
  lock_mutex();
  if (!sizeavailable_) {
    update_gauss();
  }
  void *buffer = get_input_operation(0)->initialize_tile_data(nullptr);
  unlock_mutex();
  return buffer;
}

/* TODO(manzanilla): to be removed with tiled implementation. */
void GaussianXBlurOperation::init_execution()
{
  GaussianBlurBaseOperation::init_execution();

  init_mutex();
}

/* TODO(manzanilla): to be removed with tiled implementation. */
void GaussianXBlurOperation::update_gauss()
{
  if (gausstab_ == nullptr) {
    update_size();
    float rad = max_ff(size_ * data_.sizex, 0.0f);
    rad = min_ff(rad, MAX_GAUSSTAB_RADIUS);
    filtersize_ = min_ii(ceil(rad), MAX_GAUSSTAB_RADIUS);

    gausstab_ = BlurBaseOperation::make_gausstab(rad, filtersize_);
#if BLI_HAVE_SSE2
    gausstab_sse_ = BlurBaseOperation::convert_gausstab_sse(gausstab_, filtersize_);
#endif
  }
}

void GaussianXBlurOperation::execute_pixel(float output[4], int x, int y, void *data)
{
  float ATTR_ALIGN(16) color_accum[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float multiplier_accum = 0.0f;
  MemoryBuffer *input_buffer = (MemoryBuffer *)data;
  const rcti &input_rect = input_buffer->get_rect();
  float *buffer = input_buffer->get_buffer();
  int bufferwidth = input_buffer->get_width();
  int bufferstartx = input_rect.xmin;
  int bufferstarty = input_rect.ymin;

  int xmin = max_ii(x - filtersize_, input_rect.xmin);
  int xmax = min_ii(x + filtersize_ + 1, input_rect.xmax);
  int ymin = max_ii(y, input_rect.ymin);

  int step = get_step();
  int offsetadd = get_offset_add();
  int bufferindex = ((xmin - bufferstartx) * 4) + ((ymin - bufferstarty) * 4 * bufferwidth);

#if BLI_HAVE_SSE2
  __m128 accum_r = _mm_load_ps(color_accum);
  for (int nx = xmin, index = (xmin - x) + filtersize_; nx < xmax; nx += step, index += step) {
    __m128 reg_a = _mm_load_ps(&buffer[bufferindex]);
    reg_a = _mm_mul_ps(reg_a, gausstab_sse_[index]);
    accum_r = _mm_add_ps(accum_r, reg_a);
    multiplier_accum += gausstab_[index];
    bufferindex += offsetadd;
  }
  _mm_store_ps(color_accum, accum_r);
#else
  for (int nx = xmin, index = (xmin - x) + filtersize_; nx < xmax; nx += step, index += step) {
    const float multiplier = gausstab_[index];
    madd_v4_v4fl(color_accum, &buffer[bufferindex], multiplier);
    multiplier_accum += multiplier;
    bufferindex += offsetadd;
  }
#endif
  mul_v4_v4fl(output, color_accum, 1.0f / multiplier_accum);
}

void GaussianXBlurOperation::deinit_execution()
{
  GaussianBlurBaseOperation::deinit_execution();

  if (gausstab_) {
    MEM_freeN(gausstab_);
    gausstab_ = nullptr;
  }
#if BLI_HAVE_SSE2
  if (gausstab_sse_) {
    MEM_freeN(gausstab_sse_);
    gausstab_sse_ = nullptr;
  }
#endif

  deinit_mutex();
}

bool GaussianXBlurOperation::determine_depending_area_of_interest(
    rcti *input, ReadBufferOperation *read_operation, rcti *output)
{
  rcti new_input;

  if (!sizeavailable_) {
    rcti size_input;
    size_input.xmin = 0;
    size_input.ymin = 0;
    size_input.xmax = 5;
    size_input.ymax = 5;
    NodeOperation *operation = this->get_input_operation(1);
    if (operation->determine_depending_area_of_interest(&size_input, read_operation, output)) {
      return true;
    }
  }
  {
    if (sizeavailable_ && gausstab_ != nullptr) {
      new_input.xmax = input->xmax + filtersize_ + 1;
      new_input.xmin = input->xmin - filtersize_ - 1;
      new_input.ymax = input->ymax;
      new_input.ymin = input->ymin;
    }
    else {
      new_input.xmax = this->get_width();
      new_input.xmin = 0;
      new_input.ymax = this->get_height();
      new_input.ymin = 0;
    }
    return NodeOperation::determine_depending_area_of_interest(&new_input, read_operation, output);
  }
}

}  // namespace blender::compositor
