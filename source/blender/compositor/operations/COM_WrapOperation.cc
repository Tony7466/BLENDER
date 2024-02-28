/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cmath>

#include "COM_WrapOperation.h"

namespace blender::compositor {

WrapOperation::WrapOperation(DataType datatype) : ReadBufferOperation(datatype)
{
  wrapping_type_ = CMP_NODE_WRAP_NONE;
}

inline float WrapOperation::get_wrapped_original_xpos(float x)
{
  if (this->get_width() == 0) {
    return 0;
  }
  while (x < 0) {
    x += this->get_width();
  }
  return fmodf(x, this->get_width());
}

inline float WrapOperation::get_wrapped_original_ypos(float y)
{
  if (this->get_height() == 0) {
    return 0;
  }
  while (y < 0) {
    y += this->get_height();
  }
  return fmodf(y, this->get_height());
}

void WrapOperation::set_wrapping(int wrapping_type)
{
  wrapping_type_ = wrapping_type;
}

}  // namespace blender::compositor
