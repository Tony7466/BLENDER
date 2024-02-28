/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_ReadBufferOperation.h"

namespace blender::compositor {

class WrapOperation : public ReadBufferOperation {
 private:
  int wrapping_type_;

 public:
  WrapOperation(DataType datatype);

  void set_wrapping(int wrapping_type);
  float get_wrapped_original_xpos(float x);
  float get_wrapped_original_ypos(float y);

  void setFactorXY(float factorX, float factorY);
};

}  // namespace blender::compositor
