/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_MemoryProxy.h"
#include "COM_MemoryBuffer.h"

namespace blender::compositor {

MemoryProxy::MemoryProxy(DataType datatype)
{
  buffer_ = nullptr;
  datatype_ = datatype;
}

void MemoryProxy::allocate(uint width, uint height)
{
  rcti result;
  result.xmin = 0;
  result.xmax = width;
  result.ymin = 0;
  result.ymax = height;

  buffer_ = new MemoryBuffer(this, result, MemoryBufferState::Default);
}

void MemoryProxy::free()
{
  if (buffer_) {
    delete buffer_;
    buffer_ = nullptr;
  }
}

}  // namespace blender::compositor
