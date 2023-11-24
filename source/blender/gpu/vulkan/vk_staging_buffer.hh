/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_buffer.hh"
#include "vk_common.hh"

namespace blender::gpu {

class VKStagingBuffer {
 public:
  enum class Direction {
    HostToDevice,
    DeviceToHost,
  };

 private:
  const VKBuffer &device_buffer_;
  VKBuffer host_buffer_;

 public:
  VKStagingBuffer(const VKBuffer &device_buffer, Direction direction);

  void copy_to_device(VKContext &context);
  void copy_from_device(VKContext &context);
  const VKBuffer &host_buffer_get() const
  {
    return host_buffer_;
  }
};
}  // namespace blender::gpu
