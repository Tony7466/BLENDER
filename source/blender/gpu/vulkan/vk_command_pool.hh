/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"
#include "vk_timeline_semaphore.hh"

#include "BLI_utility_mixins.hh"

namespace blender::gpu {
class VKDevice;

class VKCommandPool : public NonCopyable {
  struct InFlight {
    VKTimelineSemaphore::Value time;
    Vector<VkCommandBuffer> command_buffers;
  };
  VkCommandPool vk_command_pool_ = VK_NULL_HANDLE;
  Vector<VkCommandBuffer> reusable_handles_;
  Vector<InFlight> in_flight_handles_;

  struct {
    uint64_t command_buffers_allocated = 0;
    uint64_t command_buffers_in_flight = 0;
    uint64_t command_buffers_reused = 0;
  } stats;

 public:
  VKCommandPool();
  ~VKCommandPool();

  void init(const VKDevice &device);
  void free(const VKDevice &device);
  void trim(const VKDevice &device);
  void allocate_buffers(const VKDevice &device, MutableSpan<VkCommandBuffer> r_command_buffers);
  void free_buffers(const VKDevice &device, Span<VkCommandBuffer> command_buffers);

  void mark_buffers_in_flight(Span<VkCommandBuffer> command_buffers,
                              VKTimelineSemaphore::Value &value);
  void mark_buffers_reusable(VKTimelineSemaphore::Value &value);

  void debug_print() const;
};

}  // namespace blender::gpu
