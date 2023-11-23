/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include <queue>

#include "vk_common.hh"
#include "vk_timeline_semaphore.hh"

#include "BLI_utility_mixins.hh"

namespace blender::gpu {
class VKDevice;
struct TimelineCommandBuffers {
  VKTimelineSemaphore::Value timeline_value;
  Vector<VkCommandBuffer> vk_command_buffers;
};

class VKCommandPool : public NonCopyable {
  VkCommandPool vk_command_pool_ = VK_NULL_HANDLE;
  std::queue<TimelineCommandBuffers> in_flight_command_buffers_;

  struct {
    uint64_t command_buffers_allocated = 0;
    uint64_t command_buffers_freed = 0;
    uint64_t command_buffers_in_flight = 0;
    uint64_t trimmed = 0;
    uint64_t reset = 0;
  } stats;

 public:
  VKCommandPool();
  ~VKCommandPool();

  void init(const VKDevice &device);
  void free(const VKDevice &device);
  /**
   * Reset the command buffer, releasing memory to system.
   *
   * NOTE: Resetting a command pool may only be used when no command buffer is in pending for
   * execution state.
   */
  void reset(const VKDevice &device);
  void trim(const VKDevice &device);

  void allocate_buffers(const VKDevice &device, MutableSpan<VkCommandBuffer> r_command_buffers);
  void mark_in_flight(TimelineCommandBuffers &in_flight);
  void free_completed_buffers(const VKDevice &device);
  void free_buffers(const VKDevice &device, Span<VkCommandBuffer> command_buffers);

  void debug_print() const;
};

}  // namespace blender::gpu
