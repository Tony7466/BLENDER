/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"
#include "vk_resource_tracker.hh"

#include "BLI_utility_mixins.hh"

namespace blender::gpu {
class VKDevice;

/** Command buffer to keep track of the life-time of a command buffer. */
class VKCommandBuffer : NonCopyable, NonMovable {
  VkCommandBuffer vk_command_buffer_ = VK_NULL_HANDLE;

  struct {
    /**
     * The number of command added to the command buffer since last submission.
     */
    uint64_t recorded_command_counts = 0;
  } state;

 public:
  void init(VkCommandBuffer vk_command_buffer);
  void begin_recording();
  void end_recording();
  void reset();

  /**
   * Receive the vulkan handle of the command buffer.
   */
  VkCommandBuffer vk_command_buffer() const
  {
    return vk_command_buffer_;
  }

  bool has_recorded_commands() const
  {
    return state.recorded_command_counts != 0;
  }

  void command_recorded()
  {
    state.recorded_command_counts++;
  }
};

}  // namespace blender::gpu
