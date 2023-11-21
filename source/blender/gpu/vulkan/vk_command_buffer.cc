/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_command_buffer.hh"
#include "vk_backend.hh"

namespace blender::gpu {

void VKCommandBuffer::reset()
{
  vk_command_buffer_ = VK_NULL_HANDLE;
}

void VKCommandBuffer::init(const VkCommandBuffer vk_command_buffer)
{
  vk_command_buffer_ = vk_command_buffer;
}

void VKCommandBuffer::begin_recording()
{
  VkCommandBufferInheritanceInfo inheritance_info = {};
  inheritance_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_INHERITANCE_INFO;

  VkCommandBufferBeginInfo begin_info = {};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  begin_info.pInheritanceInfo = &inheritance_info;

  vkBeginCommandBuffer(vk_command_buffer_, &begin_info);
  state.recorded_command_counts = 0;
}

void VKCommandBuffer::end_recording()
{
  vkEndCommandBuffer(vk_command_buffer_);
}

}  // namespace blender::gpu
