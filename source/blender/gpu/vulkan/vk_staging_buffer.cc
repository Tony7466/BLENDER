/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_staging_buffer.hh"
#include "vk_backend.hh"
#include "vk_command_buffers.hh"
#include "vk_context.hh"

namespace blender::gpu {

VKStagingBuffer::VKStagingBuffer(const VKBuffer &device_buffer, Direction direction)
    : device_buffer_(device_buffer)
{
  VkBufferUsageFlags usage;
  switch (direction) {
    case Direction::HostToDevice:
      usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
      break;
    case Direction::DeviceToHost:
      usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  }

  host_buffer_.create(device_buffer.size_in_bytes(), GPU_USAGE_STREAM, usage, true);
}

void VKStagingBuffer::copy_to_device(render_graph::VKRenderGraph &render_graph)
{
  BLI_assert(host_buffer_.is_allocated() && host_buffer_.is_mapped());
  VkBufferCopy vk_buffer_copy = {};
  vk_buffer_copy.srcOffset = 0;
  vk_buffer_copy.dstOffset = 0;
  vk_buffer_copy.size = device_buffer_.size_in_bytes();
  render_graph.add_copy_buffer_node(
      host_buffer_.vk_handle(), device_buffer_.vk_handle(), vk_buffer_copy);
}

void VKStagingBuffer::copy_from_device(render_graph::VKRenderGraph &render_graph)
{
  BLI_assert(host_buffer_.is_allocated() && host_buffer_.is_mapped());
  VkBufferCopy vk_buffer_copy = {};
  vk_buffer_copy.srcOffset = 0;
  vk_buffer_copy.dstOffset = 0;
  vk_buffer_copy.size = device_buffer_.size_in_bytes();
  render_graph.add_copy_buffer_node(
      device_buffer_.vk_handle(), host_buffer_.vk_handle(), vk_buffer_copy);
}

void VKStagingBuffer::free()
{
  host_buffer_.free();
}

}  // namespace blender::gpu
