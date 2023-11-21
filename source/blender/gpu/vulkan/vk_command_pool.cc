/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_command_pool.hh"
#include "vk_backend.hh"
#include "vk_device.hh"
#include "vk_memory.hh"

namespace blender::gpu {

VKCommandPool::VKCommandPool() {}

VKCommandPool::~VKCommandPool()
{
  const VKDevice &device = VKBackend::get().device_get();
  /* TODO: free reusable buffers. */
  free(device);
}

void VKCommandPool::init(const VKDevice &device)
{
  BLI_assert(vk_command_pool_ == VK_NULL_HANDLE);

  VK_ALLOCATION_CALLBACKS;
  VkCommandPoolCreateInfo command_pool_info = {};
  command_pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  command_pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
  command_pool_info.queueFamilyIndex = device.queue_family_get();

  vkCreateCommandPool(
      device.device_get(), &command_pool_info, vk_allocation_callbacks, &vk_command_pool_);
  debug::object_label(vk_command_pool_, "CommandPool");
}

void VKCommandPool::free(const VKDevice &device)
{
  if (vk_command_pool_ == VK_NULL_HANDLE) {
    return;
  }

  VK_ALLOCATION_CALLBACKS;
  vkDestroyCommandPool(device.device_get(), vk_command_pool_, vk_allocation_callbacks);
  vk_command_pool_ = VK_NULL_HANDLE;
}

void VKCommandPool::allocate_buffers(const VKDevice &device,
                                     MutableSpan<VkCommandBuffer> command_buffers)
{
  VkCommandBufferAllocateInfo alloc_info = {};
  alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  alloc_info.commandPool = vk_command_pool_;
  alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  alloc_info.commandBufferCount = command_buffers.size();
  vkAllocateCommandBuffers(device.device_get(), &alloc_info, command_buffers.data());
}

void VKCommandPool::free_buffers(const VKDevice &device, Span<VkCommandBuffer> command_buffers)
{
  BLI_assert(vk_command_pool_ != VK_NULL_HANDLE);
  vkFreeCommandBuffers(
      device.device_get(), vk_command_pool_, command_buffers.size(), command_buffers.data());
  vkTrimCommandPool(device.device_get(), vk_command_pool_, 0);
}

}  // namespace blender::gpu