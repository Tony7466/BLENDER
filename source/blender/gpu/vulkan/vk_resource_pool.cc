/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_resource_pool.hh"
#include "vk_backend.hh"
#include "vk_memory.hh"

namespace blender::gpu {

void VKResourcePool::discard_image(VkImage vk_image, VmaAllocation vma_allocation)
{
  discarded_images_.append(std::pair(vk_image, vma_allocation));
}

void VKResourcePool::discard_image_view(VkImageView vk_image_view)
{
  discarded_image_views_.append(vk_image_view);
}

void VKResourcePool::discard_buffer(VkBuffer vk_buffer, VmaAllocation vma_allocation)
{
  discarded_buffers_.append(std::pair(vk_buffer, vma_allocation));
}

void VKResourcePool::destroy_discarded_resources()
{
  VKDevice &device = VKBackend::get().device;
  VK_ALLOCATION_CALLBACKS

  while (!discarded_image_views_.is_empty()) {
    VkImageView vk_image_view = discarded_image_views_.pop_last();
    vkDestroyImageView(device.vk_handle(), vk_image_view, vk_allocation_callbacks);
  }

  while (!discarded_images_.is_empty()) {
    std::pair<VkImage, VmaAllocation> image_allocation = discarded_images_.pop_last();
    device.resources.remove_image(image_allocation.first);
    vmaDestroyImage(device.mem_allocator_get(), image_allocation.first, image_allocation.second);
  }

  while (!discarded_buffers_.is_empty()) {
    std::pair<VkBuffer, VmaAllocation> buffer_allocation = discarded_buffers_.pop_last();
    device.resources.remove_buffer(buffer_allocation.first);
    vmaDestroyBuffer(
        device.mem_allocator_get(), buffer_allocation.first, buffer_allocation.second);
  }
}

}  // namespace blender::gpu
