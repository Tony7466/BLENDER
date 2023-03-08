/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_buffer.hh"
#include "BLI_span.hh"

namespace blender::gpu {

VKBuffer::~VKBuffer()
{
  VKContext &context = *VKContext::get();
  free(context);
}

bool VKBuffer::is_allocated() const
{
  return allocation_ != VK_NULL_HANDLE;
}

static VmaAllocationCreateFlagBits vma_allocation_flags(GPUUsageType usage)
{
  switch (usage) {
    case GPU_USAGE_STATIC:
      return static_cast<VmaAllocationCreateFlagBits>(
          VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT);
    case GPU_USAGE_DYNAMIC:
      return static_cast<VmaAllocationCreateFlagBits>(
          VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT);
    case GPU_USAGE_DEVICE_ONLY:
      return static_cast<VmaAllocationCreateFlagBits>(
          VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
          VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT);
    case GPU_USAGE_FLAG_BUFFER_TEXTURE_ONLY:
    case GPU_USAGE_STREAM:
      break;
  }
  BLI_assert_msg(false, "Unimplemented GPUUsageType");
  return static_cast<VmaAllocationCreateFlagBits>(VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                                  VMA_ALLOCATION_CREATE_MAPPED_BIT);
}

bool VKBuffer::create(VKContext &context,
                      int64_t size_in_bytes,
                      GPUUsageType usage,
                      VkBufferUsageFlagBits buffer_usage)
{
  BLI_assert(!is_allocated());

  size_in_bytes_ = size_in_bytes;

  VmaAllocator allocator = context.mem_allocator_get();
  VkBufferCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  create_info.flags = 0;
  create_info.size = size_in_bytes;
  create_info.usage = buffer_usage;
  /* We use the same command queue for the compute and graphics pipeline, so it is safe to use
   * exclusive resource handling. */
  create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  create_info.queueFamilyIndexCount = 1;
  create_info.pQueueFamilyIndices = context.queue_family_ptr_get();

  VmaAllocationCreateInfo vma_create_info = {};
  vma_create_info.flags = vma_allocation_flags(usage);
  vma_create_info.priority = 1.0f;
  vma_create_info.usage = VMA_MEMORY_USAGE_AUTO;

  VkResult result = vmaCreateBuffer(
      allocator, &create_info, &vma_create_info, &vk_buffer_, &allocation_, nullptr);
  return result == VK_SUCCESS;
}

bool VKBuffer::update(VKContext &context, const void *data)
{
  /* TODO: When size <64Kb we should use vkCmdUpdateBuffer. */
  void *mapped_memory;
  bool result = map(context, &mapped_memory);
  if (result) {
    memcpy(mapped_memory, data, size_in_bytes_);
    unmap(context);
  }
  return result;
}

static bool is_uniform(Span<uint32_t> data)
{
  BLI_assert(!data.is_empty());
  uint32_t expected_value = data[0];
  for (uint32_t value : data.drop_front(1)) {
    if (value != expected_value) {
      return false;
    }
  }
  return true;
}

void VKBuffer::clear(VKContext &context, Span<uint32_t> data)
{
  VKCommandBuffer &command_buffer = context.command_buffer_get();
  if (is_uniform(data)) {
    command_buffer.fill(*this, *data.begin());
    return;
  }

  /* TODO: Use a compute shader to clear the buffer. This is performance wise not recommended, and
   * should be avoided. There are some cases where we don't have a choice. Especially when using
   * compute shaders.*/
  BLI_assert_unreachable();
}

bool VKBuffer::map(VKContext &context, void **r_mapped_memory) const
{
  VmaAllocator allocator = context.mem_allocator_get();
  VkResult result = vmaMapMemory(allocator, allocation_, r_mapped_memory);
  return result == VK_SUCCESS;
}

void VKBuffer::unmap(VKContext &context) const
{
  VmaAllocator allocator = context.mem_allocator_get();
  vmaUnmapMemory(allocator, allocation_);
}

bool VKBuffer::free(VKContext &context)
{
  VmaAllocator allocator = context.mem_allocator_get();
  vmaDestroyBuffer(allocator, vk_buffer_, allocation_);
  return true;
}

}  // namespace blender::gpu
