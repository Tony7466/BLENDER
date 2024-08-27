/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_buffer_pool.hh"
#include "vk_buffer.hh"

namespace blender::gpu {
VKBuffer &VKBufferPool::Pool::acquire()
{
  if (free_items.is_empty()) {
    values.append(MEM_new<VKBuffer>(__func__));
    free_items.push(values.last());
  }
  VKBuffer &buffer = *free_items.pop();
  return buffer;
}

void VKBufferPool::Pool::release(VKBuffer &buffer)
{
  free_items.push(&buffer);
}

void VKBufferPool::Pool::free_data(VKDevice &device)
{
  free_items.clear();
  for (VKBuffer *buffer : values) {
    buffer->free_immediately(device);
    MEM_delete(buffer);
  }
  values.clear();
}

VKBuffer &VKBufferPool::acquire(VkDeviceSize size)
{
  if (!pools_.contains(size)) {
    pools_.add(size, Pool());
  }

  Pool &pool = pools_.lookup(size);
  VKBuffer &buffer = pool.acquire();
  if (!buffer.is_allocated()) {
    buffer.create(size,
                  GPU_USAGE_DYNAMIC,
                  VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                  true);
  }
  return buffer;
}

void VKBufferPool::release(VKBuffer &buffer)
{
  released_.push(&buffer);
}

void VKBufferPool::reset()
{
  while (!released_.is_empty()) {
    VKBuffer *buffer = released_.pop();
    Pool &pool = pools_.lookup(buffer->size_in_bytes());
    pool.release(*buffer);
  }
}

void VKBufferPool::free_data(VKDevice &device)
{
  for (Pool &pool : pools_.values()) {
    pool.free_data(device);
  }
  pools_.clear();
}

}  // namespace blender::gpu
