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
  free(device);
}

void VKCommandPool::init(const VKDevice &device)
{
  BLI_assert(vk_command_pool_ == VK_NULL_HANDLE);

  VK_ALLOCATION_CALLBACKS;
  VkCommandPoolCreateInfo command_pool_info = {};
  command_pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  command_pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT |
                            VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
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

void VKCommandPool::trim(const VKDevice &device)
{
  if (reusable_handles_.is_empty()) {
    return;
  }
  free_buffers(device, reusable_handles_);
  reusable_handles_.clear();
  vkTrimCommandPool(device.device_get(), vk_command_pool_, 0);
}

void VKCommandPool::allocate_buffers(const VKDevice &device,
                                     MutableSpan<VkCommandBuffer> r_command_buffers)
{
  if (r_command_buffers.is_empty()) {
    return;
  }
  if (reusable_handles_.size() >= r_command_buffers.size()) {
    for (int index : r_command_buffers.index_range()) {
      r_command_buffers[index] = reusable_handles_.pop_last();
    }

    return;
  }
  VkCommandBufferAllocateInfo alloc_info = {};
  alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  alloc_info.commandPool = vk_command_pool_;
  alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  alloc_info.commandBufferCount = r_command_buffers.size();
  vkAllocateCommandBuffers(device.device_get(), &alloc_info, r_command_buffers.data());
  stats.command_buffers_allocated += r_command_buffers.size();
}

void VKCommandPool::free_buffers(const VKDevice &device, Span<VkCommandBuffer> command_buffers)
{
  BLI_assert(vk_command_pool_ != VK_NULL_HANDLE);
  vkFreeCommandBuffers(
      device.device_get(), vk_command_pool_, command_buffers.size(), command_buffers.data());
}

void VKCommandPool::mark_buffers_reusable(VKTimelineSemaphore::Value &value)
{
  int completed = 0;
  for (InFlight &in_flight : in_flight_handles_) {
    if (in_flight.time < value) {
      completed += 1;
      reusable_handles_.extend(in_flight.command_buffers);
      in_flight.command_buffers.clear();
    }
    else {
      break;
    }
  }

  if (completed != 0) {
    in_flight_handles_.remove(0, completed);
  }
}

void VKCommandPool::mark_buffers_in_flight(Span<VkCommandBuffer> command_buffers,
                                           VKTimelineSemaphore::Value &value)
{
  if (in_flight_handles_.is_empty() || in_flight_handles_.last().time < value) {
    InFlight in_flight;
    in_flight.command_buffers.extend(command_buffers);
    in_flight.time = value;
    in_flight_handles_.append(in_flight);
    return;
  }
  in_flight_handles_.last().command_buffers.extend(command_buffers);
}

void VKCommandPool::debug_print() const
{
  std::cout << "VKCommandPool(";
  std::cout << "allocated=" << stats.command_buffers_allocated;
  std::cout << ",inflight=" << stats.command_buffers_in_flight;
  std::cout << ",reused=" << stats.command_buffers_reused;
  std::cout << ",reusable=" << reusable_handles_.size();
  std::cout << ")\n";
}

}  // namespace blender::gpu