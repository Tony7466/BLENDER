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

void VKCommandPool::trim(const VKDevice &device)
{
  vkTrimCommandPool(device.device_get(), vk_command_pool_, 0);
  stats.trimmed += 1;
}

void VKCommandPool::reset(const VKDevice &device)
{
  if (stats.command_buffers_allocated != stats.command_buffers_freed) {
    return;
  }

  vkResetCommandPool(
      device.device_get(), vk_command_pool_, VK_COMMAND_POOL_RESET_RELEASE_RESOURCES_BIT);
  stats.reset += 1;
}

void VKCommandPool::allocate_buffers(const VKDevice &device,
                                     MutableSpan<VkCommandBuffer> r_command_buffers)
{
  if (r_command_buffers.is_empty()) {
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
  stats.command_buffers_freed += command_buffers.size();
}

void VKCommandPool::mark_in_flight(TimelineCommandBuffers &in_flight)
{
  if (in_flight_command_buffers_.empty() ||
      in_flight_command_buffers_.back().timeline_value == in_flight.timeline_value)
  {
    in_flight_command_buffers_.push(in_flight);
  }
  else {
    in_flight_command_buffers_.back().vk_command_buffers.extend(in_flight.vk_command_buffers);
  }
  stats.command_buffers_in_flight += in_flight.vk_command_buffers.size();
}

void VKCommandPool::free_completed_buffers(const VKDevice &device)
{
  const VKTimelineSemaphore &timeline_semaphore = device.timeline_semaphore_get();
  const VKTimelineSemaphore::Value last_completed = timeline_semaphore.last_completed_value_get();
  while (!in_flight_command_buffers_.empty() &&
         in_flight_command_buffers_.front().timeline_value < last_completed)
  {
    TimelineCommandBuffers &completed = in_flight_command_buffers_.front();
    free_buffers(device, completed.vk_command_buffers);
    in_flight_command_buffers_.pop();
  }
}

void VKCommandPool::debug_print() const
{
  std::cout << "VKCommandPool(";
  std::cout << "allocated=" << stats.command_buffers_allocated;
  std::cout << ",in_flight=" << stats.command_buffers_in_flight;
  std::cout << ",freed=" << stats.command_buffers_freed;
  std::cout << ",reset=" << stats.reset;
  std::cout << ",trimmed=" << stats.trimmed;
  std::cout << ")\n";
}

}  // namespace blender::gpu