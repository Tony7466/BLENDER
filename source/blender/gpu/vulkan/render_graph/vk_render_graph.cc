/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph.hh"

namespace blender::gpu::render_graph {

VKRenderGraph::VKRenderGraph(std::unique_ptr<VKCommandBufferInterface> command_buffer,
                             std::unique_ptr<VKScheduler> scheduler)
    : scheduler_(std::move(scheduler)), command_buffer_(std::move(command_buffer))
{
}

void VKRenderGraph::deinit()
{
  command_buffer_.reset();
  scheduler_.reset();
}

/* -------------------------------------------------------------------- */
/** \name Adding resources
 * \{ */

void VKRenderGraph::add_buffer(VkBuffer vk_buffer)
{
  std::scoped_lock lock(mutex_);
  resources_.add_buffer(vk_buffer);
}

void VKRenderGraph::add_image(VkImage vk_image, VkImageLayout vk_image_layout, ResourceOwner owner)
{
  std::scoped_lock lock(mutex_);
  resources_.add_image(vk_image, vk_image_layout, owner);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Removing resources
 * \{ */

void VKRenderGraph::remove_buffer(VkBuffer vk_buffer)
{
  std::scoped_lock lock(mutex_);
  ResourceHandle handle = resources_.get_buffer_handle(vk_buffer);
  command_builder_.remove_resource(handle);
  resources_.remove_buffer(vk_buffer);
}

void VKRenderGraph::remove_image(VkImage vk_image)
{
  std::scoped_lock lock(mutex_);
  ResourceHandle handle = resources_.get_image_handle(vk_image);
  command_builder_.remove_resource(handle);
  resources_.remove_image(vk_image);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Submit graph
 * \{ */

void VKRenderGraph::submit_for_present(VkImage vk_swapchain_image)
{
  /* Needs to be executed at forehand as `add_node` also locks the mutex. */
  VKSynchronizationNode::CreateInfo synchronization = {};
  synchronization.vk_image = vk_swapchain_image;
  synchronization.vk_image_layout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
  add_node<VKSynchronizationNode, VKSynchronizationNode::CreateInfo>(synchronization);

  std::scoped_lock lock(mutex_);
  command_builder_.reset(*this);
  command_buffer_->begin_recording();
  command_builder_.build_image(*this, vk_swapchain_image);
  command_buffer_->end_recording();
  /* TODO: It is better to create and return a semaphore. this semaphore can be passed in the
   * swapchain to ensure GPU synchronization. This also require a second semaphore to pause drawing
   * until the swapchain has completed its drawing phase.
   *
   * Currently using CPU synchronization for safety. */
  command_buffer_->submit_with_cpu_synchronization();
  command_builder_.update_state_after_submission(*this);
  command_buffer_->wait_for_cpu_synchronization();
}

void VKRenderGraph::submit_buffer_for_read_back(VkBuffer vk_buffer)
{
  std::scoped_lock lock(mutex_);
  command_builder_.reset(*this);
  command_buffer_->begin_recording();
  command_builder_.build_buffer(*this, vk_buffer);
  command_buffer_->end_recording();
  command_buffer_->submit_with_cpu_synchronization();
  command_builder_.update_state_after_submission(*this);
  command_buffer_->wait_for_cpu_synchronization();
}

/** \} */

}  // namespace blender::gpu::render_graph
