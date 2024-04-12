/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph.hh"

namespace blender::gpu::render_graph {

VKRenderGraph::VKRenderGraph(std::unique_ptr<VKCommandBufferInterface> command_buffer,
                             VKResources &resources)
    : command_buffer_(std::move(command_buffer)), resources_(resources)
{
}

void VKRenderGraph::deinit()
{
  command_buffer_.reset();
}

/* -------------------------------------------------------------------- */
/** \name Submit graph
 * \{ */

void VKRenderGraph::submit_for_present(VkImage vk_swapchain_image)
{
  /* Needs to be executed at forehand as `add_node` also locks the mutex. */
  VKSynchronizationCreateInfo synchronization = {};
  synchronization.vk_image = vk_swapchain_image;
  synchronization.vk_image_layout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
  add_node<VKSynchronizationNode, VKSynchronizationCreateInfo>(synchronization);

  std::scoped_lock lock(resources_.mutex_get());
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
  std::scoped_lock lock(resources_.mutex_get());
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
