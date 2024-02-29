/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph.hh"

namespace blender::gpu {

VKRenderGraph::VKRenderGraph(std::unique_ptr<VKRenderGraphCommandBuffer> command_buffer,
                             std::unique_ptr<VKRenderGraphScheduler> scheduler)
    : scheduler_(std::move(scheduler)), command_buffer_(std::move(command_buffer))
{
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
/** \name Add Node
 * \{ */

void VKRenderGraph::add_clear_image_node(VkImage vk_image,
                                         VkClearColorValue &vk_clear_color_value,
                                         VkImageSubresourceRange &vk_image_subresource_range)
{
  std::scoped_lock lock(mutex_);
  NodeHandle handle = nodes_.add_clear_image_node(
      vk_image, vk_clear_color_value, vk_image_subresource_range);
  VersionedResource resource = resources_.get_image_and_increase_version(vk_image);
  nodes_.add_write_resource(handle, resource);
}

/* -------------------------------------------------------------------- */
/** \name Submit graph
 * \{ */

void VKRenderGraph::submit_for_present(VkImage vk_swapchain_image)
{
  std::scoped_lock lock(mutex_);
  command_builder_.reset(*this);
  command_builder_.build_image(*this, vk_swapchain_image);
  command_builder_.ensure_image_layout(*this, vk_swapchain_image, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
  // submit
  // update state
}

/** \} */

}  // namespace blender::gpu
