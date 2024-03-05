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

void VKRenderGraph::add_fill_buffer_node(VkBuffer vk_buffer, VkDeviceSize size, uint32_t data)
{
  std::scoped_lock lock(mutex_);
  NodeHandle handle = nodes_.add_fill_buffer_node(vk_buffer, size, data);

  VersionedResource resource = resources_.get_buffer_and_increase_version(vk_buffer);
  nodes_.add_write_resource(handle, resource);
}

void VKRenderGraph::add_copy_buffer_node(VkBuffer src_buffer,
                                         VkBuffer dst_buffer,
                                         const VkBufferCopy &region)
{
  std::scoped_lock lock(mutex_);
  NodeHandle handle = nodes_.add_copy_buffer_node(src_buffer, dst_buffer, region);

  VersionedResource src_resource = resources_.get_buffer(src_buffer);
  VersionedResource dst_resource = resources_.get_buffer_and_increase_version(dst_buffer);
  nodes_.add_read_resource(handle, src_resource);
  nodes_.add_write_resource(handle, dst_resource);
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

void VKRenderGraph::submit_buffer_for_read_back(VkBuffer vk_buffer)
{
  std::scoped_lock lock(mutex_);
  command_builder_.reset(*this);
  command_builder_.build_buffer(*this, vk_buffer);
  // submit
  // update state
  // wait
}

/** \} */

}  // namespace blender::gpu
