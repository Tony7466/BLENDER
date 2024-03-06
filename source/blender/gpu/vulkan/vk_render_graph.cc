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
  nodes_.add_write_resource(handle, resource, VK_ACCESS_TRANSFER_WRITE_BIT);
}

void VKRenderGraph::add_fill_buffer_node(VkBuffer vk_buffer, VkDeviceSize size, uint32_t data)
{
  std::scoped_lock lock(mutex_);
  NodeHandle handle = nodes_.add_fill_buffer_node(vk_buffer, size, data);

  VersionedResource resource = resources_.get_buffer_and_increase_version(vk_buffer);
  nodes_.add_write_resource(handle, resource, VK_ACCESS_TRANSFER_WRITE_BIT);
}

void VKRenderGraph::add_copy_buffer_node(VkBuffer src_buffer,
                                         VkBuffer dst_buffer,
                                         const VkBufferCopy &region)
{
  std::scoped_lock lock(mutex_);
  NodeHandle handle = nodes_.add_copy_buffer_node(src_buffer, dst_buffer, region);

  VersionedResource src_resource = resources_.get_buffer(src_buffer);
  VersionedResource dst_resource = resources_.get_buffer_and_increase_version(dst_buffer);
  nodes_.add_read_resource(handle, src_resource, VK_ACCESS_TRANSFER_READ_BIT);
  nodes_.add_write_resource(handle, dst_resource, VK_ACCESS_TRANSFER_WRITE_BIT);
}

void VKRenderGraph::add_dispatch_node(const VKDispatchInfo &dispatch_info)
{
  std::scoped_lock lock(mutex_);
  NodeHandle handle = nodes_.add_dispatch_node(dispatch_info);
  add_resources(handle, dispatch_info.resources);
  // TODO: we should add descriptor sets and push constants as well.
}

void VKRenderGraph::add_resources(NodeHandle handle, const VKResourceAccessInfo &resources)
{
  // TODO: validate. resources should be unique (merged).
  for (const VKBufferAccess &buffer_access : resources.buffers) {
    VkAccessFlags read_access = buffer_access.vk_access_flags & VK_ACCESS_READ_MASK;
    if (read_access != VK_ACCESS_NONE) {
      VersionedResource versioned_resource = resources_.get_buffer(buffer_access.vk_buffer);
      nodes_.add_read_resource(handle, versioned_resource, read_access);
    }

    VkAccessFlags write_access = buffer_access.vk_access_flags & VK_ACCESS_WRITE_MASK;
    if (write_access != VK_ACCESS_NONE) {
      VersionedResource versioned_resource = resources_.get_buffer_and_increase_version(
          buffer_access.vk_buffer);
      nodes_.add_write_resource(handle, versioned_resource, write_access);
    }
  }

  for (const VKImageAccess &image_access : resources.images) {
    VkAccessFlags read_access = image_access.vk_access_flags & VK_ACCESS_READ_MASK;
    if (read_access != VK_ACCESS_NONE) {
      VersionedResource versioned_resource = resources_.get_image(image_access.vk_image);
      nodes_.add_read_resource(handle, versioned_resource, read_access);
    }

    VkAccessFlags write_access = image_access.vk_access_flags & VK_ACCESS_WRITE_MASK;
    if (write_access != VK_ACCESS_NONE) {
      VersionedResource versioned_resource = resources_.get_image_and_increase_version(
          image_access.vk_image);
      nodes_.add_write_resource(handle, versioned_resource, write_access);
    }
  }
}

/* -------------------------------------------------------------------- */
/** \name Submit graph
 * \{ */

void VKRenderGraph::submit_for_present(VkImage vk_swapchain_image)
{
  std::scoped_lock lock(mutex_);
  command_builder_.reset(*this);
  command_buffer_->begin_recording();
  command_builder_.build_image(*this, vk_swapchain_image);
  command_builder_.ensure_image_layout(*this, vk_swapchain_image, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
  command_buffer_->end_recording();
  /* TODO: Can we implement gpu synchronization when presenting? */
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

}  // namespace blender::gpu
