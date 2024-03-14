/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph_command_builder.hh"
#include "vk_render_graph.hh"

namespace blender::gpu {
void VKRenderGraphCommandBuilder::reset(VKRenderGraph &render_graph)
{
  selected_nodes_.clear();
  const int64_t resource_len = render_graph.resources_.resources_.size();

  // Reset access_masks, keep last known access masks around.
  // NOTE: Access masks should be resetted when resources are removed. Handles can be reused and
  // that isn't take into account.
  if (read_access_.size() < resource_len) {
    read_access_.resize(resource_len, VK_ACCESS_NONE);
  }
  if (write_access_.size() < resource_len) {
    write_access_.resize(resource_len, VK_ACCESS_NONE);
  }
  if (read_stages_.size() < resource_len) {
    read_stages_.resize(resource_len, VK_PIPELINE_STAGE_NONE);
  }
  if (write_stages_.size() < resource_len) {
    write_stages_.resize(resource_len, VK_PIPELINE_STAGE_NONE);
  }

  // Reset image_layouts
  if (image_layouts_.size() < resource_len) {
    image_layouts_.resize(resource_len);
  }
  image_layouts_.fill(VK_IMAGE_LAYOUT_UNDEFINED);

  /* Swap chain images layouts needs to be reset, otherwise the last one will be used and that
   * might not be the correct one. For application owned resources should be undefined by default
   * and are tracked here.
   */
  for (ResourceHandle image_handle : render_graph.resources_.image_resources_.values()) {
    VKRenderGraphResources::Resource &resource = render_graph.resources_.resources_.get(
        image_handle);
    if (resource.owner == ResourceOwner::SWAP_CHAIN) {
      image_layouts_[image_handle] = resource.vk_image_layout;
    }
  }

  // Reset pipelines
  active_compute_pipeline_ = VK_NULL_HANDLE;
  active_compute_descriptor_set_ = VK_NULL_HANDLE;
}

void VKRenderGraphCommandBuilder::remove_resource(ResourceHandle handle)
{
  if (read_access_.size() >= handle) {
    read_access_[handle] = VK_ACCESS_NONE;
  }
  if (write_access_.size() >= handle) {
    write_access_[handle] = VK_ACCESS_NONE;
  }
  if (read_stages_.size() >= handle) {
    read_stages_[handle] = VK_PIPELINE_STAGE_NONE;
  }
  if (write_stages_.size() >= handle) {
    write_stages_[handle] = VK_PIPELINE_STAGE_NONE;
  }
  if (image_layouts_.size() >= handle) {
    image_layouts_[handle] = VK_IMAGE_LAYOUT_UNDEFINED;
  }
}

void VKRenderGraphCommandBuilder::build_image(VKRenderGraph &render_graph, VkImage vk_image)
{
  BLI_assert(selected_nodes_.is_empty());
  render_graph.scheduler_->select_nodes_for_image(render_graph, vk_image, selected_nodes_);
  if (selected_nodes_.is_empty()) {
    return;
  }
  for (NodeHandle node_handle : selected_nodes_) {
    VKRenderGraphNodes::Node &node = render_graph.nodes_.get(node_handle);
    build_node(render_graph, node_handle, node);
  }
}

void VKRenderGraphCommandBuilder::build_buffer(VKRenderGraph &render_graph, VkBuffer vk_buffer)
{
  BLI_assert(selected_nodes_.is_empty());
  render_graph.scheduler_->select_nodes_for_buffer(render_graph, vk_buffer, selected_nodes_);
  if (selected_nodes_.is_empty()) {
    return;
  }
  for (NodeHandle node_handle : selected_nodes_) {
    VKRenderGraphNodes::Node &node = render_graph.nodes_.get(node_handle);
    build_node(render_graph, node_handle, node);
  }
}

void VKRenderGraphCommandBuilder::build_node(VKRenderGraph &render_graph,
                                             NodeHandle node_handle,
                                             const VKRenderGraphNodes::Node &node)
{
  switch (node.type) {
    case VKRenderGraphNodes::Node::Type::UNUSED: {
      break;
    }

    case VKRenderGraphNodes::Node::Type::CLEAR_COLOR_IMAGE: {
      build_node_clear_color_image(render_graph, node_handle, node);
      break;
    }

    case VKRenderGraphNodes::Node::Type::FILL_BUFFER: {
      build_node_fill_buffer(render_graph, node_handle, node);
      break;
    }

    case VKRenderGraphNodes::Node::Type::COPY_BUFFER: {
      build_node_copy_buffer(render_graph, node_handle, node);
      break;
    }

    case VKRenderGraphNodes::Node::Type::DISPATCH: {
      build_node_dispatch(render_graph, node_handle, node);
      break;
    }

    default:
      BLI_assert_unreachable();
      break;
  }
}

void VKRenderGraphCommandBuilder::build_node_clear_color_image(
    VKRenderGraph &render_graph, NodeHandle node_handle, const VKRenderGraphNodes::Node &node)
{
  BLI_assert(node.type == VKRenderGraphNodes::Node::Type::CLEAR_COLOR_IMAGE);
  const VkImageLayout vk_image_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
  ensure_image_layout(render_graph, node.clear_color_image.vk_image, vk_image_layout);

  render_graph.command_buffer_->clear_color_image(
      node.clear_color_image.vk_image,
      vk_image_layout,
      &node.clear_color_image.vk_clear_color_value,
      1,
      &node.clear_color_image.vk_image_subresource_range);
}

void VKRenderGraphCommandBuilder::build_node_fill_buffer(VKRenderGraph &render_graph,
                                                         NodeHandle node_handle,
                                                         const VKRenderGraphNodes::Node &node)
{
  BLI_assert(node.type == VKRenderGraphNodes::Node::Type::FILL_BUFFER);
  add_buffer_barriers(render_graph, node_handle, VK_PIPELINE_STAGE_TRANSFER_BIT);
  render_graph.command_buffer_->fill_buffer(
      node.fill_buffer.vk_buffer, 0, node.fill_buffer.size, node.fill_buffer.data);
}

void VKRenderGraphCommandBuilder::build_node_copy_buffer(VKRenderGraph &render_graph,
                                                         NodeHandle node_handle,
                                                         const VKRenderGraphNodes::Node &node)
{
  BLI_assert(node.type == VKRenderGraphNodes::Node::Type::COPY_BUFFER);
  add_buffer_barriers(render_graph, node_handle, VK_PIPELINE_STAGE_TRANSFER_BIT);
  render_graph.command_buffer_->copy_buffer(
      node.copy_buffer.src_buffer, node.copy_buffer.dst_buffer, 1, &node.copy_buffer.region);
}

void VKRenderGraphCommandBuilder::build_node_dispatch(VKRenderGraph &render_graph,
                                                      NodeHandle node_handle,
                                                      const VKRenderGraphNodes::Node &node)
{
  BLI_assert(node.type == VKRenderGraphNodes::Node::Type::DISPATCH);
  add_buffer_barriers(render_graph, node_handle, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

  if (assign_if_different(active_compute_pipeline_, node.dispatch.vk_pipeline)) {
    render_graph.command_buffer_->bind_pipeline(VK_PIPELINE_BIND_POINT_COMPUTE,
                                                active_compute_pipeline_);
  }

  if (assign_if_different(active_compute_descriptor_set_,
                          node.dispatch.descriptor_set.vk_descriptor_set))
  {
    render_graph.command_buffer_->bind_descriptor_sets(
        VK_PIPELINE_BIND_POINT_COMPUTE,
        node.dispatch.descriptor_set.vk_pipeline_layout,
        0,
        1,
        &node.dispatch.descriptor_set.vk_descriptor_set,
        0,
        nullptr);
  }

  // TODO: Check changes in push constants.
  render_graph.command_buffer_->dispatch(
      node.dispatch.group_count_x, node.dispatch.group_count_y, node.dispatch.group_count_z);
}

void VKRenderGraphCommandBuilder::ensure_image_layout(VKRenderGraph &render_graph,
                                                      VkImage vk_image,
                                                      VkImageLayout vk_image_layout)
{
  ResourceHandle image_handle = render_graph.resources_.get_image_handle(vk_image);
  VkImageLayout current_layout = image_layouts_[image_handle];
  if (current_layout == vk_image_layout) {
    return;
  }

  // Create and record barrier. (note is incomplete)
  VkImageMemoryBarrier image_memory_barrier = {};
  image_memory_barrier.image = vk_image;
  image_memory_barrier.oldLayout = current_layout;
  image_memory_barrier.newLayout = vk_image_layout;
  // TODO: should get the src stage flags from usage
  render_graph.command_buffer_->pipeline_barrier(VK_PIPELINE_STAGE_NONE_KHR,
                                                 VK_PIPELINE_STAGE_NONE_KHR,
                                                 0,
                                                 0,
                                                 nullptr,
                                                 0,
                                                 nullptr,
                                                 1,
                                                 &image_memory_barrier);

  image_layouts_[image_handle] = vk_image_layout;
}

void VKRenderGraphCommandBuilder::add_buffer_barriers(VKRenderGraph &render_graph,
                                                      NodeHandle node_handle,
                                                      VkPipelineStageFlags node_stages)
{
  vk_buffer_memory_barriers_.clear();
  VkPipelineStageFlags src_stage_mask = VK_PIPELINE_STAGE_NONE;
  VkPipelineStageFlags dst_stage_mask = VK_PIPELINE_STAGE_NONE;
  add_buffer_read_barriers(render_graph, node_handle, node_stages, src_stage_mask, dst_stage_mask);
  add_buffer_write_barriers(
      render_graph, node_handle, node_stages, src_stage_mask, dst_stage_mask);
  if (vk_buffer_memory_barriers_.is_empty() || src_stage_mask == VK_PIPELINE_STAGE_NONE ||
      dst_stage_mask == VK_PIPELINE_STAGE_NONE)
  {
    return;
  }

  render_graph.command_buffer_->pipeline_barrier(src_stage_mask,
                                                 dst_stage_mask,
                                                 VK_DEPENDENCY_BY_REGION_BIT,
                                                 0,
                                                 nullptr,
                                                 vk_buffer_memory_barriers_.size(),
                                                 vk_buffer_memory_barriers_.data(),
                                                 0,
                                                 nullptr);
}

void VKRenderGraphCommandBuilder::add_buffer_read_barriers(VKRenderGraph &render_graph,
                                                           NodeHandle node_handle,
                                                           VkPipelineStageFlags node_stages,
                                                           VkPipelineStageFlags &r_src_stage_mask,
                                                           VkPipelineStageFlags &r_dst_stage_mask)
{
  for (const VKRenderGraphNodes::ResourceUsage &usage :
       render_graph.nodes_.get_read_resources(node_handle))
  {
    const VersionedResource &versioned_resource = usage.resource;
    const VKRenderGraphResources::Resource &resource = render_graph.resources_.resources_.get(
        versioned_resource.handle);
    // If resource version has not be read, no barrier needs to be added.
    VkAccessFlags read_access = read_access_[versioned_resource.handle];
    VkAccessFlags write_access = write_access_[versioned_resource.handle];
    VkAccessFlags wait_access = VK_ACCESS_NONE;

    if (read_access == (read_access | usage.vk_access_flags)) {
      // has already been covered in a previous call no need to add this one.
      // TODO: we should merge all read accesses with the first available read barrier.
      continue;
    }

    read_access |= usage.vk_access_flags;
    wait_access |= write_access;
    r_src_stage_mask |= write_stages_[versioned_resource.handle];
    r_dst_stage_mask |= node_stages;

    read_access_[versioned_resource.handle] = read_access;
    write_access_[versioned_resource.handle] = VK_ACCESS_NONE;
    read_stages_[versioned_resource.handle] |= node_stages;
    write_stages_[versioned_resource.handle] = VK_PIPELINE_STAGE_NONE;

    add_buffer_barrier(resource.vk_buffer, wait_access, read_access);
  }
}

void VKRenderGraphCommandBuilder::add_buffer_write_barriers(VKRenderGraph &render_graph,
                                                            NodeHandle node_handle,
                                                            VkPipelineStageFlags node_stages,
                                                            VkPipelineStageFlags &r_src_stage_mask,
                                                            VkPipelineStageFlags &r_dst_stage_mask)
{
  for (const VKRenderGraphNodes::ResourceUsage usage :
       render_graph.nodes_.get_write_resources(node_handle))
  {
    const VersionedResource &versioned_resource = usage.resource;
    const VKRenderGraphResources::Resource &resource = render_graph.resources_.resources_.get(
        versioned_resource.handle);
    if (resource.vk_buffer == VK_NULL_HANDLE) {
      /* Ignore image resources. */
      continue;
    }
    // If resource version has not be read, no barrier needs to be added.
    VkAccessFlags read_access = read_access_[versioned_resource.handle];
    VkAccessFlags write_access = write_access_[versioned_resource.handle];
    VkAccessFlags wait_access = VK_ACCESS_NONE;

    if (read_access != VK_ACCESS_NONE) {
      wait_access |= read_access;
    }
    if (read_access == VK_ACCESS_NONE && write_access != VK_ACCESS_NONE) {
      // Add write_write_buffer_barrier
      wait_access |= write_access;
    }

    r_src_stage_mask |= read_stages_[versioned_resource.handle] |
                        write_stages_[versioned_resource.handle];
    r_dst_stage_mask |= node_stages;

    read_access_[versioned_resource.handle] = VK_ACCESS_NONE;
    write_access_[versioned_resource.handle] = usage.vk_access_flags;
    read_stages_[versioned_resource.handle] = VK_PIPELINE_STAGE_NONE;
    write_stages_[versioned_resource.handle] = node_stages;

    if (wait_access != VK_ACCESS_NONE) {
      add_buffer_barrier(resource.vk_buffer, wait_access, usage.vk_access_flags);
    }
  }
}

void VKRenderGraphCommandBuilder::add_buffer_barrier(VkBuffer vk_buffer,
                                                     VkAccessFlags src_access_mask,
                                                     VkAccessFlags dst_access_mask)
{
  VkBufferMemoryBarrier vk_buffer_memory_barrier = {};
  vk_buffer_memory_barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
  vk_buffer_memory_barrier.pNext = nullptr;
  vk_buffer_memory_barrier.srcAccessMask = src_access_mask;
  vk_buffer_memory_barrier.dstAccessMask = dst_access_mask;
  vk_buffer_memory_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  vk_buffer_memory_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  vk_buffer_memory_barrier.buffer = vk_buffer;
  vk_buffer_memory_barrier.offset = 0;
  vk_buffer_memory_barrier.size = VK_WHOLE_SIZE;

  vk_buffer_memory_barriers_.append(vk_buffer_memory_barrier);
}

void VKRenderGraphCommandBuilder::update_state_after_submission(VKRenderGraph &render_graph)
{
  render_graph.nodes_.remove_nodes(selected_nodes_);
}

}  // namespace blender::gpu
