/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_command_builder.hh"
#include "vk_render_graph.hh"

namespace blender::gpu::render_graph {

VKCommandBuilder::VKCommandBuilder()
{
  vk_buffer_memory_barrier_ = {};
  vk_buffer_memory_barrier_.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
  vk_buffer_memory_barrier_.pNext = nullptr;
  vk_buffer_memory_barrier_.srcAccessMask = VK_ACCESS_NONE;
  vk_buffer_memory_barrier_.dstAccessMask = VK_ACCESS_NONE;
  vk_buffer_memory_barrier_.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  vk_buffer_memory_barrier_.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  vk_buffer_memory_barrier_.buffer = VK_NULL_HANDLE;
  vk_buffer_memory_barrier_.offset = 0;
  vk_buffer_memory_barrier_.size = VK_WHOLE_SIZE;

  vk_image_memory_barrier_ = {};
  vk_image_memory_barrier_.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
  vk_image_memory_barrier_.pNext = nullptr;
  vk_image_memory_barrier_.srcAccessMask = VK_ACCESS_NONE;
  vk_image_memory_barrier_.dstAccessMask = VK_ACCESS_NONE;
  vk_image_memory_barrier_.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  vk_image_memory_barrier_.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  vk_image_memory_barrier_.image = VK_NULL_HANDLE;
  vk_image_memory_barrier_.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  vk_image_memory_barrier_.newLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  vk_image_memory_barrier_.subresourceRange.aspectMask = VK_IMAGE_ASPECT_NONE;
  vk_image_memory_barrier_.subresourceRange.baseArrayLayer = 0;
  vk_image_memory_barrier_.subresourceRange.layerCount = VK_REMAINING_ARRAY_LAYERS;
  vk_image_memory_barrier_.subresourceRange.baseMipLevel = 0;
  vk_image_memory_barrier_.subresourceRange.levelCount = VK_REMAINING_MIP_LEVELS;
}

void VKCommandBuilder::reset(VKRenderGraph &render_graph)
{
  selected_nodes_.clear();
  const int64_t resource_len = render_graph.resources_.resources_.size();

  // Reset access_masks, keep last known access masks around.
  // NOTE: Access masks should be reset when resources are removed. Handles can be reused and
  // that isn't take into account.
  if (resource_states_.size() < resource_len) {
    resource_states_.resize(resource_len);
  }

  /* Swap chain images layouts needs to be reset, otherwise the last one will be used and that
   * might not be the correct one. For application owned resources should be undefined by default
   * and are tracked here.
   */
  for (ResourceHandle image_handle : render_graph.resources_.image_resources_.values()) {
    VKResources::Resource &resource = render_graph.resources_.resources_.get(image_handle);
    if (resource.owner == ResourceOwner::SWAP_CHAIN) {
      resource_states_[image_handle].image_layout = resource.vk_image_layout;
    }
  }

  // Reset pipelines
  active_pipelines = {};
}

void VKCommandBuilder::remove_resource(ResourceHandle handle)
{
  if (resource_states_.size() >= handle) {
    resource_states_[handle] = {};
  }
}

void VKCommandBuilder::build_image(VKRenderGraph &render_graph, VkImage vk_image)
{
  BLI_assert(selected_nodes_.is_empty());
  render_graph.scheduler_->select_nodes_for_image(render_graph, vk_image, selected_nodes_);
  if (selected_nodes_.is_empty()) {
    return;
  }
  for (NodeHandle node_handle : selected_nodes_) {
    VKNodes::Node &node = render_graph.nodes_.get(node_handle);
    build_node(render_graph, node_handle, node);
  }
}

void VKCommandBuilder::build_buffer(VKRenderGraph &render_graph, VkBuffer vk_buffer)
{
  BLI_assert(selected_nodes_.is_empty());
  render_graph.scheduler_->select_nodes_for_buffer(render_graph, vk_buffer, selected_nodes_);
  if (selected_nodes_.is_empty()) {
    return;
  }
  for (NodeHandle node_handle : selected_nodes_) {
    VKNodes::Node &node = render_graph.nodes_.get(node_handle);
    build_node(render_graph, node_handle, node);
  }
}

void VKCommandBuilder::build_node(VKRenderGraph &render_graph,
                                  NodeHandle node_handle,
                                  const VKNodes::Node &node)
{
  switch (node.type) {
    case VKNodeType::UNUSED: {
      break;
    }

    case VKNodeType::CLEAR_COLOR_IMAGE: {
      build_node<VKClearColorImageNode, VKClearColorImageNode::Data>(
          render_graph, *render_graph.command_buffer_, node_handle, node.clear_color_image);
      break;
    }

    case VKNodeType::FILL_BUFFER: {
      build_node<VKFillBufferNode, VKFillBufferNode::Data>(
          render_graph, *render_graph.command_buffer_, node_handle, node.fill_buffer);
      break;
    }

    case VKNodeType::COPY_BUFFER: {
      build_node<VKCopyBufferNode, VKCopyBufferNode::Data>(
          render_graph, *render_graph.command_buffer_, node_handle, node.copy_buffer);
      break;
    }

    case VKNodeType::COPY_BUFFER_TO_IMAGE: {
      build_node<VKCopyBufferToImageNode, VKCopyBufferToImageNode::Data>(
          render_graph, *render_graph.command_buffer_, node_handle, node.copy_buffer_to_image);
      break;
    }

    case VKNodeType::COPY_IMAGE: {
      build_node<VKCopyImageNode, VKCopyImageNode::Data>(
          render_graph, *render_graph.command_buffer_, node_handle, node.copy_image);
      break;
    }

    case VKNodeType::COPY_IMAGE_TO_BUFFER: {
      build_node<VKCopyImageToBufferNode, VKCopyImageToBufferNode::Data>(
          render_graph, *render_graph.command_buffer_, node_handle, node.copy_image_to_buffer);
      break;
    }

    case VKNodeType::BLIT_IMAGE: {
      build_node<VKBlitImageNode, VKBlitImageNode::Data>(
          render_graph, *render_graph.command_buffer_, node_handle, node.blit_image);
      break;
    }

    case VKNodeType::SYNCHRONIZATION: {
      build_node<VKSynchronizationNode, VKSynchronizationNode::Data>(
          render_graph, *render_graph.command_buffer_, node_handle, node.synchronization);
      break;
    }

    case VKNodeType::DISPATCH: {
      build_node<VKDispatchNode, VKDispatchNode::Data>(render_graph,
                                                       *render_graph.command_buffer_,
                                                       node_handle,
                                                       node.dispatch,
                                                       active_pipelines);
      break;
    }
  }
}

/* -------------------------------------------------------------------- */
/** \name Pipeline barriers
 * \{ */

void VKCommandBuilder::reset_barriers()
{
  vk_buffer_memory_barriers_.clear();
  vk_image_memory_barriers_.clear();
  src_stage_mask_ = VK_PIPELINE_STAGE_NONE;
  dst_stage_mask_ = VK_PIPELINE_STAGE_NONE;
}

void VKCommandBuilder::send_pipeline_barriers(VKRenderGraph &render_graph)
{
  if (vk_image_memory_barriers_.is_empty() && vk_buffer_memory_barriers_.is_empty()) {
    reset_barriers();
    return;
  }

  /* When no resources have been used, we can start the barrier at the top of the pipeline.
   * It is not allowed to set it to None. */
  // TODO: VK_KHR_synchronization2 allows setting src_stage_mask_ to NONE.
  if (src_stage_mask_ == VK_PIPELINE_STAGE_NONE) {
    src_stage_mask_ = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
  }

  render_graph.command_buffer_->pipeline_barrier(src_stage_mask_,
                                                 dst_stage_mask_,
                                                 VK_DEPENDENCY_BY_REGION_BIT,
                                                 0,
                                                 nullptr,
                                                 vk_buffer_memory_barriers_.size(),
                                                 vk_buffer_memory_barriers_.data(),
                                                 vk_image_memory_barriers_.size(),
                                                 vk_image_memory_barriers_.data());
  reset_barriers();
}

void VKCommandBuilder::add_buffer_barriers(VKRenderGraph &render_graph,
                                           NodeHandle node_handle,
                                           VkPipelineStageFlags node_stages)
{
  add_buffer_read_barriers(render_graph, node_handle, node_stages);
  add_buffer_write_barriers(render_graph, node_handle, node_stages);
}

void VKCommandBuilder::add_buffer_read_barriers(VKRenderGraph &render_graph,
                                                NodeHandle node_handle,
                                                VkPipelineStageFlags node_stages)
{
  for (const VKResourceDependencies::ResourceUsage &usage :
       render_graph.resource_dependencies_.get_read_resources(node_handle))
  {
    const VersionedResource &versioned_resource = usage.resource;
    const VKResources::Resource &resource = render_graph.resources_.resources_.get(
        versioned_resource.handle);
    if (resource.vk_buffer == VK_NULL_HANDLE) {
      /* Ignore image resources. */
      continue;
    }
    VKResourceBarrierState &resource_state = resource_states_[versioned_resource.handle];
    // If resource version has not be read, no barrier needs to be added.

    VkAccessFlags read_access = resource_state.read_access;
    VkAccessFlags write_access = resource_state.write_access;
    VkAccessFlags wait_access = VK_ACCESS_NONE;

    if (read_access == (read_access | usage.vk_access_flags)) {
      // has already been covered in a previous call no need to add this one.
      // TODO: we should merge all read accesses with the first available read barrier.
      continue;
    }

    read_access |= usage.vk_access_flags;
    wait_access |= write_access;
    src_stage_mask_ |= resource_state.write_stages;
    dst_stage_mask_ |= node_stages;

    resource_state.read_access = read_access;
    resource_state.write_access = VK_ACCESS_NONE;
    resource_state.read_stages |= node_stages;
    resource_state.write_stages = VK_PIPELINE_STAGE_NONE;

    add_buffer_barrier(resource.vk_buffer, wait_access, read_access);
  }
}

void VKCommandBuilder::add_buffer_write_barriers(VKRenderGraph &render_graph,
                                                 NodeHandle node_handle,
                                                 VkPipelineStageFlags node_stages)
{
  for (const VKResourceDependencies::ResourceUsage usage :
       render_graph.resource_dependencies_.get_write_resources(node_handle))
  {
    const VersionedResource &versioned_resource = usage.resource;
    const VKResources::Resource &resource = render_graph.resources_.resources_.get(
        versioned_resource.handle);
    if (resource.vk_buffer == VK_NULL_HANDLE) {
      /* Ignore image resources. */
      continue;
    }
    VKResourceBarrierState &resource_state = resource_states_[versioned_resource.handle];
    // If resource version has not be read, no barrier needs to be added.
    VkAccessFlags read_access = resource_state.read_access;
    VkAccessFlags write_access = resource_state.write_access;
    VkAccessFlags wait_access = VK_ACCESS_NONE;

    if (read_access != VK_ACCESS_NONE) {
      wait_access |= read_access;
    }
    if (read_access == VK_ACCESS_NONE && write_access != VK_ACCESS_NONE) {
      // Add write_write_buffer_barrier
      wait_access |= write_access;
    }

    src_stage_mask_ |= resource_state.read_stages | resource_state.write_stages;
    dst_stage_mask_ |= node_stages;

    resource_state.read_access = VK_ACCESS_NONE;
    resource_state.write_access = usage.vk_access_flags;
    resource_state.read_stages = VK_PIPELINE_STAGE_NONE;
    resource_state.write_stages = node_stages;

    if (wait_access != VK_ACCESS_NONE) {
      add_buffer_barrier(resource.vk_buffer, wait_access, usage.vk_access_flags);
    }
  }
}

void VKCommandBuilder::add_buffer_barrier(VkBuffer vk_buffer,
                                          VkAccessFlags src_access_mask,
                                          VkAccessFlags dst_access_mask)
{
  vk_buffer_memory_barrier_.srcAccessMask = src_access_mask;
  vk_buffer_memory_barrier_.dstAccessMask = dst_access_mask;
  vk_buffer_memory_barrier_.buffer = vk_buffer;
  vk_buffer_memory_barriers_.append(vk_buffer_memory_barrier_);
  vk_buffer_memory_barrier_.srcAccessMask = VK_ACCESS_NONE;
  vk_buffer_memory_barrier_.dstAccessMask = VK_ACCESS_NONE;
  vk_buffer_memory_barrier_.buffer = VK_NULL_HANDLE;
}

void VKCommandBuilder::add_image_barriers(VKRenderGraph &render_graph,
                                          NodeHandle node_handle,
                                          VkPipelineStageFlags node_stages)
{
  add_image_read_barriers(render_graph, node_handle, node_stages);
  add_image_write_barriers(render_graph, node_handle, node_stages);
}

void VKCommandBuilder::add_image_read_barriers(VKRenderGraph &render_graph,
                                               NodeHandle node_handle,
                                               VkPipelineStageFlags node_stages)
{
  for (const VKResourceDependencies::ResourceUsage &usage :
       render_graph.resource_dependencies_.get_read_resources(node_handle))
  {
    const VersionedResource &versioned_resource = usage.resource;
    const VKResources::Resource &resource = render_graph.resources_.resources_.get(
        versioned_resource.handle);
    if (resource.vk_image == VK_NULL_HANDLE) {
      /* Ignore buffer resources. */
      continue;
    }
    VKResourceBarrierState &resource_state = resource_states_[versioned_resource.handle];
    // If resource version has not be read, no barrier needs to be added.

    VkAccessFlags read_access = resource_state.read_access;
    VkAccessFlags write_access = resource_state.write_access;
    VkAccessFlags wait_access = VK_ACCESS_NONE;

    if (read_access == (read_access | usage.vk_access_flags) &&
        resource_state.image_layout == usage.vk_image_layout)
    {
      // has already been covered in a previous call no need to add this one.
      // TODO: we should merge all read accesses with the first available read barrier.
      continue;
    }

    read_access |= usage.vk_access_flags;
    wait_access |= write_access;
    src_stage_mask_ |= resource_state.write_stages;
    dst_stage_mask_ |= node_stages;

    resource_state.read_access = read_access;
    resource_state.write_access = VK_ACCESS_NONE;
    resource_state.read_stages |= node_stages;
    resource_state.write_stages = VK_PIPELINE_STAGE_NONE;

    add_image_barrier(resource.vk_image,
                      wait_access,
                      read_access,
                      resource_state.image_layout,
                      usage.vk_image_layout);
    resource_state.image_layout = usage.vk_image_layout;
  }
}

void VKCommandBuilder::add_image_write_barriers(VKRenderGraph &render_graph,
                                                NodeHandle node_handle,
                                                VkPipelineStageFlags node_stages)
{
  for (const VKResourceDependencies::ResourceUsage usage :
       render_graph.resource_dependencies_.get_write_resources(node_handle))
  {
    const VersionedResource &versioned_resource = usage.resource;
    const VKResources::Resource &resource = render_graph.resources_.resources_.get(
        versioned_resource.handle);
    if (resource.vk_image == VK_NULL_HANDLE) {
      /* Ignore image resources. */
      continue;
    }
    VKResourceBarrierState &resource_state = resource_states_[versioned_resource.handle];
    // If resource version has not be read, no barrier needs to be added.
    VkAccessFlags read_access = resource_state.read_access;
    VkAccessFlags write_access = resource_state.write_access;
    VkAccessFlags wait_access = VK_ACCESS_NONE;

    if (read_access != VK_ACCESS_NONE) {
      wait_access |= read_access;
    }
    if (read_access == VK_ACCESS_NONE && write_access != VK_ACCESS_NONE) {
      // Add write_write_buffer_barrier
      wait_access |= write_access;
    }

    src_stage_mask_ |= resource_state.read_stages | resource_state.write_stages;
    dst_stage_mask_ |= node_stages;

    resource_state.read_access = VK_ACCESS_NONE;
    resource_state.write_access = usage.vk_access_flags;
    resource_state.read_stages = VK_PIPELINE_STAGE_NONE;
    resource_state.write_stages = node_stages;

    if (wait_access != VK_ACCESS_NONE || usage.vk_image_layout != resource_state.image_layout) {
      add_image_barrier(resource.vk_image,
                        wait_access,
                        usage.vk_access_flags,
                        resource_state.image_layout,
                        usage.vk_image_layout);
      resource_state.image_layout = usage.vk_image_layout;
    }
  }
}

void VKCommandBuilder::add_image_barrier(VkImage vk_image,
                                         VkAccessFlags src_access_mask,
                                         VkAccessFlags dst_access_mask,
                                         VkImageLayout old_layout,
                                         VkImageLayout new_layout)
{
  vk_image_memory_barrier_.srcAccessMask = src_access_mask;
  vk_image_memory_barrier_.dstAccessMask = dst_access_mask;
  vk_image_memory_barrier_.image = vk_image;
  vk_image_memory_barrier_.oldLayout = old_layout;
  vk_image_memory_barrier_.newLayout = new_layout;
  /* TODO: determine the correct aspect bits. */
  vk_image_memory_barrier_.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  vk_image_memory_barriers_.append(vk_image_memory_barrier_);
  vk_image_memory_barrier_.srcAccessMask = VK_ACCESS_NONE;
  vk_image_memory_barrier_.dstAccessMask = VK_ACCESS_NONE;
  vk_image_memory_barrier_.image = VK_NULL_HANDLE;
  vk_image_memory_barrier_.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  vk_image_memory_barrier_.newLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  vk_image_memory_barrier_.subresourceRange.aspectMask = VK_IMAGE_ASPECT_NONE;
}

/** \} */

void VKCommandBuilder::update_state_after_submission(VKRenderGraph &render_graph)
{
  render_graph.resource_dependencies_.remove_nodes(selected_nodes_);
  render_graph.nodes_.remove_nodes(selected_nodes_);
}

}  // namespace blender::gpu::render_graph
