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
  // reset image_layouts
  if (image_layouts_.size() < resource_len) {
    image_layouts_.resize(resource_len);
  }
  image_layouts_.fill(VK_IMAGE_LAYOUT_UNDEFINED);

  for (ResourceHandle image_handle : render_graph.resources_.image_resources_.values()) {
    VKRenderGraphResources::Resource &resource = render_graph.resources_.resources_.get(
        image_handle);
    image_layouts_[image_handle] = resource.vk_image_layout;
  }
}

void VKRenderGraphCommandBuilder::build_image(VKRenderGraph &render_graph, VkImage vk_image)
{
  BLI_assert(selected_nodes_.is_empty());
  render_graph.scheduler_->select_nodes_for_image(render_graph, vk_image, selected_nodes_);
  if (selected_nodes_.is_empty()) {
    return;
  }
  render_graph.command_buffer_->begin_recording();
  for (NodeHandle node_handle : selected_nodes_) {
    VKRenderGraphNodes::Node &node = render_graph.nodes_.get(node_handle);
    build_node(render_graph, node);
  }

  render_graph.command_buffer_->end_recording();
}

void VKRenderGraphCommandBuilder::build_buffer(VKRenderGraph &render_graph, VkBuffer vk_buffer)
{
  BLI_assert(selected_nodes_.is_empty());
  render_graph.scheduler_->select_nodes_for_buffer(render_graph, vk_buffer, selected_nodes_);
  if (selected_nodes_.is_empty()) {
    return;
  }
  render_graph.command_buffer_->begin_recording();
  for (NodeHandle node_handle : selected_nodes_) {
    VKRenderGraphNodes::Node &node = render_graph.nodes_.get(node_handle);
    build_node(render_graph, node);
  }

  render_graph.command_buffer_->end_recording();
}

void VKRenderGraphCommandBuilder::build_node(VKRenderGraph &render_graph,
                                             VKRenderGraphNodes::Node &node)
{
  switch (node.type) {
    case VKRenderGraphNodes::Node::Type::UNUSED: {
      break;
    }

    case VKRenderGraphNodes::Node::Type::CLEAR_COLOR_IMAGE: {
      build_node_clear_color_image(render_graph, node);
      break;
    }

    case VKRenderGraphNodes::Node::Type::FILL_BUFFER: {
      build_node_fill_buffer(render_graph, node);
      break;
    }

    default:
      BLI_assert_unreachable();
      break;
  }
}

void VKRenderGraphCommandBuilder::build_node_clear_color_image(VKRenderGraph &render_graph,
                                                               VKRenderGraphNodes::Node &node)
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
                                                         VKRenderGraphNodes::Node &node)
{
  BLI_assert(node.type == VKRenderGraphNodes::Node::Type::FILL_BUFFER);
  // TODO: add pipeline barrier for sync issues.
  render_graph.command_buffer_->fill_buffer(
      node.fill_buffer.vk_buffer, 0, node.fill_buffer.size, node.fill_buffer.data);
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
  render_graph.command_buffer_->pipeline_barrier(VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
                                                 VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                                                 0,
                                                 0,
                                                 nullptr,
                                                 0,
                                                 nullptr,
                                                 1,
                                                 &image_memory_barrier);

  image_layouts_[image_handle] = vk_image_layout;
}

}  // namespace blender::gpu
