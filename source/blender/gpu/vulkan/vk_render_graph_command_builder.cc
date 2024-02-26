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
  render_graph.sorting_strategy_->select_nodes_for_image(vk_image, selected_nodes_);
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
