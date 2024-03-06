/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 *
 * Render graph is a render solution that is able to track resource usages in a single submission
 *
 * The graph contains nodes that refers to resources it depends on, or alters.
 *
 * Resources needs to be tracked as usage can alter the content of the resource. For example an
 * image can be optimized for data transfer, or optimized for sampling which can use a different
 * pixel layout on the device.
 *
 */

#pragma once

#include <mutex>
#include <optional>

#include "BLI_map.hh"
#include "BLI_vector.hh"

#include "vk_common.hh"

#include "vk_render_graph_command_builder.hh"
#include "vk_render_graph_commands.hh"
#include "vk_render_graph_nodes.hh"
#include "vk_render_graph_resources.hh"
#include "vk_render_graph_scheduler.hh"
#include "vk_render_graph_types.hh"

namespace blender::gpu {

class VKRenderGraph {
  VKRenderGraphResources resources_;
  VKRenderGraphNodes nodes_;
  VKRenderGraphCommandBuilder command_builder_;

  std::unique_ptr<VKRenderGraphScheduler> scheduler_;
  std::unique_ptr<VKRenderGraphCommandBuffer> command_buffer_;

  /**
   * Mutex locks adding new commands to a render graph that is being submitted.
   */
  std::mutex mutex_;

 public:
  VKRenderGraph(std::unique_ptr<VKRenderGraphCommandBuffer> command_buffer,
                std::unique_ptr<VKRenderGraphScheduler> sorting_strategy);

  /**
   * Register a buffer resource to the render graph.
   */
  void add_buffer(VkBuffer vk_buffer);

  /**
   * Register an image resource to the render graph.
   */
  void add_image(VkImage vk_image, VkImageLayout vk_image_layout, ResourceOwner owner);

  void add_clear_image_node(VkImage vk_image,
                            VkClearColorValue &vk_clear_color_value,
                            VkImageSubresourceRange &vk_image_subresource_range);
  void add_fill_buffer_node(VkBuffer vk_buffer, VkDeviceSize size, uint32_t data_);
  void add_copy_buffer_node(VkBuffer src_buffer, VkBuffer dst_buffer, const VkBufferCopy &region);
  void add_dispatch_node(const VKDispatchInfo &dispatch_info);

  /**
   * Submit the commands to readback the given vk_buffer to the command queue.
   */
  void submit_buffer_for_read_back(VkBuffer vk_buffer);

  /**
   * Submit the commands to readback the given vk_image to the command queue.=
   */
  void submit_image_for_read_back(VkImage vk_image);

  /**
   * Submit the commands to present the given vk_image to the command queue.
   *
   * `vk_image` needs to be a swapchain owned image.
   */
  void submit_for_present(VkImage vk_swapchain_image);

  /**
   * Wait for the submitted work to be finished.
   */
  void finish();

 private:
  void add_resources(NodeHandle node_handle, const VKResourceAccessInfo &resources);

  friend class VKRenderGraphCommandBuilder;
  friend class Sequential;
};

}  // namespace blender::gpu
