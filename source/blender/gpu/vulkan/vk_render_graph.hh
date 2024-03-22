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
#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#include "vk_common.hh"

#include "vk_render_graph_command_builder.hh"
#include "vk_render_graph_commands.hh"
#include "vk_render_graph_nodes.hh"
#include "vk_render_graph_resources.hh"
#include "vk_render_graph_scheduler.hh"
#include "vk_render_graph_types.hh"

namespace blender::gpu {

class VKRenderGraph : public NonCopyable {
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
   * Remove a buffer resource from the render graph.
   */
  void remove_buffer(VkBuffer vk_buffer);

  /**
   * Register an image resource to the render graph.
   */
  void add_image(VkImage vk_image, VkImageLayout vk_image_layout, ResourceOwner owner);

  /**
   * Remove an image resource from the render graph.
   */
  // TODO: add test case to check if resources are reset when deleted.
  void remove_image(VkImage vk_image);

  void add_clear_image_node(VkImage vk_image,
                            VkClearColorValue &vk_clear_color_value,
                            VkImageSubresourceRange &vk_image_subresource_range);
  void add_fill_buffer_node(VkBuffer vk_buffer, VkDeviceSize size, uint32_t data_);
  void add_copy_buffer_node(VkBuffer src_buffer, VkBuffer dst_buffer, const VkBufferCopy &region);
  void add_copy_buffer_to_image_node(VkBuffer src_buffer,
                                     VkImage dst_image,
                                     const VkBufferImageCopy &region);
  void add_copy_image_node(VkImage src_image, VkImage dst_image, const VkImageCopy &region);
  void add_copy_image_to_buffer_node(VkImage src_image,
                                     VkBuffer dst_buffer,
                                     const VkBufferImageCopy &region);
  void add_ensure_image_layout_node(VkImage vk_image, VkImageLayout vk_image_layout);
  void add_dispatch_node(const VKDispatchInfo &dispatch_info);

  /**
   * Submit the commands to readback the given vk_buffer to the command queue.
   */
  void submit_buffer_for_read_back(VkBuffer vk_buffer);

  /**
   * Submit the commands to present the given vk_image to the command queue.
   *
   * `vk_image` needs to be a swapchain owned image.
   *
   * Post conditions:
   * - `vk_swapchain_image` layout is transitioned to `VK_IMAGE_LAYOUT_SRC_PRESENT`.
   */
  void submit_for_present(VkImage vk_swapchain_image);

 private:
  /**
   * Add resources to a node handle.
   *
   * Clear + Copy nodes have a straight forward resource access. Drawing and compute nodes on the
   * other hand can have complex setups where the resources can be used in different shader stages
   * and access masks. #add_resources was added for these complex setups. In stead of determining
   * how the resource is accessed these nodes will provide the essential information via the
   * provided #VKResourceAccessInfo.
   */
  void add_resources(NodeHandle node_handle, const VKResourceAccessInfo &resources);

  friend class VKRenderGraphCommandBuilder;
  friend class Sequential;
};

}  // namespace blender::gpu
