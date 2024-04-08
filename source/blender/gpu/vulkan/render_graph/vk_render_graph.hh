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

#include "vk_command_buffer_wrapper.hh"
#include "vk_command_builder.hh"
#include "vk_nodes.hh"
#include "vk_resource_dependencies.hh"
#include "vk_resources.hh"
#include "vk_scheduler.hh"
#include "vk_types.hh"

namespace blender::gpu::render_graph {

class VKRenderGraph : public NonCopyable {
  VKResources resources_;
  VKResourceDependencies resource_dependencies_;
  VKNodes nodes_;
  VKCommandBuilder command_builder_;

  std::unique_ptr<VKScheduler> scheduler_;
  std::unique_ptr<VKCommandBufferInterface> command_buffer_;

  /**
   * Mutex locks adding new commands to a render graph that is being submitted.
   */
  std::mutex mutex_;

 public:
  VKRenderGraph(std::unique_ptr<VKCommandBufferInterface> command_buffer,
                std::unique_ptr<VKScheduler> sorting_strategy);

  /** Free all resources held by the render graph. */
  void deinit();

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

 private:
  /**
   * Add a node to the render graph.
   */
  template<typename NodeClass, typename NodeClassData>
  void add_node(const NodeClassData &node_data)
  {
    std::scoped_lock lock(mutex_);
    NodeHandle handle = nodes_.add_node<NodeClass, NodeClassData>(node_data);
    NodeClass::build_resource_dependencies(resources_, resource_dependencies_, handle, node_data);
  }

 public:
  void add_node(const VKClearColorImageNode::Data &clear_color_image)
  {
    add_node<VKClearColorImageNode, VKClearColorImageNode::Data>(clear_color_image);
  }
  void add_node(const VKFillBufferNode::Data &fill_buffer)
  {
    add_node<VKFillBufferNode, VKFillBufferNode::Data>(fill_buffer);
  }
  void add_node(const VKCopyBufferNode::Data &copy_buffer)
  {
    add_node<VKCopyBufferNode, VKCopyBufferNode::Data>(copy_buffer);
  }
  void add_node(const VKCopyBufferToImageNode::Data &copy_buffer_to_image)
  {
    add_node<VKCopyBufferToImageNode, VKCopyBufferToImageNode::Data>(copy_buffer_to_image);
  }
  void add_node(const VKCopyImageNode::Data &copy_image_to_buffer)
  {
    add_node<VKCopyImageNode, VKCopyImageNode::Data>(copy_image_to_buffer);
  }
  void add_node(const VKCopyImageToBufferNode::Data &copy_image_to_buffer)
  {
    add_node<VKCopyImageToBufferNode, VKCopyImageToBufferNode::Data>(copy_image_to_buffer);
  }
  void add_node(const VKBlitImageNode::Data &blit_image)
  {
    add_node<VKBlitImageNode, VKBlitImageNode::Data>(blit_image);
  }
  void add_ensure_image_layout_node(VkImage vk_image, VkImageLayout vk_image_layout);
  void add_dispatch_node(const VKDispatchNode::CreateInfo &dispatch_info);

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

  friend class VKCommandBuilder;
  friend class Sequential;
};

}  // namespace blender::gpu::render_graph
