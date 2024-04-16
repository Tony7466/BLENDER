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
#include "vk_resource_dependencies.hh"
#include "vk_resource_state_tracker.hh"
#include "vk_types.hh"

namespace blender::gpu::render_graph {
class VKScheduler;

class VKRenderGraph : public NonCopyable {
  friend class VKCommandBuilder;
  friend class VKScheduler;

  VKResourceDependencies resource_dependencies_;
  VKResourceHandles<VKNode> nodes_;
  VKCommandBuilder command_builder_;

  std::unique_ptr<VKCommandBufferInterface> command_buffer_;

  /**
   * Not owning pointer to device resources.
   *
   * Is marked optional as device could
   */
  VKResourceStateTracker &resources_;

 public:
  VKRenderGraph(std::unique_ptr<VKCommandBufferInterface> command_buffer,
                VKResourceStateTracker &resources);

  /**
   * Free all resources held by the render graph.
   */
  void deinit();

 private:
  /**
   * Add a node to the render graph.
   */
  template<typename NodeInfo> void add_node(const typename NodeInfo::CreateInfo &create_info)
  {
    std::scoped_lock lock(resources_.mutex_get());
    NodeHandle node_handle = nodes_.allocate();
    VKNode &node = nodes_.get(node_handle);
    node.set_node_data<NodeInfo>(create_info);
    node.build_resource_dependencies<NodeInfo>(
        resources_, resource_dependencies_, node_handle, create_info);
  }

 public:
  void add_node(const VKClearColorImageNode::CreateInfo &clear_color_image)
  {
    add_node<VKClearColorImageNode>(clear_color_image);
  }
  void add_node(const VKFillBufferNode::CreateInfo &fill_buffer)
  {
    add_node<VKFillBufferNode>(fill_buffer);
  }
  void add_node(const VKCopyBufferNode::CreateInfo &copy_buffer)
  {
    add_node<VKCopyBufferNode>(copy_buffer);
  }
  void add_node(const VKCopyBufferToImageNode::CreateInfo &copy_buffer_to_image)
  {
    add_node<VKCopyBufferToImageNode>(copy_buffer_to_image);
  }
  void add_node(const VKCopyImageNode::CreateInfo &copy_image_to_buffer)
  {
    add_node<VKCopyImageNode>(copy_image_to_buffer);
  }
  void add_node(const VKCopyImageToBufferNode::CreateInfo &copy_image_to_buffer)
  {
    add_node<VKCopyImageToBufferNode>(copy_image_to_buffer);
  }
  void add_node(const VKBlitImageNode::CreateInfo &blit_image)
  {
    add_node<VKBlitImageNode>(blit_image);
  }
  void add_node(const VKDispatchNode::CreateInfo &dispatch)
  {
    add_node<VKDispatchNode>(dispatch);
  }

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
  void remove_nodes(Span<NodeHandle> node_handles);
};

}  // namespace blender::gpu::render_graph
