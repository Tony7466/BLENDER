/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_node_data.hh"
#include "vk_resources.hh"
#include "vk_types.hh"

namespace blender::gpu::render_graph {

class VKCommandBuilder;

class VKNodes {

 public:
  // TODO: Replace all occurrences with VKNodeData.
  using Node = VKNodeData;

 private:
  VKRenderGraphList<NodeHandle, Node> nodes_;

 public:
  NodeHandle add_fill_buffer_node(VkBuffer vk_buffer, VkDeviceSize size, uint32_t data);
  NodeHandle add_copy_image_node(VkImage src_image, VkImage dst_image, const VkImageCopy &region);
  NodeHandle add_copy_image_to_buffer_node(VkImage src_image,
                                           VkBuffer dst_buffer,
                                           const VkBufferImageCopy &region);
  NodeHandle add_synchronization_node();
  NodeHandle add_dispatch_node(const VKDispatchNode::CreateInfo &dispatch_info);

  template<typename NodeClass, typename NodeClassData>
  NodeHandle add_node(const NodeClassData &node_data)
  {
    NodeHandle node_handle = allocate();
    Node &node = nodes_.get(node_handle);
    BLI_assert(node.type == VKNodeType::UNUSED);
    node.type = NodeClass::node_type;
    NodeClass::set_node_data(node, node_data);
    return node_handle;
  }

  void remove_nodes(Span<NodeHandle> node_handles);

  Span<const std::optional<Node>> nodes() const
  {
    return nodes_.as_span();
  }

  const Node &get(NodeHandle node_handle) const
  {
    return nodes_.get(node_handle);
  }
  Node &get(NodeHandle node_handle)
  {
    return nodes_.get(node_handle);
  }
  int64_t size() const
  {
    return nodes_.size();
  }

 private:
  /**
   * Allocate a new node handle.
   */
  NodeHandle allocate();
  void free_data(Node &node);
};

}  // namespace blender::gpu::render_graph
