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
 private:
  VKResourceList<NodeHandle, VKNodeData> nodes_;

 public:
  template<typename NodeClass, typename NodeCreateInfo>
  NodeHandle add_node(const NodeCreateInfo &create_info)
  {
    NodeHandle node_handle = allocate();
    VKNodeData &node_data = nodes_.get(node_handle);
    BLI_assert(node_data.type == VKNodeType::UNUSED);
    NodeClass node_class;
    node_data.type = node_class.node_type;
    node_class.set_node_data(node_data, create_info);
    return node_handle;
  }

  void remove_nodes(Span<NodeHandle> node_handles);

  Span<const std::optional<VKNodeData>> nodes() const
  {
    return nodes_.as_span();
  }

  const VKNodeData &get(NodeHandle node_handle) const
  {
    return nodes_.get(node_handle);
  }
  VKNodeData &get(NodeHandle node_handle)
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
};

}  // namespace blender::gpu::render_graph
