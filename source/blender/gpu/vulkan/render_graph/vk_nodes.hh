/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_node.hh"
#include "vk_resource_state_tracker.hh"
#include "vk_types.hh"

namespace blender::gpu::render_graph {

class VKCommandBuilder;

class VKNodes {
 private:
  VKResourceHandles<NodeHandle, VKNode> nodes_;

 public:
  template<typename NodeInfo> NodeHandle add_node(const typename NodeInfo::CreateInfo &create_info)
  {
    NodeHandle node_handle = allocate();
    VKNode &node = nodes_.get(node_handle);
    node.set_node_data<NodeInfo>(create_info);
    return node_handle;
  }

  void remove_nodes(Span<NodeHandle> node_handles);

  Span<const std::optional<VKNode>> nodes() const
  {
    return nodes_.as_span();
  }

  const VKNode &get(NodeHandle node_handle) const
  {
    return nodes_.get(node_handle);
  }
  VKNode &get(NodeHandle node_handle)
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
