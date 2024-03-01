/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph_scheduler.hh"
#include "vk_render_graph.hh"

#include "BLI_index_range.hh"

namespace blender::gpu {

void Sequential::select_nodes_for_image(const VKRenderGraph &render_graph,
                                        VkImage vk_image,
                                        Vector<NodeHandle> &r_selected_nodes)
{
  // select all nodes from the start of the graph to the last write access
  // For the time being we will select all nodes. In the future we could only select a subset,
  // But still in sequence.
  NodeHandle node_handle = 0;

  for (const std::optional<VKRenderGraphNodes::Node> &optional_node : render_graph.nodes_.nodes())
  {
    if (optional_node.has_value()) {
      const VKRenderGraphNodes::Node &node = *optional_node;
      if (node.type != VKRenderGraphNodes::Node::Type::UNUSED) {
        r_selected_nodes.append(node_handle);
      }
    }
    node_handle += 1;
  }
}

}  // namespace blender::gpu
