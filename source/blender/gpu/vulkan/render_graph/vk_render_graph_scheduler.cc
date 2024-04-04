/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph_scheduler.hh"
#include "vk_render_graph.hh"

#include "BLI_index_range.hh"

namespace blender::gpu::render_graph {

void Sequential::select_nodes_for_image(const VKRenderGraph &render_graph,
                                        VkImage vk_image,
                                        Vector<NodeHandle> &r_selected_nodes)
{
  UNUSED_VARS(vk_image);
  select_all_nodes(render_graph, r_selected_nodes);
}

void Sequential::select_nodes_for_buffer(const VKRenderGraph &render_graph,
                                         VkBuffer vk_buffer,
                                         Vector<NodeHandle> &r_selected_nodes)
{
  UNUSED_VARS(vk_buffer);
  select_all_nodes(render_graph, r_selected_nodes);
}

void Sequential::select_all_nodes(const VKRenderGraph &render_graph,
                                  Vector<NodeHandle> &r_selected_nodes)
{
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
