/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_scheduler.hh"
#include "vk_render_graph.hh"

#include "BLI_index_range.hh"

namespace blender::gpu::render_graph {

void VKScheduler::select_nodes_for_image(const VKRenderGraph &render_graph,
                                         VkImage vk_image,
                                         Vector<NodeHandle> &r_selected_nodes)
{
  UNUSED_VARS(vk_image);
  select_all_nodes(render_graph, r_selected_nodes);
}

void VKScheduler::select_nodes_for_buffer(const VKRenderGraph &render_graph,
                                          VkBuffer vk_buffer,
                                          Vector<NodeHandle> &r_selected_nodes)
{
  UNUSED_VARS(vk_buffer);
  select_all_nodes(render_graph, r_selected_nodes);
}

void VKScheduler::select_all_nodes(const VKRenderGraph &render_graph,
                                   Vector<NodeHandle> &r_selected_nodes)
{
  NodeHandle node_handle = 0;

  for (const std::optional<VKNodeData> &optional_node_data : render_graph.nodes_.nodes()) {
    if (optional_node_data.has_value()) {
      const VKNodeData &node_data = *optional_node_data;
      if (node_data.type != VKNodeType::UNUSED) {
        r_selected_nodes.append(node_handle);
      }
    }
    node_handle += 1;
  }
}

}  // namespace blender::gpu::render_graph
