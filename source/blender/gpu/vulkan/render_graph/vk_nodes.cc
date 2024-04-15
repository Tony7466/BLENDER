/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_nodes.hh"

namespace blender::gpu::render_graph {

void VKNodes::remove_nodes(Span<NodeHandle> node_handles)
{
  for (NodeHandle node_handle : node_handles) {
    VKNode &node = get(node_handle);
    node.reset();
    nodes_.free(node_handle);
  }
}

NodeHandle VKNodes::allocate()
{
  NodeHandle node_handle = nodes_.allocate();
  return node_handle;
}

}  // namespace blender::gpu::render_graph
