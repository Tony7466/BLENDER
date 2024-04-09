/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_nodes.hh"

namespace blender::gpu::render_graph {

static void free_data(VKNodeData &node_data)
{
  switch (node_data.type) {
    case VKNodeType::DISPATCH:
      VKDispatchNode::free_data(node_data);
      break;

    case VKNodeType::UNUSED:
    case VKNodeType::CLEAR_COLOR_IMAGE:
    case VKNodeType::FILL_BUFFER:
    case VKNodeType::COPY_BUFFER:
    case VKNodeType::COPY_IMAGE:
    case VKNodeType::COPY_IMAGE_TO_BUFFER:
    case VKNodeType::COPY_BUFFER_TO_IMAGE:
    case VKNodeType::BLIT_IMAGE:
    case VKNodeType::SYNCHRONIZATION:
      break;
  }

  memset(&node_data, 0, sizeof(VKNodeData));
  node_data.type = VKNodeType::UNUSED;
}

void VKNodes::remove_nodes(Span<NodeHandle> node_handles)
{
  for (NodeHandle node_handle : node_handles) {
    VKNodeData &node_data = get(node_handle);
    free_data(node_data);
    nodes_.free(node_handle);
  }
}

NodeHandle VKNodes::allocate()
{
  NodeHandle node_handle = nodes_.allocate();
  return node_handle;
}

}  // namespace blender::gpu::render_graph
