/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_nodes.hh"

namespace blender::gpu::render_graph {

static void localize_shader_data(VKPipelineData &dst, const VKPipelineData &src)
{
  dst.push_constants_data = nullptr;
  dst.push_constants_size = src.push_constants_size;
  if (src.push_constants_size) {
    BLI_assert(src.push_constants_data);
    void *data = MEM_mallocN(src.push_constants_size, __func__);
    memcpy(data, src.push_constants_data, src.push_constants_size);
    dst.push_constants_data = data;
  }
}

NodeHandle VKNodes::add_dispatch_node(const VKDispatchNode::CreateInfo &dispatch_info)
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == VKNodeType::UNUSED);

  node.type = VKNodeType::DISPATCH;
  node.dispatch = dispatch_info.dispatch_node;
  localize_shader_data(node.dispatch.pipeline_data, dispatch_info.dispatch_node.pipeline_data);

  return handle;
}

void VKNodes::remove_nodes(Span<NodeHandle> node_handles)
{
  for (NodeHandle node_handle : node_handles) {
    Node &node = get(node_handle);
    free_data(node);
    nodes_.free(node_handle);
  }
}

void VKNodes::free_data(Node &node)
{
  switch (node.type) {
    case VKNodeType::DISPATCH:
      VKDispatchNode::free_data(node);
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

  memset(&node, 0, sizeof(Node));
  node.type = VKNodeType::UNUSED;
}

NodeHandle VKNodes::allocate()
{
  NodeHandle node_handle = nodes_.allocate();
  return node_handle;
}

}  // namespace blender::gpu::render_graph
