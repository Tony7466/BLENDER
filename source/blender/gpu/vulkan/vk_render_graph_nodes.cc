/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph_nodes.hh"

namespace blender::gpu {

NodeHandle VKRenderGraphNodes::add_clear_image_node(
    VkImage vk_image,
    VkClearColorValue &vk_clear_color_value,
    VkImageSubresourceRange &vk_image_subresource_range)
{
  NodeHandle handle = nodes_.allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::CLEAR_COLOR_IMAGE;
  node.clear_color_image.vk_image = vk_image;
  node.clear_color_image.vk_clear_color_value = vk_clear_color_value;
  node.clear_color_image.vk_image_subresource_range = vk_image_subresource_range;

  return handle;
}

NodeHandle VKRenderGraphNodes::add_fill_buffer_node(VkBuffer vk_buffer,
                                                    VkDeviceSize size,
                                                    uint32_t data)
{
  NodeHandle handle = nodes_.allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::FILL_BUFFER;
  node.fill_buffer.vk_buffer = vk_buffer;
  node.fill_buffer.size = size;
  node.fill_buffer.data = data;

  return handle;
}

void VKRenderGraphNodes::add_write_resource(NodeHandle handle, VersionedResource resource_handle)
{
  if (write_resources_per_node_.size() < nodes_.size()) {
    write_resources_per_node_.resize(nodes_.size());
  }

  write_resources_per_node_[handle].append(resource_handle);
}

}  // namespace blender::gpu
