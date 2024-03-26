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
  NodeHandle handle = allocate();
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
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::FILL_BUFFER;
  node.fill_buffer.vk_buffer = vk_buffer;
  node.fill_buffer.size = size;
  node.fill_buffer.data = data;

  return handle;
}

NodeHandle VKRenderGraphNodes::add_copy_buffer_node(VkBuffer src_buffer,
                                                    VkBuffer dst_buffer,
                                                    const VkBufferCopy &region)
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::COPY_BUFFER;
  node.copy_buffer.src_buffer = src_buffer;
  node.copy_buffer.dst_buffer = dst_buffer;
  node.copy_buffer.region = region;

  return handle;
}

NodeHandle VKRenderGraphNodes::add_copy_image_node(VkImage src_image,
                                                   VkImage dst_image,
                                                   const VkImageCopy &region)
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::COPY_IMAGE;
  node.copy_image.src_image = src_image;
  node.copy_image.dst_image = dst_image;
  node.copy_image.region = region;

  return handle;
}

NodeHandle VKRenderGraphNodes::add_copy_buffer_to_image_node(VkBuffer src_buffer,
                                                             VkImage dst_image,
                                                             const VkBufferImageCopy &region)
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::COPY_BUFFER_TO_IMAGE;
  node.copy_buffer_to_image.src_buffer = src_buffer;
  node.copy_buffer_to_image.dst_image = dst_image;
  node.copy_buffer_to_image.region = region;

  return handle;
}

NodeHandle VKRenderGraphNodes::add_copy_image_to_buffer_node(VkImage src_image,
                                                             VkBuffer dst_buffer,
                                                             const VkBufferImageCopy &region)
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::COPY_IMAGE_TO_BUFFER;
  node.copy_image_to_buffer.src_image = src_image;
  node.copy_image_to_buffer.dst_buffer = dst_buffer;
  node.copy_image_to_buffer.region = region;

  return handle;
}

NodeHandle VKRenderGraphNodes::add_synchronization_node()
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::SYNCHRONIZATION;

  return handle;
}

static void localize_push_constants_data(VKPushConstantsData &dst, const VKPushConstantsData &src)
{
  if (src.size) {
    BLI_assert(src.data);
    void *data = MEM_mallocN(src.size, __func__);
    memcpy(data, src.data, src.size);
    dst.data = data;
  }
  dst.size = src.size;
}

static void free_shader_data(VKShaderData &data)
{
  data.push_constants.size = 0;
  MEM_SAFE_FREE(data.push_constants.data);
}

NodeHandle VKRenderGraphNodes::add_dispatch_node(const VKDispatchInfo &dispatch_info)
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::DISPATCH;
  node.dispatch = dispatch_info.dispatch_node;
  localize_push_constants_data(node.dispatch.shader_data.push_constants,
                               dispatch_info.dispatch_node.shader_data.push_constants);

  return handle;
}

void VKRenderGraphNodes::add_read_resource(NodeHandle handle,
                                           VersionedResource resource_handle,
                                           VkAccessFlags vk_access_flags,
                                           VkImageLayout vk_image_layout)
{
  ResourceUsage usage = {};
  usage.resource = resource_handle;
  usage.vk_access_flags = vk_access_flags;
  usage.vk_image_layout = vk_image_layout;
  read_resources_per_node_[handle].resources.append(usage);
}

void VKRenderGraphNodes::add_write_resource(NodeHandle handle,
                                            VersionedResource resource_handle,
                                            VkAccessFlags vk_access_flags,
                                            VkImageLayout vk_image_layout)
{
  ResourceUsage usage = {};
  usage.resource = resource_handle;
  usage.vk_access_flags = vk_access_flags;
  usage.vk_image_layout = vk_image_layout;
  write_resources_per_node_[handle].resources.append(usage);
}

void VKRenderGraphNodes::remove_nodes(Span<NodeHandle> node_handles)
{
  for (NodeHandle node_handle : node_handles) {
    // TODO: move resources.clear to functions.
    read_resources_per_node_[node_handle].resources.clear();
    write_resources_per_node_[node_handle].resources.clear();
    Node &node = get(node_handle);
    free_data(node);
    nodes_.free(node_handle);
  }
}

void VKRenderGraphNodes::free_data(Node &node)
{
  switch (node.type) {
    case Node::Type::DISPATCH:
      free_shader_data(node.dispatch.shader_data);
      break;

    case Node::Type::UNUSED:
    case Node::Type::CLEAR_COLOR_IMAGE:
    case Node::Type::FILL_BUFFER:
    case Node::Type::COPY_BUFFER:
    case Node::Type::COPY_IMAGE:
    case Node::Type::COPY_IMAGE_TO_BUFFER:
    case Node::Type::COPY_BUFFER_TO_IMAGE:
    case Node::Type::SYNCHRONIZATION:
      break;
  }

  memset(&node, 0, sizeof(Node));
  node.type = Node::Type::UNUSED;
}

NodeHandle VKRenderGraphNodes::allocate()
{
  NodeHandle node_handle = nodes_.allocate();
  ensure_vector_sizes();
  return node_handle;
}

void VKRenderGraphNodes::ensure_vector_sizes()
{
  if (read_resources_per_node_.size() < nodes_.size()) {
    read_resources_per_node_.resize(nodes_.size());
  }
  if (write_resources_per_node_.size() < nodes_.size()) {
    write_resources_per_node_.resize(nodes_.size());
  }
}

}  // namespace blender::gpu
