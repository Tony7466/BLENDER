/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_nodes.hh"

namespace blender::gpu::render_graph {

NodeHandle VKNodes::add_clear_image_node(VkImage vk_image,
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

NodeHandle VKNodes::add_fill_buffer_node(VkBuffer vk_buffer, VkDeviceSize size, uint32_t data)
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

NodeHandle VKNodes::add_copy_buffer_node(VkBuffer src_buffer,
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

NodeHandle VKNodes::add_copy_image_node(VkImage src_image,
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

NodeHandle VKNodes::add_copy_buffer_to_image_node(VkBuffer src_buffer,
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

NodeHandle VKNodes::add_copy_image_to_buffer_node(VkImage src_image,
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

NodeHandle VKNodes::add_blit_image_node(VkImage src_image,
                                        VkImage dst_image,
                                        const VkImageBlit &region,
                                        VkFilter filter)
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::BLIT_IMAGE;
  node.blit_image.src_image = src_image;
  node.blit_image.dst_image = dst_image;
  node.blit_image.region = region;
  node.blit_image.filter = filter;

  return handle;
}

NodeHandle VKNodes::add_synchronization_node()
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::SYNCHRONIZATION;

  return handle;
}

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

static void free_shader_data(VKPipelineData &data)
{
  MEM_SAFE_FREE(data.push_constants_data);
}

NodeHandle VKNodes::add_dispatch_node(const VKDispatchInfo &dispatch_info)
{
  NodeHandle handle = allocate();
  Node &node = nodes_.get(handle);
  BLI_assert(node.type == Node::Type::UNUSED);

  node.type = Node::Type::DISPATCH;
  node.dispatch = dispatch_info.dispatch_node;
  localize_shader_data(node.dispatch.pipeline_data, dispatch_info.dispatch_node.pipeline_data);

  return handle;
}

void VKNodes::add_read_resource(NodeHandle handle,
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

void VKNodes::add_write_resource(NodeHandle handle,
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

void VKNodes::remove_nodes(Span<NodeHandle> node_handles)
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

void VKNodes::free_data(Node &node)
{
  switch (node.type) {
    case Node::Type::DISPATCH:
      free_shader_data(node.dispatch.pipeline_data);
      break;

    case Node::Type::UNUSED:
    case Node::Type::CLEAR_COLOR_IMAGE:
    case Node::Type::FILL_BUFFER:
    case Node::Type::COPY_BUFFER:
    case Node::Type::COPY_IMAGE:
    case Node::Type::COPY_IMAGE_TO_BUFFER:
    case Node::Type::COPY_BUFFER_TO_IMAGE:
    case Node::Type::BLIT_IMAGE:
    case Node::Type::SYNCHRONIZATION:
      break;
  }

  memset(&node, 0, sizeof(Node));
  node.type = Node::Type::UNUSED;
}

NodeHandle VKNodes::allocate()
{
  NodeHandle node_handle = nodes_.allocate();
  ensure_vector_sizes();
  return node_handle;
}

void VKNodes::ensure_vector_sizes()
{
  if (read_resources_per_node_.size() < nodes_.size()) {
    read_resources_per_node_.resize(nodes_.size());
  }
  if (write_resources_per_node_.size() < nodes_.size()) {
    write_resources_per_node_.resize(nodes_.size());
  }
}

}  // namespace blender::gpu::render_graph
