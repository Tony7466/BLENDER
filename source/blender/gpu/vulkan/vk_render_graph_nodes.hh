/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_vector.hh"

#include "vk_render_graph_resources.hh"

#include "vk_common.hh"

namespace blender::gpu {

class VKRenderGraphCommandBuilder;

using NodeHandle = uint64_t;

class VKRenderGraphNodes {
 public:
  struct Node {
    enum class Type {
      UNUSED,
      CLEAR_COLOR_IMAGE,
      FILL_BUFFER,
      COPY_BUFFER,
    };

    Type type;
    union {
      struct {
        VkImage vk_image;
        VkClearColorValue vk_clear_color_value;
        VkImageSubresourceRange vk_image_subresource_range;
      } clear_color_image;

      struct {
        VkBuffer vk_buffer;
        VkDeviceSize size;
        uint32_t data;
      } fill_buffer;

      struct {
        VkBuffer src_buffer;
        VkBuffer dst_buffer;
        VkBufferCopy region;
      } copy_buffer;
    };
  };

 private:
  VKRenderGraphList<NodeHandle, Node> nodes_;
  Vector<Vector<VersionedResource>> read_resources_per_node_;
  Vector<Vector<VersionedResource>> write_resources_per_node_;

 public:
  NodeHandle add_clear_image_node(VkImage vk_image,
                                  VkClearColorValue &vk_clear_color_value,
                                  VkImageSubresourceRange &vk_image_subresource_range);
  NodeHandle add_fill_buffer_node(VkBuffer vk_buffer, VkDeviceSize size, uint32_t data);
  NodeHandle add_copy_buffer_node(VkBuffer src_buffer,
                                  VkBuffer dst_buffer,
                                  const VkBufferCopy &region);

  void add_read_resource(NodeHandle handle, VersionedResource resource_handle);
  void add_write_resource(NodeHandle handle, VersionedResource resource_handle);

  void remove_nodes(Span<NodeHandle> node_handles);

  Span<const std::optional<Node>> nodes() const
  {
    return nodes_.as_span();
  }

  const Node &get(NodeHandle node_handle) const
  {
    return nodes_.get(node_handle);
  }
  Node &get(NodeHandle node_handle)
  {
    return nodes_.get(node_handle);
  }
  int64_t size() const
  {
    return nodes_.size();
  }

  Span<VersionedResource> get_write_resources(NodeHandle node_handle)
  {
    return write_resources_per_node_[node_handle];
  }

  Span<VersionedResource> get_read_resources(NodeHandle node_handle)
  {
    return read_resources_per_node_[node_handle];
  }

 private:
  NodeHandle allocate();
  void ensure_vector_sizes();
  void mark_unused(Node &node);
};

}  // namespace blender::gpu
