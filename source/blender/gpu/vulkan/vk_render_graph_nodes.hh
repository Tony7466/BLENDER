/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_render_graph_resources.hh"
#include "vk_render_graph_types.hh"

namespace blender::gpu {

class VKRenderGraphCommandBuilder;
struct VKDispatchInfo;

using NodeHandle = uint64_t;

class VKRenderGraphNodes {

 public:
  struct Node {
    enum class Type {
      UNUSED,
      CLEAR_COLOR_IMAGE,
      FILL_BUFFER,
      COPY_BUFFER,
      DISPATCH,
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

      VKDispatchNode dispatch;
    };
  };

  struct ResourceUsage {
    /**
     * Which resource is being accessed.
     */
    VersionedResource resource;

    /**
     * How is the resource being accessed.
     */
    VkAccessFlags vk_access_flags;
  };

  struct NodeResources {
    Vector<ResourceUsage> resources;
  };

 private:
  VKRenderGraphList<NodeHandle, Node> nodes_;
  Vector<NodeResources> read_resources_per_node_;
  Vector<NodeResources> write_resources_per_node_;

 public:
  NodeHandle add_clear_image_node(VkImage vk_image,
                                  VkClearColorValue &vk_clear_color_value,
                                  VkImageSubresourceRange &vk_image_subresource_range);
  NodeHandle add_fill_buffer_node(VkBuffer vk_buffer, VkDeviceSize size, uint32_t data);
  NodeHandle add_copy_buffer_node(VkBuffer src_buffer,
                                  VkBuffer dst_buffer,
                                  const VkBufferCopy &region);
  NodeHandle add_dispatch_node(const VKDispatchInfo &dispatch_info);

  void add_read_resource(NodeHandle handle,
                         VersionedResource resource_handle,
                         VkAccessFlags vk_access_flags);
  void add_write_resource(NodeHandle handle,
                          VersionedResource resource_handle,
                          VkAccessFlags vk_access_flags);

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

  Span<ResourceUsage> get_write_resources(NodeHandle node_handle)
  {
    return write_resources_per_node_[node_handle].resources;
  }

  Span<ResourceUsage> get_read_resources(NodeHandle node_handle)
  {
    return read_resources_per_node_[node_handle].resources;
  }

 private:
  NodeHandle allocate();
  void ensure_vector_sizes();
  void mark_unused(Node &node);
};

}  // namespace blender::gpu
