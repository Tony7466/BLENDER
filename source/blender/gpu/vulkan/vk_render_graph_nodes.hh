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

  void add_write_resource(NodeHandle handle, VersionedResource resource_handle);

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
};

}  // namespace blender::gpu
