/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_resources.hh"
#include "vk_types.hh"

namespace blender::gpu::render_graph {
class VKResourceDependencies : NonCopyable, NonMovable {
 public:
  struct ResourceUsage {
    /**
     * Which resource is being accessed.
     */
    VersionedResource resource;

    /**
     * How is the resource being accessed.
     */
    VkAccessFlags vk_access_flags;

    /**
     * When resource is an image, which layout should the image be using.
     */
    VkImageLayout vk_image_layout;
  };

  struct NodeResources {
    Vector<ResourceUsage> resources;
  };

 private:
  Vector<NodeResources> read_resources_per_node_;
  Vector<NodeResources> write_resources_per_node_;

 public:
  void add_read_resource(NodeHandle handle,
                         VersionedResource resource_handle,
                         VkAccessFlags vk_access_flags,
                         VkImageLayout vk_image_layout);
  void add_write_resource(NodeHandle handle,
                          VersionedResource resource_handle,
                          VkAccessFlags vk_access_flags,
                          VkImageLayout vk_image_layout);

  Span<ResourceUsage> get_write_resources(NodeHandle node_handle)
  {
    return write_resources_per_node_[node_handle].resources;
  }

  Span<ResourceUsage> get_read_resources(NodeHandle node_handle)
  {
    return read_resources_per_node_[node_handle].resources;
  }

  void remove_nodes(Span<NodeHandle> node_handles);

 private:
  void ensure_vector_sizes(NodeHandle node_handle);
};

}  // namespace blender::gpu::render_graph
