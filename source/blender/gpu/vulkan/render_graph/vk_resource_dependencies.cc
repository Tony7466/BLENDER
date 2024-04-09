/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_resource_dependencies.hh"

namespace blender::gpu::render_graph {

void VKResourceDependencies::add_read_resource(NodeHandle handle,
                                               VersionedResource resource_handle,
                                               VkAccessFlags vk_access_flags,
                                               VkImageLayout vk_image_layout)
{
  ResourceUsage usage = {};
  usage.resource = resource_handle;
  usage.vk_access_flags = vk_access_flags;
  usage.vk_image_layout = vk_image_layout;
  ensure_vector_sizes(handle);
  read_resources_per_node_[handle].resources.append(usage);
}

void VKResourceDependencies::add_write_resource(NodeHandle handle,
                                                VersionedResource resource_handle,
                                                VkAccessFlags vk_access_flags,
                                                VkImageLayout vk_image_layout)
{
  ResourceUsage usage = {};
  usage.resource = resource_handle;
  usage.vk_access_flags = vk_access_flags;
  usage.vk_image_layout = vk_image_layout;
  ensure_vector_sizes(handle);
  write_resources_per_node_[handle].resources.append(usage);
}

void VKResourceDependencies::remove_nodes(Span<NodeHandle> node_handles)
{
  for (NodeHandle node_handle : node_handles) {
    // TODO: move resources.clear to functions.
    read_resources_per_node_[node_handle].resources.clear();
    write_resources_per_node_[node_handle].resources.clear();
  }
}

void VKResourceDependencies::ensure_vector_sizes(NodeHandle node_handle)
{
  int64_t needed_size = node_handle + 1;
  if (read_resources_per_node_.size() < needed_size) {
    read_resources_per_node_.resize(needed_size);
  }
  if (write_resources_per_node_.size() < needed_size) {
    write_resources_per_node_.resize(needed_size);
  }
}

}  // namespace blender::gpu::render_graph
