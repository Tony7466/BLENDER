/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph_links.hh"

namespace blender::gpu::render_graph {

void VKRenderGraphLinks::add_input(NodeHandle handle,
                                   ResourceWithStamp resource_handle,
                                   VkAccessFlags vk_access_flags,
                                   VkImageLayout vk_image_layout)
{
  Link usage = {};
  usage.resource = resource_handle;
  usage.vk_access_flags = vk_access_flags;
  usage.vk_image_layout = vk_image_layout;
  ensure_capacity(handle);
  links_per_node_[handle].inputs.append(usage);
}

void VKRenderGraphLinks::add_output(NodeHandle handle,
                                    ResourceWithStamp resource_handle,
                                    VkAccessFlags vk_access_flags,
                                    VkImageLayout vk_image_layout)
{
  Link usage = {};
  usage.resource = resource_handle;
  usage.vk_access_flags = vk_access_flags;
  usage.vk_image_layout = vk_image_layout;
  ensure_capacity(handle);
  links_per_node_[handle].outputs.append(usage);
}

void VKRenderGraphLinks::remove_links(Span<NodeHandle> node_handles)
{
  for (NodeHandle node_handle : node_handles) {
    links_per_node_[node_handle].inputs.clear();
    links_per_node_[node_handle].outputs.clear();
  }
}

void VKRenderGraphLinks::ensure_capacity(NodeHandle node_handle)
{
  int64_t needed_size = node_handle + 1;
  if (links_per_node_.size() < needed_size) {
    links_per_node_.resize(needed_size);
  }
}

}  // namespace blender::gpu::render_graph
