/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_node_class.hh"

namespace blender::gpu::render_graph {

/**
 * Information stored inside the render graph node.
 */
struct VKClearColorImageData {
  VkImage vk_image;
  VkClearColorValue vk_clear_color_value;
  VkImageSubresourceRange vk_image_subresource_range;
};

/**
 * Information needed to create for adding a clear color image node in the render graph.
 */
using VKClearColorImageCreateInfo = VKClearColorImageData;

class VKClearColorImageNode : public VKNodeClass<VKNodeType::CLEAR_COLOR_IMAGE,
                                                 VKClearColorImageCreateInfo,
                                                 VKClearColorImageData,
                                                 VK_PIPELINE_STAGE_TRANSFER_BIT,
                                                 VKResourceType::IMAGE> {
 public:
  template<typename Node>
  static void set_node_data(Node &node, const VKClearColorImageCreateInfo &create_info)
  {
    node.clear_color_image = create_info;
  }

  void build_resource_dependencies(VKResources &resources,
                                   VKResourceDependencies &dependencies,
                                   NodeHandle node_handle,
                                   const VKClearColorImageCreateInfo &create_info) override
  {
    VersionedResource resource = resources.get_image_and_increase_version(create_info.vk_image);
    dependencies.add_write_resource(
        node_handle, resource, VK_ACCESS_TRANSFER_WRITE_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
  }

   void build_commands(VKCommandBufferInterface &command_buffer,
                             const VKClearColorImageData &data,VKBoundPipelines &/*r_bound_pipelines*/) override
  {
    command_buffer.clear_color_image(data.vk_image,
                                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                     &data.vk_clear_color_value,
                                     1,
                                     &data.vk_image_subresource_range);
  }
};
}  // namespace blender::gpu::render_graph
