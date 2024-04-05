/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "../vk_command_buffer_wrapper.hh"
#include "../vk_resource_dependencies.hh"
#include "../vk_resources.hh"
#include "vk_common.hh"

namespace blender::gpu::render_graph {
struct VKClearColorImageNode : NonCopyable {
  struct Data {
    VkImage vk_image;
    VkClearColorValue vk_clear_color_value;
    VkImageSubresourceRange vk_image_subresource_range;
  };

  static constexpr bool uses_image_resources = true;
  static constexpr bool uses_buffer_resources = false;
  static constexpr VkPipelineStageFlags pipeline_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
  static constexpr VKNodeType node_type = VKNodeType::CLEAR_COLOR_IMAGE;

  template<typename Node> static void set_node_data(Node &node, const Data &data)
  {
    node.clear_color_image = data;
  }

  static void build_resource_dependencies(VKResources &resources,
                                          VKResourceDependencies &dependencies,
                                          NodeHandle node_handle,
                                          const Data &data)
  {
    VersionedResource resource = resources.get_image_and_increase_version(data.vk_image);
    dependencies.add_write_resource(
        node_handle, resource, VK_ACCESS_TRANSFER_WRITE_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
  }

  static void build_commands(VKCommandBufferInterface &command_buffer, const Data &data)
  {
    command_buffer.clear_color_image(data.vk_image,
                                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                     &data.vk_clear_color_value,
                                     1,
                                     &data.vk_image_subresource_range);
  }
};
}  // namespace blender::gpu::render_graph
