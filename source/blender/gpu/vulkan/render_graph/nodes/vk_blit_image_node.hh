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
struct VKBlitImageNode : NonCopyable {
  /**
   * Information stored inside the render graph node.
   */
  struct Data {
    VkImage src_image;
    VkImage dst_image;
    VkImageBlit region;
    VkFilter filter;
  };

  static constexpr bool uses_image_resources = true;
  static constexpr bool uses_buffer_resources = false;
  static constexpr VkPipelineStageFlags pipeline_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
  static constexpr VKNodeType node_type = VKNodeType::BLIT_IMAGE;

  template<typename Node> static void set_node_data(Node &node, const Data &data)
  {
    node.blit_image = data;
  }

  template<typename Node> static void free_data(Node & /*node*/) {}

  static void build_resource_dependencies(VKResources &resources,
                                          VKResourceDependencies &dependencies,
                                          NodeHandle node_handle,
                                          const Data &data)
  {
    VersionedResource src_resource = resources.get_image(data.src_image);
    VersionedResource dst_resource = resources.get_image_and_increase_version(data.dst_image);
    dependencies.add_read_resource(node_handle,
                                   src_resource,
                                   VK_ACCESS_TRANSFER_READ_BIT,
                                   VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    dependencies.add_write_resource(node_handle,
                                    dst_resource,
                                    VK_ACCESS_TRANSFER_WRITE_BIT,
                                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
  }

  static void build_commands(VKCommandBufferInterface &command_buffer, const Data &data)
  {
    command_buffer.blit_image(data.src_image,
                              VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                              data.dst_image,
                              VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                              1,
                              &data.region,
                              data.filter);
  }
};
}  // namespace blender::gpu::render_graph
