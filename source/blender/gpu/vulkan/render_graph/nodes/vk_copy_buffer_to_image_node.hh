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
struct VKCopyBufferToImageNode : NonCopyable {
  struct Data {
    VkBuffer src_buffer;
    VkImage dst_image;
    VkBufferImageCopy region;
  };
  using CreateInfo = Data;

  static constexpr bool uses_image_resources = true;
  static constexpr bool uses_buffer_resources = true;
  static constexpr VkPipelineStageFlags pipeline_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
  static constexpr VKNodeType node_type = VKNodeType::COPY_BUFFER_TO_IMAGE;

  template<typename Node> static void set_node_data(Node &node, const CreateInfo &create_info)
  {
    node.copy_buffer_to_image = create_info;
  }

  static void build_resource_dependencies(VKResources &resources,
                                          VKResourceDependencies &dependencies,
                                          NodeHandle node_handle,
                                          const CreateInfo &create_info)
  {
    VersionedResource src_resource = resources.get_buffer(create_info.src_buffer);
    VersionedResource dst_resource = resources.get_image_and_increase_version(create_info.dst_image);
    dependencies.add_read_resource(
        node_handle, src_resource, VK_ACCESS_TRANSFER_READ_BIT, VK_IMAGE_LAYOUT_UNDEFINED);
    dependencies.add_write_resource(node_handle,
                                    dst_resource,
                                    VK_ACCESS_TRANSFER_WRITE_BIT,
                                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
  }

  static void build_commands(VKCommandBufferInterface &command_buffer, const Data &data)
  {
    command_buffer.copy_buffer_to_image(
        data.src_buffer, data.dst_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &data.region);
  }
};
}  // namespace blender::gpu::render_graph
