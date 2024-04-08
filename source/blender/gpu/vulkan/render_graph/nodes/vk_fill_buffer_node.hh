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
struct VKFillBufferNode : NonCopyable {
  struct Data {
    VkBuffer vk_buffer;
    VkDeviceSize size;
    uint32_t data;
  };
  using CreateInfo = Data;

  static constexpr bool uses_image_resources = false;
  static constexpr bool uses_buffer_resources = true;
  static constexpr VkPipelineStageFlags pipeline_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
  static constexpr VKNodeType node_type = VKNodeType::FILL_BUFFER;

  template<typename Node> static void set_node_data(Node &node, const CreateInfo &create_info)
  {
    node.fill_buffer = create_info;
  }

  template<typename Node> static void free_data(Node & /*node*/) {}

  static void build_resource_dependencies(VKResources &resources,
                                          VKResourceDependencies &dependencies,
                                          NodeHandle node_handle,
                                          const CreateInfo &create_info)
  {
    VersionedResource resource = resources.get_buffer_and_increase_version(create_info.vk_buffer);
    dependencies.add_write_resource(
        node_handle, resource, VK_ACCESS_TRANSFER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED);
  }

  static void build_commands(VKCommandBufferInterface &command_buffer, const Data &data)
  {
    command_buffer.fill_buffer(data.vk_buffer, 0, data.size, data.data);
  }
};
}  // namespace blender::gpu::render_graph
