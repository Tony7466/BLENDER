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
struct VKSynchronizationNode : NonCopyable {
  struct Data {};
  struct CreateInfo {
    VkImage vk_image;
    VkImageLayout vk_image_layout;
  };

  static constexpr bool uses_image_resources = true;
  static constexpr bool uses_buffer_resources = true;
  static constexpr VkPipelineStageFlags pipeline_stage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
  static constexpr VKNodeType node_type = VKNodeType::SYNCHRONIZATION;

  template<typename Node> static void set_node_data(Node &node, const CreateInfo &create_info)
  {
    UNUSED_VARS(create_info);
    node.synchronization = {};
  }

  template<typename Node> static void free_data(Node & /*node*/) {}

  static void build_resource_dependencies(VKResources &resources,
                                          VKResourceDependencies &dependencies,
                                          NodeHandle node_handle,
                                          const CreateInfo &create_info)
  {
    VersionedResource resource = resources.get_image_and_increase_version(create_info.vk_image);
    dependencies.add_write_resource(
        node_handle, resource, VK_ACCESS_TRANSFER_WRITE_BIT, create_info.vk_image_layout);
  }

  static void build_commands(VKCommandBufferInterface &command_buffer, const Data &data)
  {
    UNUSED_VARS(command_buffer, data);
    /* Intentionally left empty: A pipeline barrier has already been send to the command buffer. */
  }
};
}  // namespace blender::gpu::render_graph
