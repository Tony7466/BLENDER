/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_node_class.hh"

namespace blender::gpu::render_graph {
struct VKSynchronizationData {};
struct VKSynchronizationCreateInfo {
  VkImage vk_image;
  VkImageLayout vk_image_layout;
};
class VKSynchronizationNode : public VKNodeClass<VKNodeType::SYNCHRONIZATION,
                                                 VKSynchronizationCreateInfo,
                                                 VKSynchronizationData,
                                                 VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                                                 VKResourceType::IMAGE | VKResourceType::BUFFER> {
 public:
  template<typename Node>
  static void set_node_data(Node &node, const VKSynchronizationCreateInfo &create_info)
  {
    UNUSED_VARS(create_info);
    node.synchronization = {};
  }

  static void build_resource_dependencies(VKResources &resources,
                                          VKResourceDependencies &dependencies,
                                          NodeHandle node_handle,
                                          const VKSynchronizationCreateInfo &create_info)
  {
    VersionedResource resource = resources.get_image_and_increase_version(create_info.vk_image);
    dependencies.add_write_resource(
        node_handle, resource, VK_ACCESS_TRANSFER_WRITE_BIT, create_info.vk_image_layout);
  }

  static void build_commands(VKCommandBufferInterface &command_buffer,
                             const VKSynchronizationData &data)
  {
    UNUSED_VARS(command_buffer, data);
    /* Intentionally left empty: A pipeline barrier has already been send to the command buffer. */
  }
};
}  // namespace blender::gpu::render_graph
