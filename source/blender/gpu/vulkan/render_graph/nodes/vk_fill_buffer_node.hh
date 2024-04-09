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
 * Information stored inside the render graph node. See `VKNodeData`.
 */
struct VKFillBufferData {
  VkBuffer vk_buffer;
  VkDeviceSize size;
  uint32_t data;
};

/**
 * Information needed to add a node to the render graph.
 */
using VKFillBufferCreateInfo = VKFillBufferData;
class VKFillBufferNode : public VKNodeClass<VKNodeType::FILL_BUFFER,
                                            VKFillBufferCreateInfo,
                                            VKFillBufferData,
                                            VK_PIPELINE_STAGE_TRANSFER_BIT,
                                            VKResourceType::BUFFER> {
 public:
  /**
   * Update the node data with the data inside create_info.
   *
   * Has been implemented as a template to ensure all node specific data
   * (`VK*Data`/`VK*CreateInfo`) types can be included in the same header file as the logic. The
   * actual node data (`VKNodeData` includes all header files.)
   */
  template<typename Node>
  static void set_node_data(Node &node, const VKFillBufferCreateInfo &create_info)
  {
    node.fill_buffer = create_info;
  }

  /**
   * Extract read/write resource dependencies from `create_info` and add them to `dependencies`.
   */
  void build_resource_dependencies(VKResources &resources,
                                   VKResourceDependencies &dependencies,
                                   NodeHandle node_handle,
                                   const VKFillBufferCreateInfo &create_info) override
  {
    VersionedResource resource = resources.get_buffer_and_increase_version(create_info.vk_buffer);
    dependencies.add_write_resource(
        node_handle, resource, VK_ACCESS_TRANSFER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED);
  }

  /**
   * Build the commands and add them to the command_buffer.
   */
  void build_commands(VKCommandBufferInterface &command_buffer,
                      const VKFillBufferData &data,
                      VKBoundPipelines & /*r_bound_pipelines*/) override
  {
    command_buffer.fill_buffer(data.vk_buffer, 0, data.size, data.data);
  }
};
}  // namespace blender::gpu::render_graph
