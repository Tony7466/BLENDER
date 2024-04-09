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
struct VKCopyBufferData {
  VkBuffer src_buffer;
  VkBuffer dst_buffer;
  VkBufferCopy region;
};

/**
 * Information needed to add a node to the render graph.
 */
using VKCopyBufferCreateInfo = VKCopyBufferData;

class VKCopyBufferNode : public VKNodeClass<VKNodeType::COPY_BUFFER,
                                            VKCopyBufferCreateInfo,
                                            VKCopyBufferData,
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
  static void set_node_data(Node &node, const VKCopyBufferCreateInfo &create_info)
  {
    node.copy_buffer = create_info;
  }

  /**
   * Extract read/write resource dependencies from `create_info` and add them to `dependencies`.
   */
  void build_resource_dependencies(VKResources &resources,
                                   VKResourceDependencies &dependencies,
                                   NodeHandle node_handle,
                                   const VKCopyBufferCreateInfo &create_info) override
  {
    VersionedResource src_resource = resources.get_buffer(create_info.src_buffer);
    VersionedResource dst_resource = resources.get_buffer_and_increase_version(
        create_info.dst_buffer);
    dependencies.add_read_resource(
        node_handle, src_resource, VK_ACCESS_TRANSFER_READ_BIT, VK_IMAGE_LAYOUT_UNDEFINED);
    dependencies.add_write_resource(
        node_handle, dst_resource, VK_ACCESS_TRANSFER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED);
  }

  /**
   * Build the commands and add them to the command_buffer.
   */
  void build_commands(VKCommandBufferInterface &command_buffer,
                             const VKCopyBufferData &data,VKBoundPipelines &/*r_bound_pipelines*/) override
  {
    command_buffer.copy_buffer(data.src_buffer, data.dst_buffer, 1, &data.region);
  }
};
}  // namespace blender::gpu::render_graph
