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
struct VKBlitImageData {
  VkImage src_image;
  VkImage dst_image;
  VkImageBlit region;
  VkFilter filter;
};

/**
 * Information needed to add a node to the render graph.
 */
using VKBlitImageCreateInfo = VKBlitImageData;

/**
 * Blit Image Node
 *
 * - Contains logic to copy relevant data to the VKNodeData.
 * - Determine read/write resource dependencies.
 * - Add commands to a command builder.
 */
class VKBlitImageNode : public VKNodeClass<VKNodeType::BLIT_IMAGE,
                                           VKBlitImageCreateInfo,
                                           VKBlitImageData,
                                           VK_PIPELINE_STAGE_TRANSFER_BIT,
                                           VKResourceType::IMAGE> {
 public:
  /**
   * Update the node data with the data inside create_info.
   *
   * Has been implemented as a template to ensure all node specific data
   * (`VK*Data`/`VK*CreateInfo`) types can be included in the same header file as the logic. The
   * actual node data (`VKNodeData` includes all header files.)
   */
  template<typename Node> void set_node_data(Node &node, const VKBlitImageCreateInfo &create_info)
  {
    node.blit_image = create_info;
  }

  /**
   * Extract read/write resource dependencies from `create_info` and add them to `dependencies`.
   */
  void build_resource_dependencies(VKResources &resources,
                                   VKResourceDependencies &dependencies,
                                   NodeHandle node_handle,
                                   const VKBlitImageCreateInfo &create_info) override
  {
    VersionedResource src_resource = resources.get_image(create_info.src_image);
    VersionedResource dst_resource = resources.get_image_and_increase_version(
        create_info.dst_image);
    dependencies.add_read_resource(node_handle,
                                   src_resource,
                                   VK_ACCESS_TRANSFER_READ_BIT,
                                   VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    dependencies.add_write_resource(node_handle,
                                    dst_resource,
                                    VK_ACCESS_TRANSFER_WRITE_BIT,
                                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
  }

  /**
   * Build the commands and add them to the command_buffer.
   */
  void build_commands(VKCommandBufferInterface &command_buffer,
                      const VKBlitImageData &data,
                      VKBoundPipelines & /*r_bound_pipelines*/) override
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
