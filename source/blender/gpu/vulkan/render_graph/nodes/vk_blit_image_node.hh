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
struct VKBlitImageData {
  VkImage src_image;
  VkImage dst_image;
  VkImageBlit region;
  VkFilter filter;
};
using VKBlitImageCreateInfo = VKBlitImageData;

class VKBlitImageNode : public VKNodeClass<VKBlitImageCreateInfo,
                                           VKBlitImageData,
                                           VKNodeType::BLIT_IMAGE,
                                           true,
                                           false,
                                           VK_PIPELINE_STAGE_TRANSFER_BIT> {
 public:
  template<typename Node> static void set_node_data(Node &node, const CreateInfo &create_info)
  {
    node.blit_image = create_info;
  }

  static void build_resource_dependencies(VKResources &resources,
                                          VKResourceDependencies &dependencies,
                                          NodeHandle node_handle,
                                          const CreateInfo &create_info)
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
