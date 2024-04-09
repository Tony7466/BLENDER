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
#include "vk_node_class.hh"

namespace blender::gpu::render_graph {
struct VKCopyImageData {
  VkImage src_image;
  VkImage dst_image;
  VkImageCopy region;
};
using VKCopyImageCreateInfo = VKCopyImageData;
class VKCopyImageNode : public VKNodeClass<VKNodeType::COPY_IMAGE,
                                           VKCopyImageCreateInfo,
                                           VKCopyImageData,
                                           VK_PIPELINE_STAGE_TRANSFER_BIT,
                                           VKResourceType::IMAGE> {
 public:
  template<typename Node>
  static void set_node_data(Node &node, const VKCopyImageCreateInfo &create_info)
  {
    node.copy_image = create_info;
  }

  void build_resource_dependencies(VKResources &resources,
                                   VKResourceDependencies &dependencies,
                                   NodeHandle node_handle,
                                   const VKCopyImageCreateInfo &create_info) override
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

  void build_commands(VKCommandBufferInterface &command_buffer, const VKCopyImageData &data,VKBoundPipelines &/*r_bound_pipelines*/)override
  {
    command_buffer.copy_image(data.src_image,
                              VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                              data.dst_image,
                              VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                              1,
                              &data.region);
  }
};
}  // namespace blender::gpu::render_graph
