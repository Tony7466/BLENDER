/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_node_class.hh"

namespace blender::gpu::render_graph {
struct VKCopyBufferToImageData {
  VkBuffer src_buffer;
  VkImage dst_image;
  VkBufferImageCopy region;
};
using VKCopyBufferToImageCreateInfo = VKCopyBufferToImageData;
class VKCopyBufferToImageNode
    : public VKNodeClass<VKNodeType::COPY_BUFFER_TO_IMAGE,
                         VKCopyBufferToImageCreateInfo,
                         VKCopyBufferToImageData,
                         VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VKResourceType::IMAGE | VKResourceType::BUFFER> {
 public:
  template<typename Node>
  static void set_node_data(Node &node, const VKCopyBufferToImageCreateInfo &create_info)
  {
    node.copy_buffer_to_image = create_info;
  }

  void build_resource_dependencies(VKResources &resources,
                                   VKResourceDependencies &dependencies,
                                   NodeHandle node_handle,
                                   const VKCopyBufferToImageCreateInfo &create_info) override
  {
    VersionedResource src_resource = resources.get_buffer(create_info.src_buffer);
    VersionedResource dst_resource = resources.get_image_and_increase_version(
        create_info.dst_image);
    dependencies.add_read_resource(
        node_handle, src_resource, VK_ACCESS_TRANSFER_READ_BIT, VK_IMAGE_LAYOUT_UNDEFINED);
    dependencies.add_write_resource(node_handle,
                                    dst_resource,
                                    VK_ACCESS_TRANSFER_WRITE_BIT,
                                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
  }

  void build_commands(VKCommandBufferInterface &command_buffer,
                             const VKCopyBufferToImageData &data,VKBoundPipelines &/*r_bound_pipelines*/) override
  {
    command_buffer.copy_buffer_to_image(
        data.src_buffer, data.dst_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &data.region);
  }
};
}  // namespace blender::gpu::render_graph
