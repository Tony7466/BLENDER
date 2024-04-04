/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_utility_mixins.hh"

namespace blender::gpu::render_graph {

/**
 * State of a resource for pipeline barrier building.
 *
 * NOTE: write_access_ and read_access_ are mutual exclusive. Only one can be filled at a time.
 * NOTE: write_stages_ and read_stages_ are mutual exclusive. Only one can be filled at a time.
 */
struct VKResourceBarrierState {
  /* How was the resource accessed when last written to. */
  VkAccessFlags write_access = VK_ACCESS_NONE;
  /* How is the resource currently been read from. */
  VkAccessFlags read_access = VK_ACCESS_NONE;
  /* Pipeline stage that created wrote last to the resource. */
  VkPipelineStageFlags write_stages = VK_PIPELINE_STAGE_NONE;
  /* Pipeline stage that is currently reading from the resource. */
  VkPipelineStageFlags read_stages = VK_PIPELINE_STAGE_NONE;
  /* Current image layout of the image resource. */
  VkImageLayout image_layout = VK_IMAGE_LAYOUT_UNDEFINED;
};

/**
 * Container for storing shader descriptor set and push constants.
 */
struct VKPipelineData {
  VkPipeline vk_pipeline;
  VkPipelineLayout vk_pipeline_layout;
  VkDescriptorSet vk_descriptor_set;
  uint32_t push_constants_size;
  const void *push_constants_data;
};

/**
 * Data kept inside a dispatch node.
 */
struct VKDispatchNode {
  VKPipelineData pipeline_data;
  uint32_t group_count_x;
  uint32_t group_count_y;
  uint32_t group_count_z;
};

constexpr VkAccessFlags VK_ACCESS_READ_MASK = VK_ACCESS_INDIRECT_COMMAND_READ_BIT |
                                              VK_ACCESS_INDEX_READ_BIT |
                                              VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT |
                                              VK_ACCESS_UNIFORM_READ_BIT |
                                              VK_ACCESS_INPUT_ATTACHMENT_READ_BIT |
                                              VK_ACCESS_SHADER_READ_BIT |
                                              VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                              VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                                              VK_ACCESS_TRANSFER_READ_BIT |
                                              VK_ACCESS_HOST_READ_BIT;

constexpr VkAccessFlags VK_ACCESS_WRITE_MASK = VK_ACCESS_SHADER_WRITE_BIT |
                                               VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                                               VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT |
                                               VK_ACCESS_TRANSFER_WRITE_BIT |
                                               VK_ACCESS_HOST_WRITE_BIT;

struct VKImageAccess {
  VkImage vk_image;
  VkAccessFlags vk_access_flags;
};

struct VKBufferAccess {
  VkBuffer vk_buffer;
  VkAccessFlags vk_access_flags;
};

struct VKResourceAccessInfo : NonCopyable {
  Vector<VKBufferAccess> buffers;
  Vector<VKImageAccess> images;

  void clear()
  {
    buffers.clear();
    images.clear();
  }
};

/**
 * All information to add a dispatch node.
 */
struct VKDispatchInfo : NonCopyable {
  VKDispatchNode dispatch_node;
  VKResourceAccessInfo resources;
};

}  // namespace blender::gpu::render_graph
