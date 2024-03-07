/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_utility_mixins.hh"

namespace blender::gpu {

struct VKDescriptorSetData {
  VkPipelineLayout vk_pipeline_layout;
  VkDescriptorSet vk_descriptor_set;
};

/**
 * Data kept inside a dispatch node.
 */
struct VKDispatchNode {
  VkPipeline vk_pipeline;
  uint32_t group_count_x;
  uint32_t group_count_y;
  uint32_t group_count_z;
  VKDescriptorSetData descriptor_set;
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

}  // namespace blender::gpu
