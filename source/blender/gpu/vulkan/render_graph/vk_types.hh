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
 * NOTE: write_access and read_access are mutual exclusive.
 * NOTE: write_stages and read_stages are mutual exclusive.
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

/**
 * Enum class containing the different resource types.
 */
enum class VKResourceType { IMAGE = (1 << 0), BUFFER = (1 << 1) };
ENUM_OPERATORS(VKResourceType, VKResourceType::BUFFER);

/**
 * Type of nodes of the render graph.
 */
enum class VKNodeType {
  UNUSED,
  CLEAR_COLOR_IMAGE,
  FILL_BUFFER,
  COPY_BUFFER,
  COPY_IMAGE,
  COPY_IMAGE_TO_BUFFER,
  COPY_BUFFER_TO_IMAGE,
  BLIT_IMAGE,
  DISPATCH,
  SYNCHRONIZATION,
};

/**
 * Index of a VKNodeData inside the render graph.
 */
using NodeHandle = uint64_t;

}  // namespace blender::gpu::render_graph
