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

namespace blender::gpu::render_graph {
struct VKClearColorImageNode : NonCopyable {
  struct Data {
    VkImage vk_image;
    VkClearColorValue vk_clear_color_value;
    VkImageSubresourceRange vk_image_subresource_range;
  };

  static constexpr bool uses_image_resources = true;
  static constexpr bool uses_buffer_resources = false;
  static constexpr VkPipelineStageFlags pipeline_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
  static constexpr VKNodeType node_type = VKNodeType::CLEAR_COLOR_IMAGE;

  template<typename Node> static void set_node_data(Node & /*node*/, const Data & /*data*/) {}

  template<typename Node> static void free_data(Node & /*node*/) {}

  static void build_resource_dependencies(VKResources & /*resources*/,
                                          VKResourceDependencies & /*dependencies*/,
                                          NodeHandle /*node_handle*/,
                                          const Data & /*data*/)
  {
    NOT_YET_IMPLEMENTED
  }

  static void build_commands(VKCommandBufferInterface & /*command_buffer*/, const Data & /*data*/)
  {
    NOT_YET_IMPLEMENTED
  }
};
}  // namespace blender::gpu::render_graph
