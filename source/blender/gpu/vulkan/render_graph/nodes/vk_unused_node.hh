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
struct VKUnusedNode : NonCopyable {
  struct Data {};
  static constexpr bool uses_image_resources = false;
  static constexpr bool uses_buffer_resources = false;
  static constexpr VkPipelineStageFlags pipeline_stage = VK_PIPELINE_STAGE_NONE;
  static constexpr VKNodeType node_type = VKNodeType::UNUSED;

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
