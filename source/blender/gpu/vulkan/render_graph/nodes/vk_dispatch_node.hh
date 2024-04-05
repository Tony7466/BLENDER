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
#include "vk_types_pipeline.hh"

namespace blender::gpu::render_graph {
struct VKDispatchNode : NonCopyable {
  struct Data {
    VKPipelineData pipeline_data;
    uint32_t group_count_x;
    uint32_t group_count_y;
    uint32_t group_count_z;
  };

  /**
   * All information to add a new dispatch node to the render graph.
   */
  struct CreateInfo : NonCopyable {
    VKDispatchNode::Data dispatch_node;
    VKResourceAccessInfo resources;
  };

  static constexpr bool uses_image_resources = true;
  static constexpr bool uses_buffer_resources = true;
  static constexpr VkPipelineStageFlags pipeline_stage = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
  static constexpr VKNodeType node_type = VKNodeType::DISPATCH;

  template<typename Node> static void set_node_data(Node & /*node*/, const Data & /*data*/)
  {
    NOT_YET_IMPLEMENTED
  }

  template<typename Node> static void free_data(Node &node)
  {
    vk_pipeline_free_data(node.dispatch.pipeline_data);
  }

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
