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
struct VKSynchronizationNode : NonCopyable {
  struct Data {};

  static constexpr bool uses_image_resources = true;
  static constexpr bool uses_buffer_resources = true;
  static constexpr VkPipelineStageFlags pipeline_stage = VK_PIPELINE_STAGE_NONE;
  static constexpr VKNodeType node_type = VKNodeType::SYNCHRONIZATION;

  template<typename Node> static void set_node_data(Node &node, const Data &data)
  {
    node.synchronization = data;
  }

  template<typename Node> static void free_data(Node & /*node*/) {}

  static void build_resource_dependencies(VKResources &resources,
                                          VKResourceDependencies &dependencies,
                                          NodeHandle node_handle,
                                          const Data &data)
  {
    NOT_YET_IMPLEMENTED
  }

  static void build_commands(VKCommandBufferInterface &command_buffer, const Data &data)
  {
    UNUSED_VARS(command_buffer, data);
    /* Intentionally left empty: A pipeline barrier has already been send to the command buffer. */
  }
};
}  // namespace blender::gpu::render_graph
