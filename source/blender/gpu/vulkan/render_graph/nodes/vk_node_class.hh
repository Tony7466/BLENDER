/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "render_graph/vk_command_buffer_wrapper.hh"
#include "render_graph/vk_resource_dependencies.hh"
#include "render_graph/vk_resources.hh"
#include "vk_common.hh"
#include "vk_types_pipeline.hh"

namespace blender::gpu::render_graph {

enum class VKResourceType { IMAGE = (1 << 0), BUFFER = (1 << 1) };
ENUM_OPERATORS(VKResourceType, VKResourceType::BUFFER);

template<VKNodeType NodeType,
         typename NodeCreateInfo,
         typename NodeData,
         VkPipelineStageFlagBits PipelineStage,
         VKResourceType ResourceUsages>
class VKNodeClass : public NonCopyable {
 public:
  static constexpr VKNodeType node_type = NodeType;
  static constexpr VkPipelineStageFlags pipeline_stage = PipelineStage;
  static constexpr VKResourceType resource_usages = ResourceUsages;

  virtual void build_resource_dependencies(VKResources &resources,
                                           VKResourceDependencies &dependencies,
                                           NodeHandle node_handle,
                                           const NodeCreateInfo &create_info) = 0;

  virtual void build_commands(VKCommandBufferInterface &command_buffer,
                              const NodeData &data,
                              VKBoundPipelines &r_bound_pipelines) = 0;
};
}  // namespace blender::gpu::render_graph
