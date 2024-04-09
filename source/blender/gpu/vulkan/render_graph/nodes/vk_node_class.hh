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
template<typename NodeCreateInfo,
         typename NodeData,
         VKNodeType NodeType,
         bool UseImages,
         bool UseBuffers,
         VkPipelineStageFlagBits PipelineStage>
class VKNodeClass : public NonCopyable {
 public:
  using CreateInfo = NodeCreateInfo;
  using Data = NodeData;

  static constexpr VKNodeType node_type = NodeType;
  static constexpr bool uses_image_resources = UseImages;
  static constexpr bool uses_buffer_resources = UseBuffers;
  static constexpr VkPipelineStageFlags pipeline_stage = PipelineStage;
};
}  // namespace blender::gpu::render_graph
