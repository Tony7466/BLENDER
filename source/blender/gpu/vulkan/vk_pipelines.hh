/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"
#include "vk_render_graph_list.hh"

namespace blender::gpu {

class VKDevice;

using ComputePipelineHandle = int64_t;

/**
 * Pipelines are lazy initialized.
 */
// TODO: Add Mutex
class VKPipelines {
 public:
  /**
   * To reuse pipelines the key information about the pipeline are stored in these structs.
   * When requestion for a pipeline a pipeline handle will be returned. This pipeline handle
   * contains the handle to the actual pipeline.
   *
   * For compute shaders this isn't that relevant, but for graphics shaders the actual vertex
   * buffer layout can be added resulting in more batching more commands.
   */
  struct ComputeInfo {
    VkShaderModule shader_module;
    VkPipelineLayout layout;
    Vector<shader::ShaderCreateInfo::SpecializationConstant::Value> specialization_constants;
  };

 private:
  Map<ComputeInfo, VkPipeline> compute_pipelines_;
  /* Partially initialized structures to reuse. */
  VkComputePipelineCreateInfo vk_compute_pipeline_create_info_;
  VkSpecializationInfo vk_specialization_info_;

  struct GraphicsPipelineInfo {};

 public:
  VKPipelines();
  VkPipeline get_or_create_compute_pipeline(ComputeInfo &compute_info);
  void deinitialize();
};

}  // namespace blender::gpu
