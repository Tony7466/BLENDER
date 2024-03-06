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
 * Pipelines are lazy initialized and same pipelines should share their handle.
 *
 * To improve performance we want to keep track of pipelines globally. Same pipeline should share
 * the same VKPipeline. This makes it easier to detect if pipelines are actually changed.
 *
 * VkPipelineCache is normally used to share internal resources of pipelines. However it the
 * responsibility of the driver how this is handled. Some drivers might do ref-counting other may
 * return a differnent pipeline handle. Due to this we cannot rely on the pipeline cache to merge
 * same pipelines.
 *
 * Many information is needed to create a graphics pipeline. Some of the information is
 * boilerplating; or at least from Blender point of view. To improve lookup performance we use a
 * slimmed down version of the pipeline create info structs. The idea is that we can limit the
 * required data because we control which data we actually use, removing te boiler plating and
 * improve hashing performance. See #VKPipelines::ComputeInfo.
 */
// TODO: Make thread safe by adding a mutex.
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
  /**
   * Get an existing or create a new compute pipeline based on the provided ComputeInfo.
   */
  VkPipeline get_or_create_compute_pipeline(ComputeInfo &compute_info);

  /**
   * Destroy all created pipelines.
   */
  void deinitialize();
};

}  // namespace blender::gpu
