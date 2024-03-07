/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"
#include "vk_render_graph_list.hh"

namespace blender {
namespace gpu {
struct VKComputeInfo {
  VkShaderModule vk_shader_module;
  VkPipelineLayout vk_pipeline_layout;
  Vector<shader::ShaderCreateInfo::SpecializationConstant::Value> specialization_constants;

  bool operator==(const VKComputeInfo &other) const
  {
    return vk_shader_module == other.vk_shader_module &&
           vk_pipeline_layout == other.vk_pipeline_layout &&
           specialization_constants == other.specialization_constants;
  };
};

}  // namespace gpu

/* TODO: this is duplicated from gl_shader.hh, perhaps move it to create info. ß*/
template<>
struct DefaultHash<Vector<gpu::shader::ShaderCreateInfo::SpecializationConstant::Value>> {
  uint64_t operator()(
      const Vector<gpu::shader::ShaderCreateInfo::SpecializationConstant::Value> &key) const
  {
    uint64_t hash = 0;
    for (const gpu::shader::ShaderCreateInfo::SpecializationConstant::Value &value : key) {
      hash = hash * 33 + value.u;
    }
    return hash;
  }
};

template<> struct DefaultHash<gpu::VKComputeInfo> {
  uint64_t operator()(const gpu::VKComputeInfo &key) const
  {
    uint64_t hash = uint64_t(key.vk_shader_module);
    hash = hash * 33 ^ uint64_t(key.vk_pipeline_layout);
    hash = hash * 33 ^ get_default_hash(key.specialization_constants);
    return hash;
  }
};

namespace gpu {

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
 * return a different pipeline handle. Due to this we cannot rely on the pipeline cache to merge
 * same pipelines.
 *
 * Many information is needed to create a graphics pipeline. Some of the information is
 * boilerplate; or at least from Blender point of view. To improve lookup performance we use a
 * slimmed down version of the pipeline create info structs. The idea is that we can limit the
 * required data because we control which data we actually use, removing te boiler plating and
 * improve hashing performance. See #VKPipelines::ComputeInfo.
 */
// TODO: Make thread safe by adding a mutex.
class VKPipelines {
 public:
 private:
  Map<VKComputeInfo, VkPipeline> compute_pipelines_;
  /* Partially initialized structures to reuse. */
  VkComputePipelineCreateInfo vk_compute_pipeline_create_info_;
  VkSpecializationInfo vk_specialization_info_;

  std::mutex mutex_;

  struct GraphicsPipelineInfo {};

 public:
  VKPipelines();
  /**
   * Get an existing or create a new compute pipeline based on the provided ComputeInfo.
   *
   * When vk_pipeline_base is a valid pipeline handle, the pipeline base will be used to speed up
   * pipeline creation process.
   */
  VkPipeline get_or_create_compute_pipeline(VKComputeInfo &compute_info,
                                            VkPipeline vk_pipeline_base = VK_NULL_HANDLE);

  /**
   * Remove all shader pipelines that uses the given shader_module.
   */
  // TODO: When shaders are destroyed, the shader modules should be removed.
  // Might require a ref counting system when we want to share shader modules as well.
  void remove(Span<VkShaderModule> vk_shader_modules);

  /**
   * Destroy all created pipelines.
   */
  void deinit();
};

}  // namespace gpu

}  // namespace blender
