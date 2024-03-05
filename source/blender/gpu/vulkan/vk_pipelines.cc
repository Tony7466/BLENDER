/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_pipelines.hh"
#include "vk_backend.hh"
#include "vk_memory.hh"

namespace blender::gpu {

VKPipelines::VKPipelines()
{
  /* Initialize VkComputePipelineCreateInfo*/
  vk_compute_pipeline_create_info_.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  vk_compute_pipeline_create_info_.pNext = nullptr;
  vk_compute_pipeline_create_info_.flags = 0;
  vk_compute_pipeline_create_info_.layout = VK_NULL_HANDLE;
  vk_compute_pipeline_create_info_.basePipelineHandle = VK_NULL_HANDLE;
  vk_compute_pipeline_create_info_.basePipelineIndex = 0;
  VkPipelineShaderStageCreateInfo &vk_pipeline_shader_stage_create_info =
      vk_compute_pipeline_create_info_.stage;
  vk_pipeline_shader_stage_create_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  vk_pipeline_shader_stage_create_info.pNext = nullptr;
  vk_pipeline_shader_stage_create_info.flags = 0;
  vk_pipeline_shader_stage_create_info.stage = VK_SHADER_STAGE_COMPUTE_BIT;
  vk_pipeline_shader_stage_create_info.module = VK_NULL_HANDLE;
  vk_pipeline_shader_stage_create_info.pName = "main";

  /* Initialize VkSpecializationInfo. */
  vk_specialization_info_.mapEntryCount = 0;
  vk_specialization_info_.pMapEntries = nullptr;
  vk_specialization_info_.dataSize = 0;
  vk_specialization_info_.pData = nullptr;
}

VkPipeline VKPipelines::get_or_create_compute_pipeline(ComputeInfo &compute_info)
{
#if 0
    const VkPipeline *found_pipeline = compute_pipelines_.lookup_ptr(compute_info);
    if (found_pipeline) {
        VkPipeline result = *found_pipeline;
        BLI_assert(result != VK_NULL_HANDLE);
        return result;
    }
#endif

  vk_compute_pipeline_create_info_.layout = compute_info.layout;
  vk_compute_pipeline_create_info_.stage.module = compute_info.shader_module;
  if (compute_info.specialization_constants.is_empty()) {
    vk_compute_pipeline_create_info_.stage.pSpecializationInfo = nullptr;
  }
  else {
    vk_compute_pipeline_create_info_.stage.pSpecializationInfo = &vk_specialization_info_;
    vk_specialization_info_.dataSize = compute_info.specialization_constants.size() *
                                       sizeof(uint32_t);
    vk_specialization_info_.pData = compute_info.specialization_constants.data();
  }

  /* Build pipeline. */
  VKBackend &backend = VKBackend::get();
  VKDevice &device = backend.device_get();
  VK_ALLOCATION_CALLBACKS;

  VkPipeline pipeline = VK_NULL_HANDLE;
  vkCreateComputePipelines(device.device_get(),
                           device.vk_pipeline_cache_get(),
                           1,
                           &vk_compute_pipeline_create_info_,
                           vk_allocation_callbacks,
                           &pipeline);
#if 0
  compute_pipelines_.add(compute_info, pipeline);
#endif

  /* Reset values to initial value. */
  vk_compute_pipeline_create_info_.layout = VK_NULL_HANDLE;
  vk_compute_pipeline_create_info_.stage.module = VK_NULL_HANDLE;
  vk_specialization_info_.dataSize = 0;
  vk_specialization_info_.pData = nullptr;

  return pipeline;
}

void VKPipelines::deinitialize()
{
  // TODO: free all resources;
}

}  // namespace blender::gpu
