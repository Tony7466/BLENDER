/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_pipeline_pool.hh"
#include "vk_backend.hh"
#include "vk_memory.hh"

namespace blender::gpu {

VKPipelinePool::VKPipelinePool()
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

  /* Initialize VkGraphicsPipelineCreateInfo */
  vk_graphics_pipeline_create_info_ = {};
  vk_graphics_pipeline_create_info_.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  vk_graphics_pipeline_create_info_.pNext = nullptr;
  vk_graphics_pipeline_create_info_.stageCount = 0;
  vk_graphics_pipeline_create_info_.pStages = vk_pipeline_shader_stage_create_info_;

  /* Initialize VkPipelineShaderStageCreateInfo */
  for (int i : IndexRange(3)) {
    vk_pipeline_shader_stage_create_info_[i].sType =
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vk_pipeline_shader_stage_create_info_[i].pNext = nullptr;
    vk_pipeline_shader_stage_create_info_[i].flags = 0;
    vk_pipeline_shader_stage_create_info_[i].module = VK_NULL_HANDLE;
    vk_pipeline_shader_stage_create_info_[i].pName = "main";
  }
  vk_pipeline_shader_stage_create_info_[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
  vk_pipeline_shader_stage_create_info_[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  vk_pipeline_shader_stage_create_info_[2].stage = VK_SHADER_STAGE_GEOMETRY_BIT;

  /* Initialize VkSpecializationInfo. */
  vk_specialization_info_.mapEntryCount = 0;
  vk_specialization_info_.pMapEntries = nullptr;
  vk_specialization_info_.dataSize = 0;
  vk_specialization_info_.pData = nullptr;

  vk_push_constant_range_.stageFlags = 0;
  vk_push_constant_range_.offset = 0;
  vk_push_constant_range_.size = 0;
}

VkPipeline VKPipelinePool::get_or_create_compute_pipeline(VKComputeInfo &compute_info,
                                                          VkPipeline vk_pipeline_base)
{
  std::scoped_lock lock(mutex_);
  const VkPipeline *found_pipeline = compute_pipelines_.lookup_ptr(compute_info);
  if (found_pipeline) {
    VkPipeline result = *found_pipeline;
    BLI_assert(result != VK_NULL_HANDLE);
    return result;
  }

  vk_compute_pipeline_create_info_.layout = compute_info.vk_pipeline_layout;
  vk_compute_pipeline_create_info_.stage.module = compute_info.vk_shader_module;
  vk_compute_pipeline_create_info_.basePipelineHandle = vk_pipeline_base;
  if (compute_info.specialization_constants.is_empty()) {
    vk_compute_pipeline_create_info_.stage.pSpecializationInfo = nullptr;
  }
  else {
    while (vk_specialization_map_entries_.size() < compute_info.specialization_constants.size()) {
      uint32_t constant_id = vk_specialization_map_entries_.size();
      VkSpecializationMapEntry vk_specialization_map_entry = {};
      vk_specialization_map_entry.constantID = constant_id;
      vk_specialization_map_entry.offset = constant_id * sizeof(uint32_t);
      vk_specialization_map_entry.size = sizeof(uint32_t);
      vk_specialization_map_entries_.append(vk_specialization_map_entry);
    }
    vk_compute_pipeline_create_info_.stage.pSpecializationInfo = &vk_specialization_info_;
    vk_specialization_info_.dataSize = compute_info.specialization_constants.size() *
                                       sizeof(uint32_t);
    vk_specialization_info_.pData = compute_info.specialization_constants.data();
    vk_specialization_info_.mapEntryCount = compute_info.specialization_constants.size();
    vk_specialization_info_.pMapEntries = vk_specialization_map_entries_.data();
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
  compute_pipelines_.add(compute_info, pipeline);

  /* Reset values to initial value. */
  vk_compute_pipeline_create_info_.layout = VK_NULL_HANDLE;
  vk_compute_pipeline_create_info_.stage.module = VK_NULL_HANDLE;
  vk_compute_pipeline_create_info_.stage.pSpecializationInfo = nullptr;
  vk_compute_pipeline_create_info_.basePipelineHandle = VK_NULL_HANDLE;
  vk_specialization_info_.dataSize = 0;
  vk_specialization_info_.pData = nullptr;
  vk_specialization_info_.mapEntryCount = 0;
  vk_specialization_info_.pMapEntries = nullptr;

  return pipeline;
}

VkPipeline VKPipelinePool::get_or_create_graphics_pipeline(VKGraphicsInfo &graphics_info,
                                                           VkPipeline vk_pipeline_base)
{
  std::scoped_lock lock(mutex_);
  const VkPipeline *found_pipeline = graphic_pipelines_.lookup_ptr(graphics_info);
  if (found_pipeline) {
    VkPipeline result = *found_pipeline;
    BLI_assert(result != VK_NULL_HANDLE);
    return result;
  }

  /* Specialization constants */
  VkSpecializationInfo *specialization_info = nullptr;
  if (!graphics_info.specialization_constants.is_empty()) {
    specialization_info = &vk_specialization_info_;
    while (vk_specialization_map_entries_.size() < graphics_info.specialization_constants.size()) {
      uint32_t constant_id = vk_specialization_map_entries_.size();
      VkSpecializationMapEntry vk_specialization_map_entry = {};
      vk_specialization_map_entry.constantID = constant_id;
      vk_specialization_map_entry.offset = constant_id * sizeof(uint32_t);
      vk_specialization_map_entry.size = sizeof(uint32_t);
      vk_specialization_map_entries_.append(vk_specialization_map_entry);
    }
    vk_compute_pipeline_create_info_.stage.pSpecializationInfo = &vk_specialization_info_;
    vk_specialization_info_.dataSize = graphics_info.specialization_constants.size() *
                                       sizeof(uint32_t);
    vk_specialization_info_.pData = graphics_info.specialization_constants.data();
    vk_specialization_info_.mapEntryCount = graphics_info.specialization_constants.size();
    vk_specialization_info_.pMapEntries = vk_specialization_map_entries_.data();
  }

  /* Shader stages */
  vk_graphics_pipeline_create_info_.stageCount =
      graphics_info.pre_rasterization.vk_geometry_module == VK_NULL_HANDLE ? 2 : 3;
  vk_pipeline_shader_stage_create_info_[0].module =
      graphics_info.pre_rasterization.vk_vertex_module;
  vk_pipeline_shader_stage_create_info_[0].pSpecializationInfo = specialization_info;
  vk_pipeline_shader_stage_create_info_[1].module =
      graphics_info.fragment_shader.vk_fragment_module;
  vk_pipeline_shader_stage_create_info_[1].pSpecializationInfo = specialization_info;
  vk_pipeline_shader_stage_create_info_[2].module =
      graphics_info.pre_rasterization.vk_geometry_module;
  vk_pipeline_shader_stage_create_info_[2].pSpecializationInfo = specialization_info;

  /* Common values */
  vk_graphics_pipeline_create_info_.layout = graphics_info.vk_pipeline_layout;
  vk_graphics_pipeline_create_info_.basePipelineHandle = vk_pipeline_base;

  /* Build pipeline. */
  VKBackend &backend = VKBackend::get();
  VKDevice &device = backend.device_get();
  VK_ALLOCATION_CALLBACKS;

  VkPipeline pipeline = VK_NULL_HANDLE;
  vkCreateGraphicsPipelines(device.device_get(),
                            device.vk_pipeline_cache_get(),
                            1,
                            &vk_graphics_pipeline_create_info_,
                            vk_allocation_callbacks,
                            &pipeline);
  graphic_pipelines_.add(graphics_info, pipeline);

  /* Reset values to initial value. */
  vk_specialization_info_.dataSize = 0;
  vk_specialization_info_.pData = nullptr;
  vk_specialization_info_.mapEntryCount = 0;
  vk_specialization_info_.pMapEntries = nullptr;
  vk_graphics_pipeline_create_info_.stageCount = 0;
  vk_graphics_pipeline_create_info_.layout = VK_NULL_HANDLE;
  vk_graphics_pipeline_create_info_.basePipelineHandle = VK_NULL_HANDLE;
  for (VkPipelineShaderStageCreateInfo &info :
       MutableSpan<VkPipelineShaderStageCreateInfo>(vk_pipeline_shader_stage_create_info_, 3))
  {
    info.module = VK_NULL_HANDLE;
    info.pSpecializationInfo = nullptr;
  }

  return pipeline;
}

void VKPipelinePool::remove(Span<VkShaderModule> vk_shader_modules)
{
  std::scoped_lock lock(mutex_);
  Vector<VkPipeline> pipelines_to_destroy;
  compute_pipelines_.remove_if([&](auto item) {
    if (vk_shader_modules.contains(item.key.vk_shader_module)) {
      pipelines_to_destroy.append(item.value);
      return true;
    }
    return false;
  });

  VKDevice &device = VKBackend::get().device_get();
  VK_ALLOCATION_CALLBACKS;
  for (VkPipeline vk_pipeline : pipelines_to_destroy) {
    vkDestroyPipeline(device.device_get(), vk_pipeline, vk_allocation_callbacks);
  }
}

void VKPipelinePool::free_data()
{
  std::scoped_lock lock(mutex_);
  VKDevice &device = VKBackend::get().device_get();
  VK_ALLOCATION_CALLBACKS;
  for (VkPipeline &vk_pipeline : compute_pipelines_.values()) {
    vkDestroyPipeline(device.device_get(), vk_pipeline, vk_allocation_callbacks);
  }
  compute_pipelines_.clear();
}

}  // namespace blender::gpu
