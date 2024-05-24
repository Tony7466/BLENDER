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
  vk_graphics_pipeline_create_info_.pNext = &vk_pipeline_rendering_create_info_;
  vk_graphics_pipeline_create_info_.stageCount = 0;
  vk_graphics_pipeline_create_info_.pStages = vk_pipeline_shader_stage_create_info_;
  vk_graphics_pipeline_create_info_.pInputAssemblyState =
      &vk_pipeline_input_assembly_state_create_info_;
  vk_graphics_pipeline_create_info_.pVertexInputState =
      &vk_pipeline_vertex_input_state_create_info_;
  vk_graphics_pipeline_create_info_.pRasterizationState =
      &vk_pipeline_rasterization_state_create_info_;
  vk_graphics_pipeline_create_info_.pViewportState = &vk_pipeline_viewport_state_create_info_;
  vk_graphics_pipeline_create_info_.pMultisampleState =
      &vk_pipeline_multisample_state_create_info_;

  /* Initialize VkPipelineRenderingCreateInfo */
  vk_pipeline_rendering_create_info_ = {};
  vk_pipeline_rendering_create_info_.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;

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

  /* Initialize VkPipelineInputAssemblyStateCreateInfo */
  vk_pipeline_input_assembly_state_create_info_ = {};
  vk_pipeline_input_assembly_state_create_info_.sType =
      VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;

  /* Initialize VkPipelineVertexInputStateCreateInfo */
  vk_pipeline_vertex_input_state_create_info_ = {};
  vk_pipeline_vertex_input_state_create_info_.sType =
      VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

  /* Initialize VkPipelineRasterizationStateCreateInfo */
  vk_pipeline_rasterization_state_create_info_ = {};
  vk_pipeline_rasterization_state_create_info_.sType =
      VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  vk_pipeline_rasterization_state_create_info_.lineWidth = 1.0f;
  vk_pipeline_rasterization_state_create_info_.frontFace = VK_FRONT_FACE_CLOCKWISE;

  vk_pipeline_viewport_state_create_info_ = {};
  vk_pipeline_viewport_state_create_info_.sType =
      VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;

  /* Initialize VkPipelineMultisampleStateCreateInfo */
  vk_pipeline_multisample_state_create_info_ = {};
  vk_pipeline_multisample_state_create_info_.sType =
      VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  vk_pipeline_multisample_state_create_info_.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
  vk_pipeline_multisample_state_create_info_.minSampleShading = 1.0f;

  /* Initialize VkSpecializationInfo. */
  vk_specialization_info_.mapEntryCount = 0;
  vk_specialization_info_.pMapEntries = nullptr;
  vk_specialization_info_.dataSize = 0;
  vk_specialization_info_.pData = nullptr;

  vk_push_constant_range_.stageFlags = 0;
  vk_push_constant_range_.offset = 0;
  vk_push_constant_range_.size = 0;
}

VkSpecializationInfo *VKPipelinePool::specialization_info_update(
    Span<shader::ShaderCreateInfo::SpecializationConstant::Value> specialization_constants)
{
  if (specialization_constants.is_empty()) {
    return nullptr;
  }

  while (vk_specialization_map_entries_.size() < specialization_constants.size()) {
    uint32_t constant_id = vk_specialization_map_entries_.size();
    VkSpecializationMapEntry vk_specialization_map_entry = {};
    vk_specialization_map_entry.constantID = constant_id;
    vk_specialization_map_entry.offset = constant_id * sizeof(uint32_t);
    vk_specialization_map_entry.size = sizeof(uint32_t);
    vk_specialization_map_entries_.append(vk_specialization_map_entry);
  }
  vk_specialization_info_.dataSize = specialization_constants.size() * sizeof(uint32_t);
  vk_specialization_info_.pData = specialization_constants.data();
  vk_specialization_info_.mapEntryCount = specialization_constants.size();
  vk_specialization_info_.pMapEntries = vk_specialization_map_entries_.data();
  return &vk_specialization_info_;
}

void VKPipelinePool::specialization_info_reset()
{
  vk_specialization_info_.dataSize = 0;
  vk_specialization_info_.pData = nullptr;
  vk_specialization_info_.mapEntryCount = 0;
  vk_specialization_info_.pMapEntries = nullptr;
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
  vk_compute_pipeline_create_info_.stage.pSpecializationInfo = specialization_info_update(
      compute_info.specialization_constants);

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
  specialization_info_reset();

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
  VkSpecializationInfo *specialization_info = specialization_info_update(
      graphics_info.specialization_constants);

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

  /* Input assembly */
  vk_pipeline_input_assembly_state_create_info_.topology = graphics_info.vertex_in.vk_topology;
  vk_pipeline_input_assembly_state_create_info_.primitiveRestartEnable =
      ELEM(graphics_info.vertex_in.vk_topology,
           VK_PRIMITIVE_TOPOLOGY_POINT_LIST,
           VK_PRIMITIVE_TOPOLOGY_LINE_LIST,
           VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,
           VK_PRIMITIVE_TOPOLOGY_LINE_LIST_WITH_ADJACENCY) ?
          VK_FALSE :
          VK_TRUE;
  vk_pipeline_vertex_input_state_create_info_.pVertexAttributeDescriptions =
      graphics_info.vertex_in.attributes.data();
  vk_pipeline_vertex_input_state_create_info_.vertexAttributeDescriptionCount =
      graphics_info.vertex_in.attributes.size();
  vk_pipeline_vertex_input_state_create_info_.pVertexBindingDescriptions =
      graphics_info.vertex_in.bindings.data();
  vk_pipeline_vertex_input_state_create_info_.vertexBindingDescriptionCount =
      graphics_info.vertex_in.bindings.size();

  /* Rasterization state */
  vk_pipeline_rasterization_state_create_info_.frontFace = graphics_info.state.invert_facing ?
                                                               VK_FRONT_FACE_COUNTER_CLOCKWISE :
                                                               VK_FRONT_FACE_CLOCKWISE;
  vk_pipeline_rasterization_state_create_info_.cullMode = to_vk_cull_mode_flags(
      static_cast<eGPUFaceCullTest>(graphics_info.state.culling_test));
  if (graphics_info.state.shadow_bias) {
    vk_pipeline_rasterization_state_create_info_.depthBiasEnable = VK_TRUE;
    vk_pipeline_rasterization_state_create_info_.depthBiasSlopeFactor = 2.0f;
    vk_pipeline_rasterization_state_create_info_.depthBiasConstantFactor = 1.0f;
    vk_pipeline_rasterization_state_create_info_.depthBiasClamp = 0.0f;
  }
  else {
    vk_pipeline_rasterization_state_create_info_.depthBiasEnable = VK_FALSE;
  }

  /* Viewport state */
  vk_pipeline_viewport_state_create_info_.pViewports =
      graphics_info.fragment_shader.viewports.data();
  vk_pipeline_viewport_state_create_info_.viewportCount =
      graphics_info.fragment_shader.viewports.size();
  vk_pipeline_viewport_state_create_info_.pScissors =
      graphics_info.fragment_shader.scissors.data();
  vk_pipeline_viewport_state_create_info_.scissorCount =
      graphics_info.fragment_shader.scissors.size();

  /* Common values */
  vk_graphics_pipeline_create_info_.layout = graphics_info.vk_pipeline_layout;
  // TODO: based on `vk_pipeline_base` we should update the flags.
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
  specialization_info_reset();
  vk_graphics_pipeline_create_info_.stageCount = 0;
  vk_graphics_pipeline_create_info_.layout = VK_NULL_HANDLE;
  vk_graphics_pipeline_create_info_.basePipelineHandle = VK_NULL_HANDLE;
  for (VkPipelineShaderStageCreateInfo &info :
       MutableSpan<VkPipelineShaderStageCreateInfo>(vk_pipeline_shader_stage_create_info_, 3))
  {
    info.module = VK_NULL_HANDLE;
    info.pSpecializationInfo = nullptr;
  }
  vk_pipeline_input_assembly_state_create_info_.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
  vk_pipeline_input_assembly_state_create_info_.primitiveRestartEnable = VK_TRUE;
  vk_pipeline_vertex_input_state_create_info_.pVertexAttributeDescriptions = nullptr;
  vk_pipeline_vertex_input_state_create_info_.vertexAttributeDescriptionCount = 0;
  vk_pipeline_vertex_input_state_create_info_.pVertexBindingDescriptions = nullptr;
  vk_pipeline_vertex_input_state_create_info_.vertexBindingDescriptionCount = 0;
  vk_pipeline_rasterization_state_create_info_.frontFace = VK_FRONT_FACE_CLOCKWISE;
  vk_pipeline_rasterization_state_create_info_.cullMode = VK_CULL_MODE_NONE;
  vk_pipeline_rasterization_state_create_info_.depthBiasEnable = VK_FALSE;
  vk_pipeline_rasterization_state_create_info_.depthBiasSlopeFactor = 0.0f;
  vk_pipeline_rasterization_state_create_info_.depthBiasConstantFactor = 0.0f;
  vk_pipeline_rasterization_state_create_info_.depthBiasClamp = 0.0f;
  vk_pipeline_viewport_state_create_info_.pScissors = nullptr;
  vk_pipeline_viewport_state_create_info_.scissorCount = 0;
  vk_pipeline_viewport_state_create_info_.pViewports = nullptr;
  vk_pipeline_viewport_state_create_info_.viewportCount = 0;

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
