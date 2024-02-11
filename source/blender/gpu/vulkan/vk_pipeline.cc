/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */
#include "BLI_timer.hh"

#include "vk_backend.hh"
#include "vk_batch.hh"
#include "vk_context.hh"
#include "vk_framebuffer.hh"
#include "vk_memory.hh"
#include "vk_pipeline.hh"
#include "vk_shader.hh"
#include "vk_state_manager.hh"
#include "vk_vertex_attribute_object.hh"

namespace blender::gpu {
#ifdef VK_STAT_PIPELINE_CACHE
static int pipeline_cache_global_count = 0;

static std::mutex mutex_cache_global;
void StateCache::set_name(std::string sh_name, std::string fb_name)
{
  std::unique_lock<std::mutex> lock(mutex_cache_global);
  BLI_assert(name_ == "" || sh_name == name_);
  name_ = sh_name;
  pipeline_cache_global_count++;
  BLI_info_always(
      "StateCache  %s  Layout %zu prim_types %zu  blend %u   line_smooth %u   write_mask %u   "
      "point_size  %u   line_width %u  stencil_write_mask %u vertex %zu  fragment %zu geometry "
      "%zu states %zu  mutable_states %zu  vertex_inputs %zu  framebufer %s\n",
      sh_name.c_str(),
      pipeline_layouts.size(),
      prim_types.size(),
      blend.size(),
      line_smooth.size(),
      write_mask.size(),
      point_size.size(),
      line_width.size(),
      stencil_write_mask.size(),
      vertex.size(),
      fragment.size(),
      geometry.size(),
      states.size(),
      mutable_states.size(),
      vertex_inputs.size(),
      fb_name.c_str());
  BLI_info_always("PipelineCreationGlobalCount %d\n", pipeline_cache_global_count);
}
void StateCache::append(VkPipelineLayout layout,
                        GPUPrimType type,
                        VkShaderModule vmodule,
                        VkShaderModule gmodule,
                        VkShaderModule fmodule,
                        const GPUState &state,
                        const GPUStateMutable &mutable_state,
                        const VkPipelineVertexInputStateCreateInfo &info)
{
  std::unique_lock<std::mutex> lock(mutex_cache_global);
  pipeline_layouts.add(layout);
  prim_types.add(type);
  vertex.add(vmodule);
  fragment.add(gmodule);
  geometry.add(fmodule);
  states.add(state.data);
  blend.add(state.blend);
  line_smooth.add(state.line_smooth);
  write_mask.add(state.write_mask);
  point_size.add(mutable_state.point_size);
  line_width.add(mutable_state.line_width);
  stencil_write_mask.add(mutable_state.stencil_write_mask);
  std::stringstream stream;
  for (int i = 0; i < 9; i++) {
    stream << std::setfill('0') << std::setw(20) << mutable_state.data[i];
  }
  mutable_states.add(stream.str());

  std::string bytes = "";
  bytes += encode_struct(info.pVertexBindingDescriptions, info.vertexBindingDescriptionCount);
  bytes += encode_struct(info.pVertexAttributeDescriptions, info.vertexAttributeDescriptionCount);
  if (info.pNext != VK_NULL_HANDLE) {
    auto *dev_info = reinterpret_cast<VkPipelineVertexInputDivisorStateCreateInfoEXT *>(
        const_cast<void *>(info.pNext));
    bytes += encode_struct(dev_info->pVertexBindingDivisors, dev_info->vertexBindingDivisorCount);
  }
  vertex_inputs.add(bytes);
}

StateCache &StateCacheMap::get(std::string fb_name)
{
  std::unique_lock<std::mutex> lock(mutex_cache_global);
  if (state_cache_map.count(fb_name) > 0) {
    return state_cache_map[fb_name];
  };
  state_cache_map[fb_name] = StateCache();
  return state_cache_map[fb_name];
}
#endif

VKPipeline::VKPipeline(VKPushConstants &&push_constants)
    : active_vk_pipeline_(VK_NULL_HANDLE), push_constants_(std::move(push_constants))
{
}
#ifdef VK_PIPELINE_REFACTOR
VKPipeline::VKPipeline(VkPipeline vk_pipeline, VKPushConstants &&push_constants, bool is_compute)
    : active_vk_pipeline_(vk_pipeline), push_constants_(std::move(push_constants))
{
  if (is_compute) {
    vk_pipelines_.append(vk_pipeline);
  }
}
#else
VKPipeline::VKPipeline(VkPipeline vk_pipeline, VKPushConstants &&push_constants)
    : active_vk_pipeline_(vk_pipeline), push_constants_(std::move(push_constants))
{
  vk_pipelines_.append(vk_pipeline);
}
#endif
VKPipeline::~VKPipeline()
{
  destroy();
}

void VKPipeline::destroy()
{

  VK_ALLOCATION_CALLBACKS
  const VKDevice &device = VKBackend::get().device_get();
  for (VkPipeline vk_pipeline : vk_pipelines_) {
    vkDestroyPipeline(device.device_get(), vk_pipeline, vk_allocation_callbacks);
  }
}

VKPipeline VKPipeline::create_compute_pipeline(
    VkShaderModule compute_module,
    VkPipelineLayout &pipeline_layout,
    const VKPushConstants::Layout &push_constants_layout,
    const VkSpecializationInfo *specialzation)
{
  VK_ALLOCATION_CALLBACKS
  const VKDevice &device = VKBackend::get().device_get();
  VkComputePipelineCreateInfo pipeline_info = {};
  pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipeline_info.flags = 0;
  pipeline_info.stage = {};
  pipeline_info.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  pipeline_info.stage.flags = 0;
  pipeline_info.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
  pipeline_info.stage.module = compute_module;
  pipeline_info.layout = pipeline_layout;
  pipeline_info.stage.pSpecializationInfo = specialzation;
  pipeline_info.stage.pName = "main";

  VkPipeline vk_pipeline;
  if (vkCreateComputePipelines(device.device_get(),
                               device.vk_pipeline_cache_get(),
                               1,
                               &pipeline_info,
                               vk_allocation_callbacks,
                               &vk_pipeline) != VK_SUCCESS)
  {
    return VKPipeline();
  }

  VKPushConstants push_constants(&push_constants_layout);

#ifdef VK_PIPELINE_REFACTOR
  return VKPipeline(vk_pipeline, std::move(push_constants), true);
#else
  return VKPipeline(vk_pipeline, std::move(push_constants));
#endif
}

VKPipeline VKPipeline::create_graphics_pipeline(
    const VKPushConstants::Layout &push_constants_layout)
{
  VKPushConstants push_constants(&push_constants_layout);
  return VKPipeline(std::move(push_constants));
}

VkPipeline VKPipeline::vk_handle() const
{
  return active_vk_pipeline_;
}

bool VKPipeline::is_valid() const
{
  return active_vk_pipeline_ != VK_NULL_HANDLE;
}

void VKPipeline::finalize(VKContext &context,
                          VkShaderModule vertex_module,
                          VkShaderModule geometry_module,
                          VkShaderModule fragment_module,
                          VkPipelineLayout &pipeline_layout,
                          const GPUPrimType prim_type,
                          const VKVertexAttributeObject &vertex_attribute_object)
{
  BLI_assert(vertex_module != VK_NULL_HANDLE);

  VK_ALLOCATION_CALLBACKS
  VKShader *shader = reinterpret_cast<VKShader *>(context.shader);
  shader->specialzation_ensure();
  VKFrameBuffer &framebuffer = *context.active_framebuffer_get();
  framebuffer.renderpass_ensure();
  VKPipelineStateManager &state_manager = state_manager_get();

#ifdef VK_PIPELINE_REFACTOR
  VKRenderPass &renderpass = framebuffer.render_pass_get();
  int type = 3;  // shader->cache_enable();
  if (type > 0) {
    switch (type) {
      case 1:
        active_vk_pipeline_ = renderpass.has_pipeline_cache(
            prim_type, state_manager.get_state(), 1);
        break;
      case 2:
        active_vk_pipeline_ = renderpass.has_pipeline_cache(prim_type,
                                                            state_manager.get_state(),
                                                            vertex_module,
                                                            geometry_module,
                                                            fragment_module,
                                                            2);
        break;
      case 3:
        active_vk_pipeline_ = renderpass.has_pipeline_cache(prim_type,
                                                            state_manager.get_state(),
                                                            vertex_module,
                                                            geometry_module,
                                                            fragment_module,
                                                            vertex_attribute_object.info,
                                                            3);
        break;
    }
    if (active_vk_pipeline_) {
      return;
    }
  }
#endif
#ifdef VK_STAT_PIPELINE_CACHE
  {
    auto name_ = std::string(framebuffer.name_get());
    auto num = std::to_string((uintptr_t)(&framebuffer));
    auto name = (name_.size() == 0) ? std::string("none") : name_;
    auto &state_cache = state_cache_map.get(name);
    state_cache.append(pipeline_layout,
                       prim_type,
                       vertex_module,
                       geometry_module,
                       fragment_module,
                       state_manager.get_state(),
                       state_manager.get_mutable_state(),
                       vertex_attribute_object.info);
    state_cache.set_name(shader->name_get(), name);
  }
#endif
  Vector<VkPipelineShaderStageCreateInfo> pipeline_stages;
  VkPipelineShaderStageCreateInfo vertex_stage_info = {};
  vertex_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  vertex_stage_info.stage = VK_SHADER_STAGE_VERTEX_BIT;
  vertex_stage_info.module = vertex_module;
  vertex_stage_info.pName = "main";
  auto specialization_info = shader->build_specialzation();
  vertex_stage_info.pSpecializationInfo = &specialization_info;
  pipeline_stages.append(vertex_stage_info);

  if (geometry_module != VK_NULL_HANDLE) {
    VkPipelineShaderStageCreateInfo geometry_stage_info = {};
    geometry_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    geometry_stage_info.stage = VK_SHADER_STAGE_GEOMETRY_BIT;
    geometry_stage_info.module = geometry_module;
    geometry_stage_info.pSpecializationInfo = &specialization_info;
    geometry_stage_info.pName = "main";
    pipeline_stages.append(geometry_stage_info);
  }

  if (fragment_module != VK_NULL_HANDLE) {
    VkPipelineShaderStageCreateInfo fragment_stage_info = {};
    fragment_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    fragment_stage_info.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragment_stage_info.module = fragment_module;
    fragment_stage_info.pSpecializationInfo = &specialization_info;
    fragment_stage_info.pName = "main";
    pipeline_stages.append(fragment_stage_info);
  }

  VkGraphicsPipelineCreateInfo pipeline_create_info = {};
  pipeline_create_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  pipeline_create_info.stageCount = pipeline_stages.size();
  pipeline_create_info.pStages = pipeline_stages.data();
  pipeline_create_info.layout = pipeline_layout;
  pipeline_create_info.renderPass = framebuffer.vk_render_pass_get();
  pipeline_create_info.subpass = framebuffer.subpass_current_get();

  /* Vertex input state. */
  VkPipelineVertexInputStateCreateInfo vertex_input_state = {};
  vertex_input_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
  vertex_input_state.vertexBindingDescriptionCount = vertex_attribute_object.bindings.size();
  vertex_input_state.pVertexBindingDescriptions = vertex_attribute_object.bindings.data();
  vertex_input_state.vertexAttributeDescriptionCount = vertex_attribute_object.attributes.size();
  vertex_input_state.pVertexAttributeDescriptions = vertex_attribute_object.attributes.data();
  pipeline_create_info.pVertexInputState = &vertex_input_state;

  /* Input assembly state. */
  VkPipelineInputAssemblyStateCreateInfo pipeline_input_assembly = {};
  pipeline_input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  pipeline_input_assembly.topology = to_vk_primitive_topology(prim_type);
  pipeline_input_assembly.primitiveRestartEnable =
      ELEM(prim_type, GPU_PRIM_TRIS, GPU_PRIM_LINES, GPU_PRIM_POINTS, GPU_PRIM_LINES_ADJ) ?
          VK_FALSE :
          VK_TRUE;
  pipeline_create_info.pInputAssemblyState = &pipeline_input_assembly;

  /* Multi-sample state. */
  VkPipelineMultisampleStateCreateInfo multisample_state = {};
  multisample_state.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisample_state.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
  multisample_state.minSampleShading = 1.0f;
  pipeline_create_info.pMultisampleState = &multisample_state;

  /* States from the state manager. */
  state_manager.finalize_color_blend_state(framebuffer);
  pipeline_create_info.pColorBlendState = &state_manager.pipeline_color_blend_state;
  pipeline_create_info.pRasterizationState = &state_manager.rasterization_state;
  pipeline_create_info.pDepthStencilState = &state_manager.depth_stencil_state;
  /* Viewport state. */
  VkPipelineViewportStateCreateInfo viewport_state = {
      VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
  pipeline_create_info.pViewportState = &viewport_state;
  pipeline_create_info.pDynamicState = &state_manager.dynamic_state;
  viewport_state.pViewports = VK_NULL_HANDLE;
  viewport_state.viewportCount = (framebuffer.is_multi_viewport()) ? GPU_MAX_VIEWPORTS : 1;
  viewport_state.pScissors = VK_NULL_HANDLE;
  viewport_state.scissorCount = 1;
  const VKDevice &device = VKBackend::get().device_get();
  vkCreateGraphicsPipelines(device.device_get(),
                            device.vk_pipeline_cache_get(),
                            1,
                            &pipeline_create_info,
                            vk_allocation_callbacks,
                            &active_vk_pipeline_);

#ifdef VK_PIPELINE_REFACTOR
  if (type > 0) {
    switch (type) {
      case 1:
        renderpass.set_pipeline(active_vk_pipeline_, prim_type, state_manager.get_state(), 1);
        break;
      case 2:
        renderpass.set_pipeline(active_vk_pipeline_,
                                prim_type,
                                vertex_module,
                                geometry_module,
                                fragment_module,
                                state_manager.get_state(),
                                2);
        break;
      case 3:
        renderpass.set_pipeline(active_vk_pipeline_,
                                prim_type,
                                vertex_module,
                                geometry_module,
                                fragment_module,
                                state_manager.get_state(),
                                vertex_attribute_object.info,
                                3);
        break;
    }
  }
  /*
  else {
    vk_pipelines_.append(active_vk_pipeline_);
  }
  */
#else
  /* TODO: we should cache several pipeline instances and detect pipelines we can reuse. This might
   * also be done using a VkPipelineCache. For now we just destroy any available pipeline so it
   * won't be overwritten by the newly created one. */
  vk_pipelines_.append(active_vk_pipeline_);
#endif

  debug::object_label(active_vk_pipeline_, "GraphicsPipeline");
}

void VKPipeline::bind(VKContext &context, VkPipelineBindPoint vk_pipeline_bind_point)
{
  VKCommandBuffers &command_buffers = context.command_buffers_get();
  command_buffers.bind(*this, vk_pipeline_bind_point);
  if (VK_PIPELINE_BIND_POINT_COMPUTE != vk_pipeline_bind_point) {
    context.active_framebuffer_get()->dynamic_state_set();
  }
}

void VKPipeline::update_push_constants(VKContext &context)
{
  push_constants_.update(context);
}

}  // namespace blender::gpu
