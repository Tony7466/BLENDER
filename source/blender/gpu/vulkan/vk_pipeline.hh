/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include <optional>

#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_descriptor_set.hh"
#include "vk_pipeline_state.hh"
#include "vk_push_constants.hh"

#ifdef VK_STAT_PIPELINE_CACHE
#  include "BLI_set.hh"
#  include <iomanip>
#  include <iostream>
#  include <sstream>
#  include <unordered_map>
#endif

namespace blender::gpu {
class VKContext;
class VKShader;
class VKVertexAttributeObject;
class VKBatch;
#ifdef VK_STAT_PIPELINE_CACHE
#  define VK_CACHE_VERTEX_INPUT_MAX 16
struct MutableState {
  float depth_range0;
  float depth_range1;
  float point_size;
  float line_width;
  uint8_t stencil_write_mask;
  uint8_t stencil_compare_mask;
  uint8_t stencil_reference;
};
class StateCache {
  Set<const uint64_t> states;
  Set<const std::string> mutable_states;

  struct VertexInputNumsCache {
    uint32_t vertexBindingDescriptionCount;
    uint32_t vertexAttributeDescriptionCount;
    uint32_t vertexBindingDivisorCount;
  };
#endif
  Set<const std::string> vertex_inputs;
  Set<VkShaderModule> vertex;
  Set<VkShaderModule> fragment;
  Set<VkShaderModule> geometry;
  Set<VkPipelineLayout> pipeline_layouts;
  Set<GPUPrimType> prim_types;
  Set<uint32_t> blend;
  Set<uint32_t> line_smooth;
  Set<uint32_t> write_mask;
  Set<uint32_t> point_size;
  Set<uint32_t> line_width;
  Set<uint32_t> stencil_write_mask;
  std::string name_ = "";

 public:
  void set_name(std::string sh_name, std::string fb_name);
  void append(VkPipelineLayout layout,
              GPUPrimType type,
              VkShaderModule vmodule,
              VkShaderModule gmodule,
              VkShaderModule fmodule,
              const GPUState &state,
              const GPUStateMutable &mutable_state,
              const VkPipelineVertexInputStateCreateInfo &info);
};
class StateCacheMap {
 public:
  std::unordered_map<std::string, StateCache> state_cache_map;
  StateCache &get(std::string fb_name);
};

/**
 * Pipeline can be a compute pipeline or a graphic pipeline.
 *
 * Compute pipelines can be constructed early on, but graphics
 * pipelines depends on the actual GPU state/context.
 *
 * - TODO: we should sanitize the interface. There we can also
 *   use late construction for compute pipelines.
 */
class VKPipeline : NonCopyable {
  /* Active pipeline handle. */
  VkPipeline active_vk_pipeline_ = VK_NULL_HANDLE;
  /** Keep track of all pipelines as they can still be in flight. */
  Vector<VkPipeline> vk_pipelines_;
  VKPushConstants push_constants_;
  VKPipelineStateManager state_manager_;
#ifdef VK_STAT_PIPELINE_CACHE
  StateCacheMap state_cache_map;
#endif
 public:
  VKPipeline() = default;

  virtual ~VKPipeline();
  VKPipeline(VKPushConstants &&push_constants);
#ifndef VK_PIPELINE_REFACTOR
  VKPipeline(VkPipeline vk_pipeline, VKPushConstants &&push_constants);
#else
  VKPipeline(VkPipeline vk_pipeline, VKPushConstants &&push_constants, bool is_compute = false);
#endif
  void destroy();
  VKPipeline &operator=(VKPipeline &&other)
  {
    active_vk_pipeline_ = other.active_vk_pipeline_;
    other.active_vk_pipeline_ = VK_NULL_HANDLE;
    push_constants_ = std::move(other.push_constants_);
    vk_pipelines_ = std::move(other.vk_pipelines_);
    other.vk_pipelines_.clear();
    return *this;
  }

  static VKPipeline create_compute_pipeline(VkShaderModule compute_module,
                                            VkPipelineLayout &pipeline_layouts,
                                            const VKPushConstants::Layout &push_constants_layout,
                                            const VkSpecializationInfo *specialzation);
  static VKPipeline create_graphics_pipeline(const VKPushConstants::Layout &push_constants_layout);

  VKPushConstants &push_constants_get()
  {
    return push_constants_;
  }

  VKPipelineStateManager &state_manager_get()
  {
    return state_manager_;
  }

  VkPipeline vk_handle() const;
  bool is_valid() const;

  void finalize(VKContext &context,
                VkShaderModule vertex_module,
                VkShaderModule geometry_module,
                VkShaderModule fragment_module,
                VkPipelineLayout &pipeline_layout,
                const GPUPrimType prim_type,
                const VKVertexAttributeObject &vertex_attribute_object);

  void bind(VKContext &context, VkPipelineBindPoint vk_pipeline_bind_point);
  void update_push_constants(VKContext &context);
};

}  // namespace blender::gpu
