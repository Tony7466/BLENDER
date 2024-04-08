/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

namespace blender::gpu::render_graph {
/**
 * Container for storing shader descriptor set and push constants.
 *
 * Compute and graphic shaders use the same structure to setup the pipeline for execution.
 */
struct VKPipelineData {
  VkPipeline vk_pipeline;
  VkPipelineLayout vk_pipeline_layout;
  VkDescriptorSet vk_descriptor_set;
  uint32_t push_constants_size;
  const void *push_constants_data;
};

struct VKBoundPipeline {
  VkPipeline vk_pipeline;
  VkDescriptorSet vk_descriptor_set;
};
struct VKBoundPipelines {
  VKBoundPipeline compute;
  VKBoundPipeline graphics;
};

// TODO: This is a copy constructor in disguise.
BLI_INLINE void localize_shader_data(VKPipelineData &dst, const VKPipelineData &src)
{
  dst.push_constants_data = nullptr;
  dst.push_constants_size = src.push_constants_size;
  if (src.push_constants_size) {
    BLI_assert(src.push_constants_data);
    void *data = MEM_mallocN(src.push_constants_size, __func__);
    memcpy(data, src.push_constants_data, src.push_constants_size);
    dst.push_constants_data = data;
  }
}

BLI_INLINE void vk_pipeline_free_data(VKPipelineData &data)
{
  MEM_SAFE_FREE(data.push_constants_data);
}

}  // namespace blender::gpu::render_graph
