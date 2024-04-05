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

}  // namespace blender::gpu::render_graph
