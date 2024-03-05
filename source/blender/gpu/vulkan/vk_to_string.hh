/* SPDX-FileCopyrightText: 2024 Blender Authors All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */
#pragma once

#include "vk_common.hh"

namespace blender::gpu {

const char *to_string(VkFilter vk_filter);
const char *to_string(VkImageLayout vk_image_layout);
const char *to_string(VkIndexType vk_index_type);
const char *to_string(VkObjectType vk_object_type);
const char *to_string(VkPipelineBindPoint vk_pipeline_bind_point);
const char *to_string(VkSubpassContents vk_subpass_contents);
std::string to_string_vk_dependency_flags(VkDependencyFlags vk_dependency_flags);
std::string to_string_vk_pipeline_stage_flags(VkPipelineStageFlags vk_pipeline_stage_flags);
std::string to_string_vk_shader_stage_flags(VkShaderStageFlags vk_shader_stage_flags);

}  // namespace blender::gpu
