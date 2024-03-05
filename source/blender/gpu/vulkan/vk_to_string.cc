
/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_to_string.hh"

#include <sstream>

namespace blender::gpu {
const char *to_string(const VkFilter vk_filter)
{
  switch (vk_filter) {
    case VK_FILTER_NEAREST:
      return STRINGIFY(VK_FILTER_NEAREST);

    case VK_FILTER_LINEAR:
      return STRINGIFY(VK_FILTER_LINEAR);

    default:
      break;
  }
  return STRINGIFY_ARG(vk_filter);
}

const char *to_string(const VkImageLayout vk_image_layout)
{
  switch (vk_image_layout) {
    case VK_IMAGE_LAYOUT_UNDEFINED:
      return STRINGIFY(VK_IMAGE_LAYOUT_UNDEFINED);

    case VK_IMAGE_LAYOUT_GENERAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_GENERAL);

    case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);

    case VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL);

    case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);

    case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

    case VK_IMAGE_LAYOUT_PREINITIALIZED:
      return STRINGIFY(VK_IMAGE_LAYOUT_PREINITIALIZED);

    /* Extensions for VK_VERSION_1_1. */
    case VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_STENCIL_ATTACHMENT_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_STENCIL_ATTACHMENT_OPTIMAL);

    case VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_STENCIL_READ_ONLY_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_STENCIL_READ_ONLY_OPTIMAL);

    /* Extensions for VK_VERSION_1_2. */
    case VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL);

    case VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL);

    case VK_IMAGE_LAYOUT_STENCIL_ATTACHMENT_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_STENCIL_ATTACHMENT_OPTIMAL);

    case VK_IMAGE_LAYOUT_STENCIL_READ_ONLY_OPTIMAL:
      return STRINGIFY(VK_IMAGE_LAYOUT_STENCIL_READ_ONLY_OPTIMAL);

    /* Extensions for VK_KHR_swapchain. */
    case VK_IMAGE_LAYOUT_PRESENT_SRC_KHR:
      return STRINGIFY(VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);

    default:
      break;
  }
  return STRINGIFY_ARG(vk_image_layout);
}

const char *to_string(const VkIndexType vk_index_type)
{
  switch (vk_index_type) {
    case VK_INDEX_TYPE_UINT16:
      return STRINGIFY(VK_INDEX_TYPE_UINT16);

    case VK_INDEX_TYPE_UINT32:
      return STRINGIFY(VK_INDEX_TYPE_UINT32);

    default:
      break;
  }
  return STRINGIFY_ARG(vk_index_type);
}

const char *to_string(const VkObjectType vk_object_type)
{
  switch (vk_object_type) {
    case VK_OBJECT_TYPE_UNKNOWN:
      return STRINGIFY(VK_OBJECT_TYPE_UNKNOWN);

    case VK_OBJECT_TYPE_INSTANCE:
      return STRINGIFY(VK_OBJECT_TYPE_INSTANCE);

    case VK_OBJECT_TYPE_PHYSICAL_DEVICE:
      return STRINGIFY(VK_OBJECT_TYPE_PHYSICAL_DEVICE);

    case VK_OBJECT_TYPE_DEVICE:
      return STRINGIFY(VK_OBJECT_TYPE_DEVICE);

    case VK_OBJECT_TYPE_QUEUE:
      return STRINGIFY(VK_OBJECT_TYPE_QUEUE);

    case VK_OBJECT_TYPE_SEMAPHORE:
      return STRINGIFY(VK_OBJECT_TYPE_SEMAPHORE);

    case VK_OBJECT_TYPE_COMMAND_BUFFER:
      return STRINGIFY(VK_OBJECT_TYPE_COMMAND_BUFFER);

    case VK_OBJECT_TYPE_FENCE:
      return STRINGIFY(VK_OBJECT_TYPE_FENCE);

    case VK_OBJECT_TYPE_DEVICE_MEMORY:
      return STRINGIFY(VK_OBJECT_TYPE_DEVICE_MEMORY);

    case VK_OBJECT_TYPE_BUFFER:
      return STRINGIFY(VK_OBJECT_TYPE_BUFFER);

    case VK_OBJECT_TYPE_IMAGE:
      return STRINGIFY(VK_OBJECT_TYPE_IMAGE);

    case VK_OBJECT_TYPE_EVENT:
      return STRINGIFY(VK_OBJECT_TYPE_EVENT);

    case VK_OBJECT_TYPE_QUERY_POOL:
      return STRINGIFY(VK_OBJECT_TYPE_QUERY_POOL);

    case VK_OBJECT_TYPE_BUFFER_VIEW:
      return STRINGIFY(VK_OBJECT_TYPE_BUFFER_VIEW);

    case VK_OBJECT_TYPE_IMAGE_VIEW:
      return STRINGIFY(VK_OBJECT_TYPE_IMAGE_VIEW);

    case VK_OBJECT_TYPE_SHADER_MODULE:
      return STRINGIFY(VK_OBJECT_TYPE_SHADER_MODULE);

    case VK_OBJECT_TYPE_PIPELINE_CACHE:
      return STRINGIFY(VK_OBJECT_TYPE_PIPELINE_CACHE);

    case VK_OBJECT_TYPE_PIPELINE_LAYOUT:
      return STRINGIFY(VK_OBJECT_TYPE_PIPELINE_LAYOUT);

    case VK_OBJECT_TYPE_RENDER_PASS:
      return STRINGIFY(VK_OBJECT_TYPE_RENDER_PASS);

    case VK_OBJECT_TYPE_PIPELINE:
      return STRINGIFY(VK_OBJECT_TYPE_PIPELINE);

    case VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT:
      return STRINGIFY(VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT);

    case VK_OBJECT_TYPE_SAMPLER:
      return STRINGIFY(VK_OBJECT_TYPE_SAMPLER);

    case VK_OBJECT_TYPE_DESCRIPTOR_POOL:
      return STRINGIFY(VK_OBJECT_TYPE_DESCRIPTOR_POOL);

    case VK_OBJECT_TYPE_DESCRIPTOR_SET:
      return STRINGIFY(VK_OBJECT_TYPE_DESCRIPTOR_SET);

    case VK_OBJECT_TYPE_FRAMEBUFFER:
      return STRINGIFY(VK_OBJECT_TYPE_FRAMEBUFFER);

    case VK_OBJECT_TYPE_COMMAND_POOL:
      return STRINGIFY(VK_OBJECT_TYPE_COMMAND_POOL);

    /* Extensions for VK_VERSION_1_1. */
    case VK_OBJECT_TYPE_SAMPLER_YCBCR_CONVERSION:
      return STRINGIFY(VK_OBJECT_TYPE_SAMPLER_YCBCR_CONVERSION);

    case VK_OBJECT_TYPE_DESCRIPTOR_UPDATE_TEMPLATE:
      return STRINGIFY(VK_OBJECT_TYPE_DESCRIPTOR_UPDATE_TEMPLATE);

    /* Extensions for VK_KHR_swapchain. */
    case VK_OBJECT_TYPE_SWAPCHAIN_KHR:
      return STRINGIFY(VK_OBJECT_TYPE_SWAPCHAIN_KHR);

    default:
      break;
  }
  return STRINGIFY_ARG(vk_object_type);
}

const char *to_string(const VkPipelineBindPoint vk_pipeline_bind_point)
{
  switch (vk_pipeline_bind_point) {
    case VK_PIPELINE_BIND_POINT_GRAPHICS:
      return STRINGIFY(VK_PIPELINE_BIND_POINT_GRAPHICS);

    case VK_PIPELINE_BIND_POINT_COMPUTE:
      return STRINGIFY(VK_PIPELINE_BIND_POINT_COMPUTE);

    default:
      break;
  }
  return STRINGIFY_ARG(vk_pipeline_bind_point);
}

const char *to_string(const VkSubpassContents vk_subpass_contents)
{
  switch (vk_subpass_contents) {
    case VK_SUBPASS_CONTENTS_INLINE:
      return STRINGIFY(VK_SUBPASS_CONTENTS_INLINE);

    case VK_SUBPASS_CONTENTS_SECONDARY_COMMAND_BUFFERS:
      return STRINGIFY(VK_SUBPASS_CONTENTS_SECONDARY_COMMAND_BUFFERS);

    default:
      break;
  }
  return STRINGIFY_ARG(vk_subpass_contents);
}

std::string to_string_vk_dependency_flags(const VkDependencyFlags vk_dependency_flags)
{
  std::stringstream ss;

  if (vk_dependency_flags & VK_DEPENDENCY_BY_REGION_BIT) {
    ss << STRINGIFY(VK_DEPENDENCY_BY_REGION_BIT) << ", ";
  }
  /* Extensions for VK_VERSION_1_1. */
  if (vk_dependency_flags & VK_DEPENDENCY_DEVICE_GROUP_BIT) {
    ss << STRINGIFY(VK_DEPENDENCY_DEVICE_GROUP_BIT) << ", ";
  }
  if (vk_dependency_flags & VK_DEPENDENCY_VIEW_LOCAL_BIT) {
    ss << STRINGIFY(VK_DEPENDENCY_VIEW_LOCAL_BIT) << ", ";
  }

  std::string result = ss.str();
  if (result.size() >= 2) {
    result.erase(result.size() - 2, 2);
  }
  return result;
}

std::string to_string_vk_pipeline_stage_flags(const VkPipelineStageFlags vk_pipeline_stage_flags)
{
  std::stringstream ss;

  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_VERTEX_INPUT_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_VERTEX_INPUT_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_VERTEX_SHADER_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_VERTEX_SHADER_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_TESSELLATION_CONTROL_SHADER_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_TESSELLATION_CONTROL_SHADER_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_TESSELLATION_EVALUATION_SHADER_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_TESSELLATION_EVALUATION_SHADER_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_GEOMETRY_SHADER_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_GEOMETRY_SHADER_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_TRANSFER_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_TRANSFER_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_HOST_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_HOST_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT) << ", ";
  }
  if (vk_pipeline_stage_flags & VK_PIPELINE_STAGE_ALL_COMMANDS_BIT) {
    ss << STRINGIFY(VK_PIPELINE_STAGE_ALL_COMMANDS_BIT) << ", ";
  }

  std::string result = ss.str();
  if (result.size() >= 2) {
    result.erase(result.size() - 2, 2);
  }
  return result;
}

std::string to_string_vk_shader_stage_flags(const VkShaderStageFlags vk_shader_stage_flags)
{
  std::stringstream ss;

  if (vk_shader_stage_flags & VK_SHADER_STAGE_VERTEX_BIT) {
    ss << STRINGIFY(VK_SHADER_STAGE_VERTEX_BIT) << ", ";
  }
  if (vk_shader_stage_flags & VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT) {
    ss << STRINGIFY(VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT) << ", ";
  }
  if (vk_shader_stage_flags & VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT) {
    ss << STRINGIFY(VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT) << ", ";
  }
  if (vk_shader_stage_flags & VK_SHADER_STAGE_GEOMETRY_BIT) {
    ss << STRINGIFY(VK_SHADER_STAGE_GEOMETRY_BIT) << ", ";
  }
  if (vk_shader_stage_flags & VK_SHADER_STAGE_FRAGMENT_BIT) {
    ss << STRINGIFY(VK_SHADER_STAGE_FRAGMENT_BIT) << ", ";
  }
  if (vk_shader_stage_flags & VK_SHADER_STAGE_COMPUTE_BIT) {
    ss << STRINGIFY(VK_SHADER_STAGE_COMPUTE_BIT) << ", ";
  }
  if (vk_shader_stage_flags & VK_SHADER_STAGE_ALL_GRAPHICS) {
    ss << STRINGIFY(VK_SHADER_STAGE_ALL_GRAPHICS) << ", ";
  }
  if (vk_shader_stage_flags & VK_SHADER_STAGE_ALL) {
    ss << STRINGIFY(VK_SHADER_STAGE_ALL) << ", ";
  }

  std::string result = ss.str();
  if (result.size() >= 2) {
    result.erase(result.size() - 2, 2);
  }
  return result;
}
}  // namespace blender::gpu
