/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation */

/** \file
 * \ingroup gpu
 */

#pragma once

#ifdef __APPLE__
#  include <MoltenVK/vk_mvk_moltenvk.h>
#else
#  include <vulkan/vulkan.h>
#endif

#include "vk_mem_alloc.h"

#include "gpu_index_buffer_private.hh"
#include "gpu_texture_private.hh"

namespace blender::gpu {

VkImageAspectFlagBits to_vk_image_aspect_flag_bits(const eGPUTextureFormat format);
VkFormat to_vk_format(const eGPUTextureFormat format);
VkFormat to_vk_format(const GPUVertCompType type, const uint32_t size);
VkComponentMapping to_vk_component_mapping(const eGPUTextureFormat format);
VkImageViewType to_vk_image_view_type(const eGPUTextureType type);
VkImageType to_vk_image_type(const eGPUTextureType type);
VkClearColorValue to_vk_clear_color_value(const eGPUDataFormat format, const void *data);
VkIndexType to_vk_index_type(const GPUIndexBufType index_type);
VkPrimitiveTopology to_vk_primitive_topology(const GPUPrimType prim_type);
VkCullModeFlags to_vk_cull_mode_flags(const eGPUFaceCullTest cull_test);

}  // namespace blender::gpu
