/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

#include "vk_descriptor_pools.hh"

namespace blender::gpu {
class VKResourcePool {
 private:
  Vector<std::pair<VkImage, VmaAllocation>> discarded_images_;
  Vector<std::pair<VkBuffer, VmaAllocation>> discarded_buffers_;
  Vector<VkImageView> discarded_image_views_;
  Vector<VkShaderModule> discarded_shader_modules_;
  Vector<VkPipelineLayout> discarded_pipeline_layouts_;

 public:
  VKDescriptorPools descriptor_pools;
  VKDescriptorSetTracker descriptor_set;

  void init(VKDevice &device);
  void deinit(VKDevice &device);

  void discard_image(VkImage vk_image, VmaAllocation vma_allocation);
  void discard_image_view(VkImageView vk_image_view);
  void discard_buffer(VkBuffer vk_buffer, VmaAllocation vma_allocation);
  void discard_shader_module(VkShaderModule vk_shader_module);
  void discard_pipeline_layout(VkPipelineLayout vk_pipeline_layout);
  void destroy_discarded_resources(VKDevice &device);
};
}  // namespace blender::gpu
