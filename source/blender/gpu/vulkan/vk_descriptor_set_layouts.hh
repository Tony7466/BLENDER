/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 *
 * Multiple shaders can use the same descriptor set layout. VKDescriptorSetLayouts can create and
 * will own all descriptor set layouts.
 */

#pragma once

#include "BLI_map.hh"
#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#include "vk_common.hh"

namespace blender::gpu {
struct VKDescriptorSetLayoutInfo {
  using Bindings = Vector<VkDescriptorType>;

  Bindings bindings;
  VkShaderStageFlags vk_shader_stage_flags;

  bool operator==(const VKDescriptorSetLayoutInfo &other) const
  {
    return vk_shader_stage_flags == other.vk_shader_stage_flags && bindings == other.bindings;
  };
};

};  // namespace blender::gpu

namespace blender {
template<> struct DefaultHash<gpu::VKDescriptorSetLayoutInfo> {
  uint64_t operator()(const gpu::VKDescriptorSetLayoutInfo &key) const
  {
    uint64_t hash = uint64_t(key.vk_shader_stage_flags);
    for (VkDescriptorType vk_descriptor_type : key.bindings) {
      hash = hash * 33 ^ uint64_t(vk_descriptor_type);
    }
    return hash;
  }
};
}  // namespace blender

namespace blender::gpu {

class VKDescriptorSetLayouts : NonCopyable {

 private:
  Map<VKDescriptorSetLayoutInfo, VkDescriptorSetLayout> vk_descriptor_set_layouts_;

  VkDescriptorSetLayoutCreateInfo vk_descriptor_set_layout_create_info_;
  Vector<VkDescriptorSetLayoutBinding> vk_descriptor_set_layout_bindings_;
  std::mutex mutex_;

 public:
  VKDescriptorSetLayouts();
  virtual ~VKDescriptorSetLayouts();

  VkDescriptorSetLayout get_or_create(const VKDescriptorSetLayoutInfo &info,
                                      bool &r_created,
                                      bool &r_needed);

  void deinit();

  int64_t size() const
  {
    return vk_descriptor_set_layouts_.size();
  }

 private:
  void update_layout_bindings(const VKDescriptorSetLayoutInfo &info);
};

}  // namespace blender::gpu
