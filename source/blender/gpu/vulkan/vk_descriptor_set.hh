/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#ifdef __APPLE__
#  include <MoltenVK/vk_mvk_moltenvk.h>
#else
#  include <vulkan/vulkan.h>
#endif

namespace blender::gpu {
class VKStorageBuffer;
class VKVertexBuffer;
class VKIndexBuffer;
class VKTexture;

class VKDescriptorSet : NonCopyable {
  struct Binding {
    int location;
    VkDescriptorType type;
    VkBuffer vk_buffer = VK_NULL_HANDLE;
    VkDeviceSize buffer_size;
  };

  VkDescriptorPool vk_descriptor_pool_ = VK_NULL_HANDLE;
  VkDescriptorSet vk_descriptor_set_ = VK_NULL_HANDLE;

  Vector<Binding> bindings_;

 public:
  VKDescriptorSet() = default;
  VKDescriptorSet(VkDescriptorPool vk_descriptor_pool, VkDescriptorSet vk_descriptor_set)
      : vk_descriptor_pool_(vk_descriptor_pool), vk_descriptor_set_(vk_descriptor_set)
  {
  }
  virtual ~VKDescriptorSet();

  VKDescriptorSet &operator=(VKDescriptorSet &&other)
  {
    vk_descriptor_set_ = other.vk_descriptor_set_;
    vk_descriptor_pool_ = other.vk_descriptor_pool_;
    other.mark_freed();
    return *this;
  }

  VkDescriptorSet vk_handle() const
  {
    return vk_descriptor_set_;
  }

  VkDescriptorPool vk_pool_handle() const
  {
    return vk_descriptor_pool_;
  }

  void bind_as_ssbo(VKVertexBuffer &buffer, int location);
  void bind_as_ssbo(VKIndexBuffer &buffer, int location);
  void bind(VKStorageBuffer &buffer, int location);
  void image_bind(VKTexture &texture, int location);

  /**
   * Update the descriptor set on the device.
   */
  void update(VkDevice vk_device);

  void mark_freed();

 private:
  Binding &ensure_location(int location);
};

}  // namespace blender::gpu
