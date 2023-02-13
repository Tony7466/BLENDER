/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_descriptor_set.hh"
#include "vk_index_buffer.hh"
#include "vk_storage_buffer.hh"
#include "vk_texture.hh"
#include "vk_vertex_buffer.hh"

#include "BLI_assert.h"

namespace blender::gpu {
VKDescriptorSet::~VKDescriptorSet()
{
  if (vk_descriptor_set_ != VK_NULL_HANDLE) {
    /* Handle should be given back to the pool.*/
    VKContext &context = *VKContext::get();
    context.descriptor_pools_get().free(*this);
    BLI_assert(vk_descriptor_set_ == VK_NULL_HANDLE);
  }
}

void VKDescriptorSet::mark_freed()
{
  vk_descriptor_set_ = VK_NULL_HANDLE;
  vk_descriptor_pool_ = VK_NULL_HANDLE;
}

void VKDescriptorSet::bind(VKStorageBuffer &buffer, int location)
{
  Binding &binding = ensure_location(location);
  binding.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  binding.vk_buffer = buffer.vk_handle();
  binding.buffer_size = buffer.size_in_bytes();
}

void VKDescriptorSet::bind_as_ssbo(VKVertexBuffer &buffer, int location)
{
  Binding &binding = ensure_location(location);
  binding.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  binding.vk_buffer = buffer.vk_handle();
  binding.buffer_size = buffer.size_used_get();
}

void VKDescriptorSet::bind_as_ssbo(VKIndexBuffer &buffer, int location)
{
  Binding &binding = ensure_location(location);
  binding.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  binding.vk_buffer = buffer.vk_handle();
  binding.buffer_size = buffer.size_get();
}

void VKDescriptorSet::image_bind(VKTexture &texture, int location)
{
}

VKDescriptorSet::Binding &VKDescriptorSet::ensure_location(int location)
{
  for (Binding &binding : bindings_) {
    if (binding.location == location) {
      return binding;
    }
  }

  Binding binding;
  binding.vk_buffer = VK_NULL_HANDLE;
  binding.buffer_size = 0;
  binding.location = location;
  bindings_.append(binding);
  return bindings_.last();
}

void VKDescriptorSet::update(VkDevice vk_device)
{
  Vector<VkDescriptorBufferInfo> buffer_infos;
  Vector<VkWriteDescriptorSet> descriptor_writes;
  for (const Binding &binding : bindings_) {
    VkDescriptorBufferInfo buffer_info = {};
    buffer_info.buffer = binding.vk_buffer;
    buffer_info.range = binding.buffer_size;
    buffer_infos.append(buffer_info);

    VkWriteDescriptorSet write_descriptor = {};
    write_descriptor.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write_descriptor.dstSet = vk_descriptor_set_;
    write_descriptor.dstBinding = binding.location;
    write_descriptor.descriptorCount = 1;
    write_descriptor.descriptorType = binding.type;
    write_descriptor.pBufferInfo = &buffer_infos.last();

    descriptor_writes.append(write_descriptor);
  }

  vkUpdateDescriptorSets(
      vk_device, descriptor_writes.size(), descriptor_writes.data(), 0, nullptr);
}

}  // namespace blender::gpu
