/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_descriptor_set.hh"
#include "vk_index_buffer.hh"
#include "vk_sampler.hh"
#include "vk_shader.hh"
#include "vk_shader_interface.hh"
#include "vk_state_manager.hh"
#include "vk_storage_buffer.hh"
#include "vk_texture.hh"
#include "vk_uniform_buffer.hh"
#include "vk_vertex_buffer.hh"

#include "BLI_assert.h"

namespace blender::gpu {

void VKDescriptorSetTracker::reset()
{
  vk_descriptor_image_infos_.clear();
  vk_descriptor_buffer_infos_.clear();
  vk_buffer_views_.clear();
  vk_write_descriptor_sets_.clear();
  vk_descriptor_set = VK_NULL_HANDLE;
}

void VKDescriptorSetTracker::bind_buffer(VkDescriptorType vk_descriptor_type,
                                         VkBuffer vk_buffer,
                                         VkDeviceSize size_in_bytes,
                                         VKDescriptorSet::Location location)
{
  vk_descriptor_buffer_infos_.append({vk_buffer, 0, size_in_bytes});
  vk_write_descriptor_sets_.append({VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                                    nullptr,
                                    VK_NULL_HANDLE,
                                    location,
                                    0,
                                    1,
                                    vk_descriptor_type,
                                    nullptr,
                                    nullptr,
                                    nullptr});
}

void VKDescriptorSetTracker::bind_texel_buffer(VKVertexBuffer &vertex_buffer,
                                               const VKDescriptorSet::Location location)
{
  vk_buffer_views_.append(vertex_buffer.vk_buffer_view_get());
  vk_write_descriptor_sets_.append({VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                                    nullptr,
                                    VK_NULL_HANDLE,
                                    location,
                                    0,
                                    1,
                                    VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER,
                                    nullptr,
                                    nullptr,
                                    nullptr});
}

void VKDescriptorSetTracker::bind_image(VkDescriptorType vk_descriptor_type,
                                        VkSampler vk_sampler,
                                        VkImageView vk_image_view,
                                        VkImageLayout vk_image_layout,
                                        VKDescriptorSet::Location location)
{
  vk_descriptor_image_infos_.append({vk_sampler, vk_image_view, vk_image_layout});
  vk_write_descriptor_sets_.append({VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                                    nullptr,
                                    VK_NULL_HANDLE,
                                    location,
                                    0,
                                    1,
                                    vk_descriptor_type,
                                    nullptr,
                                    nullptr,
                                    nullptr});
}

void VKDescriptorSetTracker::update(VKContext &context,
                                    render_graph::VKResourceAccessInfo &access_info)
{
  // reset();
  VKShader &shader = *unwrap(context.shader);
  const VKShaderInterface &shader_interface = shader.interface_get();
  VKStateManager &state_manager = context.state_manager_get();
  for (const VKResourceBinding &resource_binding : shader_interface.resource_bindings_get()) {
    if (resource_binding.location.binding == -1) {
      continue;
    }

    switch (resource_binding.bind_type) {
      case shader::ShaderCreateInfo::Resource::BindType::IMAGE:
        break;
      case shader::ShaderCreateInfo::Resource::BindType::SAMPLER:
        break;
      case shader::ShaderCreateInfo::Resource::BindType::STORAGE_BUFFER:
        break;
      case shader::ShaderCreateInfo::Resource::BindType::UNIFORM_BUFFER: {
        VKUniformBuffer &uniform_buffer = *state_manager.uniform_buffers_.get(
            resource_binding.binding);
        uniform_buffer.ensure_updated();
        bind_buffer(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                    uniform_buffer.vk_handle(),
                    uniform_buffer.size_in_bytes(),
                    resource_binding.location);
        access_info.buffers.append({uniform_buffer.vk_handle(), resource_binding.access_mask});
        break;
      }
    }
  }

  /* Add uniform push constants */
  VKPushConstants &push_constants = shader.push_constants;
  if (push_constants.layout_get().storage_type_get() ==
      VKPushConstants::StorageType::UNIFORM_BUFFER)
  {
    push_constants.update_uniform_buffer();
    const VKUniformBuffer &uniform_buffer = *push_constants.uniform_buffer_get().get();
    bind_buffer(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                uniform_buffer.vk_handle(),
                uniform_buffer.size_in_bytes(),
                push_constants.layout_get().descriptor_set_location_get());
    access_info.buffers.append({uniform_buffer.vk_handle(), VK_ACCESS_UNIFORM_READ_BIT});
  }

  // go over each resource binding
  // get resource from context.statemanager
  // get descriptor set location
  // determine image, texel buffer or buffer
  // bind

  // if shader_interface has uniform push constants...

  VkDescriptorSetLayout vk_descriptor_set_layout = shader.vk_descriptor_set_layout_get();
  vk_descriptor_set = context.descriptor_pools_get().allocate(vk_descriptor_set_layout);
  BLI_assert(vk_descriptor_set != VK_NULL_HANDLE);
  debug::object_label(vk_descriptor_set, shader.name_get());

  /* Populate the final addresses and handles */
  int buffer_index = 0;
  int buffer_view_index = 0;
  int image_index = 0;
  for (int write_index : vk_write_descriptor_sets_.index_range()) {
    VkWriteDescriptorSet &vk_write_descriptor_set = vk_write_descriptor_sets_[write_index++];
    vk_write_descriptor_set.dstSet = vk_descriptor_set;

    switch (vk_write_descriptor_set.descriptorType) {
      case VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER:
      case VK_DESCRIPTOR_TYPE_STORAGE_IMAGE:
        vk_write_descriptor_set.pImageInfo = &vk_descriptor_image_infos_[image_index++];
        break;

      case VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER:
        vk_write_descriptor_set.pTexelBufferView = &vk_buffer_views_[buffer_view_index++];
        break;

      case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER:
      case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER:
        vk_write_descriptor_set.pBufferInfo = &vk_descriptor_buffer_infos_[buffer_index++];
        break;

      default:
        BLI_assert_unreachable();
        break;
    }
  }

  /* Update the descriptor set on the device. */
  const VKDevice &device = VKBackend::get().device;
  vkUpdateDescriptorSets(device.vk_handle(),
                         vk_write_descriptor_sets_.size(),
                         vk_write_descriptor_sets_.data(),
                         0,
                         nullptr);
}

}  // namespace blender::gpu
