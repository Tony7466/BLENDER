/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_shader_interface.hh"
#include "vk_context.hh"

namespace blender::gpu {

void VKShaderInterface::init(const shader::ShaderCreateInfo &info)
{
  using namespace blender::gpu::shader;

  attr_len_ = 0;
  uniform_len_ = info.push_constants_.size();
  ssbo_len_ = 0;
  ubo_len_ = 0;
  image_offset_ = -1;

  Vector<ShaderCreateInfo::Resource> all_resources;
  all_resources.extend(info.pass_resources_);
  all_resources.extend(info.batch_resources_);

  for (ShaderCreateInfo::Resource &res : all_resources) {
    switch (res.bind_type) {
      case ShaderCreateInfo::Resource::BindType::IMAGE:
        uniform_len_++;
        break;
      case ShaderCreateInfo::Resource::BindType::SAMPLER:
        image_offset_ = max_ii(image_offset_, res.slot);
        uniform_len_++;
        break;
      case ShaderCreateInfo::Resource::BindType::UNIFORM_BUFFER:
        ubo_len_++;
        break;
      case ShaderCreateInfo::Resource::BindType::STORAGE_BUFFER:
        ssbo_len_++;
        break;
    }
  }

  /* Reserve 1 storage buffer for push constants fallback. */
  size_t names_size = info.interface_names_size_;
  VKContext &context = *VKContext::get();
  const VKPushConstantsLayout::StorageType push_constants_storage_type =
      VKPushConstantsLayout::determine_storage_type(info, context.physical_device_limits_get());
  if (push_constants_storage_type == VKPushConstantsLayout::StorageType::STORAGE_BUFFER) {
    ssbo_len_++;
    names_size += PUSH_CONSTANTS_FALLBACK_NAME_LEN + 1;
  }
  else if (push_constants_storage_type == VKPushConstantsLayout::StorageType::UNIFORM_BUFFER) {
    ubo_len_++;
    names_size += PUSH_CONSTANTS_FALLBACK_NAME_LEN + 1;
  }

  /* Make sure that the image slots don't overlap with the sampler slots.*/
  image_offset_++;

  int32_t input_tot_len = ubo_len_ + uniform_len_ + ssbo_len_;
  inputs_ = static_cast<ShaderInput *>(
      MEM_calloc_arrayN(input_tot_len, sizeof(ShaderInput), __func__));
  ShaderInput *input = inputs_;

  name_buffer_ = (char *)MEM_mallocN(names_size, "name_buffer");
  uint32_t name_buffer_offset = 0;

  int32_t location = 0;

  /* Uniform blocks */
  for (const ShaderCreateInfo::Resource &res : all_resources) {
    if (res.bind_type == ShaderCreateInfo::Resource::BindType::UNIFORM_BUFFER) {
      copy_input_name(input, res.image.name, name_buffer_, name_buffer_offset);
      input->location = location++;
      input->binding = res.slot;
      input++;
    }
  }
  /* Add push constant when using uniform buffer as fallback. */
  int32_t push_constants_fallback_location = -1;
  ShaderInput *push_constants_fallback_input = nullptr;
  if (push_constants_storage_type == VKPushConstantsLayout::StorageType::UNIFORM_BUFFER) {
    copy_input_name(input, PUSH_CONSTANTS_FALLBACK_NAME, name_buffer_, name_buffer_offset);
    input->location = push_constants_fallback_location = location++;
    input->binding = -1;
    push_constants_fallback_input = input;
    input++;
  }

  /* Images, Samplers and buffers. */
  for (const ShaderCreateInfo::Resource &res : all_resources) {
    if (res.bind_type == ShaderCreateInfo::Resource::BindType::SAMPLER) {
      copy_input_name(input, res.sampler.name, name_buffer_, name_buffer_offset);
      input->location = location++;
      input->binding = res.slot;
      input++;
    }
    else if (res.bind_type == ShaderCreateInfo::Resource::BindType::IMAGE) {
      copy_input_name(input, res.image.name, name_buffer_, name_buffer_offset);
      input->location = location++;
      input->binding = res.slot + image_offset_;
      input++;
    }
  }

  /* Push constants. */
  /* NOTE: Push constants must be added after other uniform resources as resources have strict
   * rules for their 'location' due to descriptor sets. Push constants only need an unique location
   * as it is only used by the GPU module internally.*/
  int32_t push_constant_location = location + 1024;
  for (const ShaderCreateInfo::PushConst &push_constant : info.push_constants_) {
    copy_input_name(input, push_constant.name, name_buffer_, name_buffer_offset);
    input->location = push_constant_location++;
    input->binding = -1;
    input++;
  }

  /* Storage buffers */
  for (const ShaderCreateInfo::Resource &res : all_resources) {
    if (res.bind_type == ShaderCreateInfo::Resource::BindType::STORAGE_BUFFER) {
      copy_input_name(input, res.storagebuf.name, name_buffer_, name_buffer_offset);
      input->location = location++;
      input->binding = res.slot;
      input++;
    }
  }

  /* Push constant post initialization.*/
  /*
   * When using storage buffer storage type we need to add it to the the name list here.
   * Also determine the location as this is used inside the descriptor set as its binding number.
   */
  if (push_constants_storage_type == VKPushConstantsLayout::StorageType::STORAGE_BUFFER) {
    copy_input_name(input, PUSH_CONSTANTS_FALLBACK_NAME, name_buffer_, name_buffer_offset);
    input->location = push_constants_fallback_location = location++;
    input->binding = -1;
    push_constants_fallback_input = input;
    input++;
  }
  push_constants_layout_.init(
      info, *this, push_constants_storage_type, push_constants_fallback_input);

  sort_inputs();
  debug_print();
}

const ShaderInput *VKShaderInterface::shader_input_get(
    const shader::ShaderCreateInfo::Resource &resource) const
{
  return shader_input_get(resource.bind_type, resource.slot);
}

const ShaderInput *VKShaderInterface::shader_input_get(
    const shader::ShaderCreateInfo::Resource::BindType &bind_type, int binding) const
{
  switch (bind_type) {
    case shader::ShaderCreateInfo::Resource::BindType::IMAGE:
      return texture_get(binding + image_offset_);
    case shader::ShaderCreateInfo::Resource::BindType::SAMPLER:
      return texture_get(binding);
    case shader::ShaderCreateInfo::Resource::BindType::STORAGE_BUFFER:
      return ssbo_get(binding);
    case shader::ShaderCreateInfo::Resource::BindType::UNIFORM_BUFFER:
      return ubo_get(binding);
  }
  return nullptr;
}

}  // namespace blender::gpu
