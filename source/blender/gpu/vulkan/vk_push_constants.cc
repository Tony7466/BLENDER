/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_push_constants.hh"
#include "vk_backend.hh"
#include "vk_memory.hh"
#include "vk_shader_interface.hh"

namespace blender::gpu {

template<typename Layout>
static void align(const shader::ShaderCreateInfo::PushConst &push_constant, uint32_t *r_offset)
{
  uint32_t alignment = Layout::element_alignment(push_constant.type);
  uint32_t alignment_mask = alignment - 1;
  uint32_t offset = *r_offset;
  if ((offset & alignment_mask) != 0) {
    offset &= ~alignment_mask;
    offset += alignment;
    *r_offset = offset;
  }
}

template<typename Layout>
static void reserve(const shader::ShaderCreateInfo::PushConst &push_constant, uint32_t *r_offset)
{
  uint32_t size = Layout::element_components_len(push_constant.type) *
                  Layout::component_mem_size(push_constant.type);
  if (push_constant.array_size != 0) {
    size *= push_constant.array_size;
  }
  *r_offset += size;
}

template<typename Layout>
static VKPushConstantsLayout::PushConstantLayout init_constant(
    const shader::ShaderCreateInfo::PushConst &push_constant,
    const ShaderInput &shader_input,
    uint32_t *r_offset)
{
  align<Layout>(push_constant, r_offset);

  VKPushConstantsLayout::PushConstantLayout layout;
  layout.location = shader_input.location;
  layout.type = push_constant.type;
  layout.array_size = push_constant.array_size;
  layout.offset = *r_offset;

  reserve<Layout>(push_constant, r_offset);
  return layout;
}

VKPushConstantsLayout::StorageType VKPushConstantsLayout::determine_storage_type(
    const shader::ShaderCreateInfo &info, const VkPhysicalDeviceLimits &vk_physical_device_limits)
{
  if (info.push_constants_.is_empty()) {
    return StorageType::NONE;
  }

  uint32_t offset = 0;
  for (const shader::ShaderCreateInfo::PushConst &push_constant : info.push_constants_) {
    align<Std430>(push_constant, &offset);
    reserve<Std430>(push_constant, &offset);
  }
  return offset <= vk_physical_device_limits.maxPushConstantsSize ? StorageType::PUSH_CONSTANTS :
                                                                    StorageType::STORAGE_BUFFER;
}

void VKPushConstantsLayout::init(const shader::ShaderCreateInfo &info,
                                 const VKShaderInterface &interface,
                                 const StorageType storage_type,
                                 const ShaderInput *shader_input)
{
  BLI_assert(push_constants.is_empty());
  storage_type_ = storage_type;
  if (storage_type == StorageType::STORAGE_BUFFER) {
    BLI_assert(shader_input);
    storage_buffer_binding_ = VKDescriptorSet::Location(shader_input);
  }
  uint32_t offset = 0;
  for (const shader::ShaderCreateInfo::PushConst &push_constant : info.push_constants_) {
    const ShaderInput *shader_input = interface.uniform_get(push_constant.name.c_str());
    BLI_assert(shader_input);
    push_constants.append(init_constant<Std430>(push_constant, *shader_input, &offset));
  }
  size_in_bytes_ = offset;
}

const VKPushConstantsLayout::PushConstantLayout *VKPushConstantsLayout::find(
    int32_t location) const
{
  for (const PushConstantLayout &push_constant : push_constants) {
    if (push_constant.location == location) {
      return &push_constant;
    }
  }
  return nullptr;
}
VKPushConstants::VKPushConstants() = default;
VKPushConstants::VKPushConstants(const VKPushConstantsLayout *layout) : layout_(layout)
{
  data_ = MEM_mallocN(layout->size_in_bytes(), __func__);
  if (storage_type_get() == VKPushConstantsLayout::StorageType::STORAGE_BUFFER) {
    storage_buffer_ = new VKStorageBuffer(size_in_bytes(), GPU_USAGE_DYNAMIC, __func__);
  }
}

VKPushConstants::VKPushConstants(VKPushConstants &&other) : layout_(other.layout_)
{
  data_ = other.data_;
  other.data_ = nullptr;

  storage_buffer_ = other.storage_buffer_;
  other.storage_buffer_ = nullptr;
}

VKPushConstants::~VKPushConstants()
{
  if (data_ != nullptr) {
    MEM_freeN(data_);
    data_ = nullptr;
  }

  delete storage_buffer_;
  storage_buffer_ = nullptr;
}

VKPushConstants &VKPushConstants::operator=(VKPushConstants &&other)
{
  layout_ = other.layout_;

  data_ = other.data_;
  other.data_ = nullptr;

  storage_buffer_ = other.storage_buffer_;
  other.storage_buffer_ = nullptr;

  return *this;
}

void VKPushConstants::update_storage_buffer(VkDevice /*vk_device*/)
{
  BLI_assert(storage_type_get() == VKPushConstantsLayout::StorageType::STORAGE_BUFFER);
  BLI_assert(storage_buffer_ != nullptr);
  BLI_assert(data_ != nullptr);
  storage_buffer_->update(data_);
}

VKStorageBuffer &VKPushConstants::storage_buffer_get()
{
  BLI_assert(storage_type_get() == VKPushConstantsLayout::StorageType::STORAGE_BUFFER);
  BLI_assert(storage_buffer_ != nullptr);
  return *storage_buffer_;
}

}  // namespace blender::gpu
