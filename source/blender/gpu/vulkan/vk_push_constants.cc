/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_push_constants.hh"

namespace blender::gpu {

static uint32_t to_component_size(const shader::Type /*type*/)
{
  return 4;
}

static uint32_t to_num_components(const shader::Type type)
{
  switch (type) {
    case shader::Type::FLOAT:
    case shader::Type::UINT:
    case shader::Type::INT:
    case shader::Type::BOOL:
      return 1;
    case shader::Type::VEC2:
    case shader::Type::UVEC2:
    case shader::Type::IVEC2:
      return 2;
    case shader::Type::VEC3:
    case shader::Type::UVEC3:
    case shader::Type::IVEC3:
    case shader::Type::VEC4:
    case shader::Type::UVEC4:
    case shader::Type::IVEC4:
      return 4;
    case shader::Type::MAT3:
      return 12;
    case shader::Type::MAT4:
      return 16;
    default:
      BLI_assert_msg(false, "Type not supported as push constant");
  }
  return 0;
}

static uint32_t to_alignment(const shader::Type type)
{
  switch (type) {
    case shader::Type::FLOAT:
    case shader::Type::UINT:
    case shader::Type::INT:
    case shader::Type::BOOL:
      return 4;
    case shader::Type::VEC2:
    case shader::Type::UVEC2:
    case shader::Type::IVEC2:
      return 8;
    case shader::Type::VEC3:
    case shader::Type::UVEC3:
    case shader::Type::IVEC3:
    case shader::Type::VEC4:
    case shader::Type::UVEC4:
    case shader::Type::IVEC4:
    case shader::Type::MAT3:
    case shader::Type::MAT4:
      return 16;
    default:
      BLI_assert_msg(false, "Type not supported as push constant");
  }
  return 0;
}

static VKPushConstantsLayout::PushConstantLayout init_constant(
    const shader::ShaderCreateInfo::PushConst &push_constant,
    const ShaderInput &shader_input,
    uint32_t *r_offset)
{
  VKPushConstantsLayout::PushConstantLayout layout;
  layout.location = shader_input.location;
  layout.type = push_constant.type;
  layout.array_size = push_constant.array_size;
  layout.offset = *r_offset;

  /* Perform alignment. */
  uint32_t alignment = to_alignment(push_constant.type);
  uint32_t alignment_mask = alignment - 1;
  if ((layout.offset & alignment_mask) != 0) {
    layout.offset &= ~alignment_mask;
    layout.offset += alignment;
  }

  uint32_t size = to_num_components(push_constant.type) * to_component_size(push_constant.type);
  if (push_constant.array_size != 0) {
    size *= push_constant.array_size;
  }

  *r_offset = layout.offset + size;
  return layout;
}

void VKPushConstantsLayout::init(const shader::ShaderCreateInfo &info,
                                 const VKShaderInterface &interface)
{
  BLI_assert(push_constants.is_empty());
  uint32_t offset = 0;
  for (const shader::ShaderCreateInfo::PushConst &push_constant : info.push_constants_) {
    const ShaderInput *shader_input = interface.uniform_get(push_constant.name.c_str());
    BLI_assert(shader_input);
    push_constants.append(init_constant(push_constant, *shader_input, &offset));
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

VKPushConstants::VKPushConstants(VKPushConstantsLayout &layout) : layout_(layout)
{
  data_ = MEM_mallocN(layout.size_in_bytes(), __func__);
}

VKPushConstants::VKPushConstants(VKPushConstants &&other) : layout_(other.layout_)
{
  data_ = other.data_;
  other.data_ = nullptr;
}

VKPushConstants::~VKPushConstants()
{
  if (data_ != nullptr) {
    MEM_freeN(data_);
    data_ = nullptr;
  }
}

VKPushConstants &VKPushConstants::operator=(VKPushConstants &&other)
{
  layout_ = other.layout_;
  data_ = other.data_;
  other.data_ = nullptr;

  return *this;
}

}  // namespace blender::gpu
