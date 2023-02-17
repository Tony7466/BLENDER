/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_shader_interface.hh"

namespace blender::gpu {

struct VKPushConstantsLayout {
  struct PushConstantLayout {
    uint32_t offset;
    shader::Type type;
    int array_size;
  };

 private:
  Vector<PushConstantLayout> push_constants;
  uint32_t size_in_bytes_;

 public:
  void init(const shader::ShaderCreateInfo &info, const VKShaderInterface &interface);

  uint32_t size_in_bytes() const
  {
    return size_in_bytes_;
  }
};

struct VKPushConstants {
  const VKPushConstantsLayout &layout_;
  uint32_t *data = nullptr;

  VKPushConstants(const VKPushConstantsLayout &layout);
};

}  // namespace blender::gpu
