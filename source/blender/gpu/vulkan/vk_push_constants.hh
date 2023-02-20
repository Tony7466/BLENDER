/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_shader_interface.hh"

namespace blender::gpu {

struct VKPushConstantsLayout {
  struct PushConstantLayout {
    /* TODO: location requires sequential lookups, we should make the location index based for
     * quicker access. */
    int32_t location;

    /** Offset in the push constant data (in bytes). */
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

  const PushConstantLayout *find(int32_t location) const;
};

static VKPushConstantsLayout dummy_layout;
class VKPushConstants : NonCopyable {

 private:
  VKPushConstantsLayout &layout_ = dummy_layout;
  void *data_ = nullptr;

 public:
  VKPushConstants() = default;
  VKPushConstants(VKPushConstantsLayout &layout);
  VKPushConstants(VKPushConstants &&other);
  virtual ~VKPushConstants();

  VKPushConstants &operator=(VKPushConstants &&other);

  size_t offset() const
  {
    return 0;
  }

  size_t size_in_bytes() const
  {
    return layout_.size_in_bytes();
  }

  const void *data() const
  {
    return data_;
  }

  template<typename T>
  void push_constant_set(int32_t location,
                         int32_t comp_len,
                         int32_t array_size,
                         const T *input_data)
  {
    const VKPushConstantsLayout::PushConstantLayout *push_constant_layout = layout_.find(location);
    if (push_constant_layout == nullptr) {
      /* Currently the builtin uniforms are set using a predefined location each time a shader is
       * bound.*/
      return;
    }
    BLI_assert_msg(push_constant_layout->offset + comp_len * array_size * sizeof(T) <=
                       layout_.size_in_bytes(),
                   "Tried to write outside the push constant allocated memory.");
    uint8_t *bytes = static_cast<uint8_t *>(data_);
    T *dst = static_cast<T *>(static_cast<void *>(&bytes[push_constant_layout->offset]));
    memcpy(dst, input_data, comp_len * array_size * sizeof(T));
  }
};

}  // namespace blender::gpu
