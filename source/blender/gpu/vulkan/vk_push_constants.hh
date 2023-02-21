/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 *
 * Push constants is a way to quickly provide a small amount of uniform data to shaders. It should
 * be much quicker than UBOs but a huge limitation is the size of data - spec requires 128 bytes to
 * be available for a push constant range. Hardware vendors may support more, but compared to other
 * means it is still very little (for example 256 bytes).
 *
 * Due to this size requirements we try to use push constants when it fits on the device. If it
 * doesn't fit we fallback to use an uniform buffer.
 *
 * Shader developers are responsible to fine-tune the performance of the shader. One way to do this
 * is to tailor what will be sent as a push constant to keep the push constants within the limits.
 */

#pragma once

#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#include "gpu_shader_create_info.hh"

#include "vk_common.hh"
#include "vk_descriptor_set.hh"

namespace blender::gpu {
class VKShaderInterface;
class VKUniformBuffer;
class VKStorageBuffer;

/**
 * Describe the layout of the push constants and the storage type that should be used.
 */
struct VKPushConstantsLayout {
  /* Different methods to store push constants.*/
  enum class StorageType {
    /** Push constants aren't in use.*/
    NONE,

    /** Store push constants as regular vulkan push constants.*/
    PUSH_CONSTANTS,

    /**
     * Fallback when push constants doesn't meet the device requirements. This fallback uses a
     * storage buffer.
     */
    STORAGE_BUFFER,

    /**
     * Fallback when push constants doesn't meet the device requirements. This fallback uses an
     * uniform buffer.
     */
    UNIFORM_BUFFER,
  };
  static constexpr StorageType STORAGE_TYPE_DEFAULT = StorageType::PUSH_CONSTANTS;
  static constexpr StorageType STORAGE_TYPE_FALLBACK = StorageType::UNIFORM_BUFFER;

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
  uint32_t size_in_bytes_ = 0;
  StorageType storage_type_ = StorageType::NONE;
  /**
   * Binding index in the descriptor set when the push constants use an uniform buffer.
   */
  VKDescriptorSet::Location storage_buffer_binding_;

 public:
  static StorageType determine_storage_type(
      const shader::ShaderCreateInfo &info,
      const VkPhysicalDeviceLimits &vk_physical_device_limits);
  void init(const shader::ShaderCreateInfo &info,
            const VKShaderInterface &interface,
            StorageType storage_type,
            const ShaderInput *shader_input);

  /**
   * Return the storage type that is used.
   */
  StorageType storage_type_get() const
  {
    return storage_type_;
  }

  VKDescriptorSet::Location storage_buffer_binding_get() const
  {
    return storage_buffer_binding_;
  }

  uint32_t size_in_bytes() const
  {
    return size_in_bytes_;
  }

  const PushConstantLayout *find(int32_t location) const;
};

class VKPushConstants : NonCopyable {

 private:
  const VKPushConstantsLayout *layout_ = nullptr;
  void *data_ = nullptr;
  VKStorageBuffer *storage_buffer_ = nullptr;
  VKUniformBuffer *uniform_buffer_ = nullptr;

 public:
  VKPushConstants();
  VKPushConstants(const VKPushConstantsLayout *layout);
  VKPushConstants(VKPushConstants &&other);
  virtual ~VKPushConstants();

  VKPushConstants &operator=(VKPushConstants &&other);

  size_t offset() const
  {
    return 0;
  }

  const VKPushConstantsLayout &layout_get() const
  {
    return *layout_;
  }

  void update_storage_buffer(VkDevice vk_device);
  VKStorageBuffer &storage_buffer_get();

  void update_uniform_buffer(VkDevice vk_device);
  VKUniformBuffer &uniform_buffer_get();

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
    const VKPushConstantsLayout::PushConstantLayout *push_constant_layout = layout_->find(
        location);
    if (push_constant_layout == nullptr) {
      /* TODO: Currently the builtin uniforms are set using a predefined location each time a
       * shader is bound. This needs to be fixed in the VKShaderInterface.*/
      return;
    }
    BLI_assert_msg(push_constant_layout->offset + comp_len * array_size * sizeof(T) <=
                       layout_->size_in_bytes(),
                   "Tried to write outside the push constant allocated memory.");
    uint8_t *bytes = static_cast<uint8_t *>(data_);
    T *dst = static_cast<T *>(static_cast<void *>(&bytes[push_constant_layout->offset]));
    memcpy(dst, input_data, comp_len * array_size * sizeof(T));
  }
};

}  // namespace blender::gpu
