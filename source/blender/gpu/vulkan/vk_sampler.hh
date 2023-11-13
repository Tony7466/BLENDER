/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "gpu_shader_private.hh"

#include "vk_common.hh"

#include "BLI_utility_mixins.hh"

namespace blender::gpu {
class VKContext;

class VKSampler : public NonCopyable {
  VkSampler vk_sampler_ = VK_NULL_HANDLE;
  bool used_ = true;

 public:
  VKSampler() = default;
  VKSampler(VKSampler &&other)
  {
    vk_sampler_ = other.vk_sampler_;
    used_ = other.used_;

    other.vk_sampler_ = VK_NULL_HANDLE;
  }
  virtual ~VKSampler();
  void create(const GPUSamplerState &sampler_state);
  void free();

  VkSampler vk_handle() const
  {
    BLI_assert(vk_sampler_ != VK_NULL_HANDLE);
    return vk_sampler_;
  }

  bool is_initialized() const
  {
    return vk_sampler_ != VK_NULL_HANDLE;
  }

  void mark_unused()
  {
    used_ = false;
  }

  void mark_used()
  {
    used_ = true;
  }

  bool is_used() const
  {
    return used_;
  }
};

}  // namespace blender::gpu
