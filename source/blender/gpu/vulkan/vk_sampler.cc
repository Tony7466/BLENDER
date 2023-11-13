/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_sampler.hh"
#include "vk_backend.hh"
#include "vk_context.hh"
#include "vk_memory.hh"

#include "DNA_userdef_types.h"

namespace blender::gpu {
VKSampler::~VKSampler()
{
  free();
}

void VKSampler::create(const GPUSamplerState &sampler_state)
{
  BLI_assert(vk_sampler_ == VK_NULL_HANDLE);

  VK_ALLOCATION_CALLBACKS

  VkSamplerCreateInfo sampler_info = {};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;

  /* Apply filtering. */
  if (sampler_state.filtering & GPU_SAMPLER_FILTERING_LINEAR) {
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
  }
  if (sampler_state.filtering & GPU_SAMPLER_FILTERING_MIPMAP) {
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  }
  if (sampler_state.filtering & GPU_SAMPLER_FILTERING_ANISOTROPIC) {
    sampler_info.anisotropyEnable = VK_TRUE;
    sampler_info.maxAnisotropy = min_ff(1.0f, U.anisotropic_filter);
  }

  /* */
  NOT_YET_IMPLEMENTED;

  const VKDevice &device = VKBackend::get().device_get();
  vkCreateSampler(device.device_get(), &sampler_info, vk_allocation_callbacks, &vk_sampler_);
  debug::object_label(vk_sampler_, "Sampler");
}

void VKSampler::free()
{

  if (vk_sampler_ != VK_NULL_HANDLE) {
    VK_ALLOCATION_CALLBACKS

    const VKDevice &device = VKBackend::get().device_get();
    if (device.device_get() != VK_NULL_HANDLE) {
      vkDestroySampler(device.device_get(), vk_sampler_, vk_allocation_callbacks);
    }
    vk_sampler_ = VK_NULL_HANDLE;
  }
}

}  // namespace blender::gpu
