/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation */

/** \file
 * \ingroup gpu
 */

#include "vk_sampler.hh"
#include "vk_backend.hh"
#include "vk_context.hh"
#include "vk_memory.hh"

namespace blender::gpu {
VKSampler::~VKSampler()
{
  VKContext &context = *VKContext::get();
  free(context);
}

void VKSampler::create(VKContext &context)
{
  BLI_assert(vk_sampler_ == VK_NULL_HANDLE);

  VK_ALLOCATION_CALLBACKS

  VkSamplerCreateInfo sampler_info = {};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  const VKDevice &device = VKBackend::get().device_get();
  vkCreateSampler(device.device_get(), &sampler_info, vk_allocation_callbacks, &vk_sampler_);
  debug::object_label(&context, vk_sampler_, "DummySampler");
}

void VKSampler::free(VKContext & /*context*/)
{
  VK_ALLOCATION_CALLBACKS

  if (vk_sampler_ != VK_NULL_HANDLE) {
    const VKDevice &device = VKBackend::get().device_get();
    vkDestroySampler(device.device_get(), vk_sampler_, vk_allocation_callbacks);
    vk_sampler_ = VK_NULL_HANDLE;
  }
}

}  // namespace blender::gpu
