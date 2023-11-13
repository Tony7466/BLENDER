/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_samplers.hh"

namespace blender::gpu {

void VKSamplers::init()
{
}

void VKSamplers::free()
{
  samplers_.clear();
}

const VKSampler &VKSamplers::get(const GPUSamplerState &key)
{
  VKSampler &result = samplers_.lookup_or_add_default(key);
  if (!result.is_initialized()) {
    result.create(key);
  }

  result.mark_used();
  return result;
}

void VKSamplers::discard_unused()
{
  samplers_.remove_if([](const Map<GPUSamplerState, VKSampler>::MutableItem &item) {
    return !item.value.is_used();
  });
}

void VKSamplers::mark_all_unused()
{
  for (VKSampler &sampler : samplers_.values()) {
    sampler.mark_unused();
  }
}

}  // namespace blender::gpu
