/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_samplers.hh"

namespace blender::gpu {

void VKSamplers::init() {}

void VKSamplers::free()
{
  samplers_.clear();
}

const VKSampler &VKSamplers::get(const GPUSamplerState &key)
{
  VKSampler &result = samplers_.lookup_or_add_default(key);
  if (!result.is_initialized()) {
    result.create(key);
    stats.created += 1;
  }

  result.mark_used();
  return result;
}

void VKSamplers::discard_unused()
{
  int64_t size_before = samplers_.size();
  samplers_.remove_if([](const Map<GPUSamplerState, VKSampler>::MutableItem &item) {
    return !item.value.is_used();
  });
  int64_t size_after = samplers_.size();
  stats.freed += size_before - size_after;
}

void VKSamplers::mark_all_unused()
{
  for (VKSampler &sampler : samplers_.values()) {
    sampler.mark_unused();
  }
}

void VKSamplers::debug_print() const
{
  std::cout << "VKSamplers(created=" << stats.created;
  std::cout << ", active=" << samplers_.size();
  std::cout << ", freed=" << stats.freed << ")\n";
}

}  // namespace blender::gpu
