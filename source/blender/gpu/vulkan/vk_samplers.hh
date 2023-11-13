/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_sampler.hh"
#include "vk_samplers.hh"

#include "BLI_map.hh"

namespace blender::gpu {

struct GPUSamplerStateHasher {
  uint64_t operator()(const GPUSamplerState &value) const
  {
    uint64_t hash = 0;
    hash = (hash << 0) | value.filtering;
    hash = (hash << 8) | value.extend_x;
    hash = (hash << 4) | value.extend_yz;
    hash = (hash << 4) | value.custom_type;
    hash = (hash << 8) | value.type;
    return hash;
  }
};

/**
 * Collection of samplers currently in use.
 *
 * In Vulkan samplers are device owned and can be shared between contexts.
 */
class VKSamplers : NonCopyable {
  Map<GPUSamplerState,
      VKSampler,
      default_inline_buffer_capacity(sizeof(GPUSamplerState) + sizeof(VKSampler)),
      DefaultProbingStrategy,
      GPUSamplerStateHasher>
      samplers_;

 public:
  void init();
  void free();

  const VKSampler &get(const GPUSamplerState &key);

  void discard_unused();
  void mark_all_unused();
};

}  // namespace blender::gpu
