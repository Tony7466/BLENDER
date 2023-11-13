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

struct VKSamplerKey {
};

/**
 * Collection of samplers currently in use.
 *
 * In Vulkan samplers are device owned and can be shared between contexts.
 */
class VKSamplers : NonCopyable {
  Map<VKSamplerKey, VKSampler> samplers_;
  VKSampler sampler_;

 public:
  void init();
  void free();

  const VKSampler &get(const VKSamplerKey &key);
};

}  // namespace blender::gpu
