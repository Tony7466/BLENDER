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
  sampler_.create();
}

void VKSamplers::free()
{
  sampler_.free();
  samplers_.clear();
}

const VKSampler &VKSamplers::get(const VKSamplerKey &key)
{
  return sampler_;
}

}  // namespace blender::gpu
