/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#include "eevee_reflection_probes.hh"
#include "eevee_instance.hh"

namespace blender::eevee {

void ReflectionProbeModule::init()
{
  if (!initialized_) {
    cubemaps_tx_.ensure_cube_array(GPU_RGBA16F,
                                   max_resolution_,
                                   max_probes_,
                                   GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT,
                                   NULL,
                                   max_mipmap_levels_);
    GPU_texture_mipmap_mode(cubemaps_tx_, true, true);
    initialized_ = true;

    dummy_tx_.ensure_cube_array(
        GPU_RGBA16F, 1, 1, GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT, NULL, 1);
    dummy_tx_.clear(float4(0.0f));
    GPU_texture_mipmap_mode(dummy_tx_, false, false);
  }
}

}  // namespace blender::eevee
