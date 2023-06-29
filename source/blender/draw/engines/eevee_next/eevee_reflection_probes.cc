/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#include "eevee_reflection_probes.hh"
#include "eevee_instance.hh"

namespace blender::eevee {

void ReflectionProbeModule::init()
{
  if (cubemaps_.is_empty()) {
    cubemaps_.reserve(max_probes);

    /* Initialize the world cubemap. */
    ReflectionProbe world_cubemap;
    world_cubemap.type = ReflectionProbe::Type::WORLD;
    world_cubemap.is_dirty = true;
    cubemaps_.append(world_cubemap);

    cubemaps_tx_.ensure_cube_array(GPU_RGBA16F,
                                   max_resolution,
                                   max_probes,
                                   GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT,
                                   NULL,
                                   max_mipmap_levels);
    GPU_texture_mipmap_mode(cubemaps_tx_, true, true);
  }
}

}  // namespace blender::eevee
