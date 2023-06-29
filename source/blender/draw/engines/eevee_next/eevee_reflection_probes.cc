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

void ReflectionProbeModule::sync()
{
  for (int index : IndexRange(max_probes)) {
    ReflectionProbe &cubemap = cubemaps_[index];
    if (!cubemap.needs_update()) {
      continue;
    }
    sync(cubemap);
    cubemap.is_dirty = false;
  }
}

void ReflectionProbeModule::sync(const ReflectionProbe &cubemap)
{
  if (cubemap.type == ReflectionProbe::Type::WORLD) {
    GPUMaterial *world_material = instance_.world.get_world_material();
    instance_.pipelines.world.sync(world_material);
  }
  else {
    BLI_assert_unreachable();
  }
}

void ReflectionProbeModule::set_world_dirty()
{
  cubemaps_[world_slot].is_dirty = true;
}

/* -------------------------------------------------------------------- */
/** \name World
 *
 * \{ */

bool ReflectionProbe::needs_update() const
{
  return type != Type::UNUSED && is_dirty;
}

/** \} */

}  // namespace blender::eevee
