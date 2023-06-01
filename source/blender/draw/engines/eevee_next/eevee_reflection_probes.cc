/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#include "eevee_reflection_probes.hh"
#include "eevee_instance.hh"

namespace blender::eevee {

void ReflectionProbeModule::init()
{
  if (cubemaps_.is_empty()) {
    cubemaps_.reserve(MAX_PROBES);

    /* Initialize the world cubemap. */
    ReflectionProbe world_cubemap;
    world_cubemap.type = ReflectionProbe::Type::World;
    world_cubemap.is_dirty = true;
    cubemaps_.append(world_cubemap);

    cubemaps_tx_.ensure_cube_array(GPU_RGBA16F,
                                   MAX_RESOLUTION,
                                   MAX_PROBES,
                                   GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT,
                                   NULL,
                                   MIPMAP_LEVELS);
    GPU_texture_mipmap_mode(cubemaps_tx_, true, true);
  }
}

void ReflectionProbeModule::sync()
{
  for (int index : IndexRange(MAX_PROBES)) {
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
  switch (cubemap.type) {
    case ReflectionProbe::Type::World: {
      GPUMaterial *world_material = instance_.world.get_world_material();
      instance_.pipelines.world_probe.sync(world_material);
      break;
    }
    case ReflectionProbe::Type::Probe: {
      /* TODO: Implement*/
      break;
    }
    case ReflectionProbe::Type::Unused: {
      break;
    }
  }
}

void ReflectionProbeModule::set_world_dirty()
{
  cubemaps_[WORLD_SLOT].is_dirty = true;
}

void ReflectionProbeModule::sync_object(Object *ob,
                                        ObjectHandle &ob_handle,
                                        ResourceHandle res_handle,
                                        bool is_dirty)
{
  ReflectionProbe &probe = find_or_insert(ob_handle);
  probe.is_dirty |= is_dirty;
}

ReflectionProbe &ReflectionProbeModule::find_or_insert(ObjectHandle &ob_handle)
{
  ReflectionProbe *first_unused = nullptr;
  for (ReflectionProbe &reflection_probe : cubemaps_) {
    if (reflection_probe.object_hash_value == ob_handle.object_key.hash_value) {
      return reflection_probe;
    }
    if (first_unused == nullptr && reflection_probe.type == ReflectionProbe::Type::Unused) {
      first_unused = &reflection_probe;
    }
  }

  if (first_unused == nullptr) {
    ReflectionProbe new_slot;
    cubemaps_.append_as(new_slot);
    first_unused = &cubemaps_.last();
  }
  BLI_assert(first_unused != nullptr);
  first_unused->is_dirty = true;
  first_unused->object_hash_value = ob_handle.object_key.hash_value;
  first_unused->type = ReflectionProbe::Type::Probe;

  /* TODO: We should free slots that aren't used anymore. This could be implemented with a tag
   * or priority list. */
  return *first_unused;
}

/* -------------------------------------------------------------------- */
/** \name World
 *
 * \{ */

bool ReflectionProbe::needs_update() const
{
  return type != Type::Unused && is_dirty;
}

/** \} */

}  // namespace blender::eevee
