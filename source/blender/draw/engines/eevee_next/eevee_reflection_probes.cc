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

  for (ReflectionProbe &reflection_probe : cubemaps_) {
    if (reflection_probe.type == ReflectionProbe::Type::Probe) {
      reflection_probe.is_used = false;
    }
  }
}

void ReflectionProbeModule::sync()
{
  for (ReflectionProbe &cubemap : cubemaps_) {
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
    if (reflection_probe.type == ReflectionProbe::Type::Probe &&
        reflection_probe.object_hash_value == ob_handle.object_key.hash_value)
    {
      reflection_probe.is_used = true;
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
  first_unused->is_used = true;
  first_unused->object_hash_value = ob_handle.object_key.hash_value;
  first_unused->type = ReflectionProbe::Type::Probe;

  return *first_unused;
}

void ReflectionProbeModule::end_sync()
{
  for (ReflectionProbe &probe : cubemaps_) {
    if (probe.type == ReflectionProbe::Type::Probe && !probe.is_used) {
      probe.type = ReflectionProbe::Type::Unused;
      probe.object_hash_value = 0;
    }
  }
}

/* -------------------------------------------------------------------- */
/** \name World
 *
 * \{ */

bool ReflectionProbe::needs_update() const
{
  switch (type) {
    case Type::Unused:
      return false;
    case Type::World:
      return is_dirty;
    case Type::Probe:
      return is_dirty && is_used;
  }
  return false;
}

/** \} */

}  // namespace blender::eevee
