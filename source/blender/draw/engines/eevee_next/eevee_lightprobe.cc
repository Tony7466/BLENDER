/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * Module that handles light probe update tagging.
 * Lighting data is contained in their respective module `IrradianceCache` and `ReflectionProbes`.
 */

#include "DNA_lightprobe_types.h"
#include "WM_api.hh"

#include "eevee_instance.hh"
#include "eevee_lightprobe.hh"

#include "draw_debug.hh"

namespace blender::eevee {

void LightProbeModule::begin_sync()
{
  auto_bake_enabled_ = inst_.is_viewport() &&
                       (inst_.scene->eevee.flag & SCE_EEVEE_GI_AUTOBAKE) != 0;
}

void LightProbeModule::sync_grid(const Object *ob, ObjectHandle &handle)
{
  IrradianceGrid &grid = grid_map_.lookup_or_add_default(handle.object_key);
  grid.used = true;
  if (handle.recalc != 0 || grid.initialized == false) {
    const ::LightProbe *lightprobe = static_cast<const ::LightProbe *>(ob->data);

    grid.initialized = true;
    grid.updated = true;
    grid.surfel_density = static_cast<const ::LightProbe *>(ob->data)->surfel_density;
    grid.object_to_world = float4x4(ob->object_to_world);
    grid.world_to_object = float4x4(
        math::normalize(math::transpose(float3x3(grid.object_to_world))));

    grid.cache = ob->lightprobe_cache;
    grid.normal_bias = lightprobe->grid_normal_bias;
    grid.view_bias = lightprobe->grid_view_bias;
    grid.facing_bias = lightprobe->grid_facing_bias;

    grid.validity_threshold = lightprobe->grid_validity_threshold;
    grid.dilation_threshold = lightprobe->grid_dilation_threshold;
    grid.dilation_radius = lightprobe->grid_dilation_radius;
    grid.intensity = lightprobe->intensity;

    grid.viewport_display = lightprobe->flag & LIGHTPROBE_FLAG_SHOW_DATA;
    grid.viewport_display_size = lightprobe->data_display_size;

    /* Force reupload. */
    inst_.irradiance_cache.bricks_free(grid.bricks);
  }
}

void LightProbeModule::sync_cube(const Object * /*ob*/, ObjectHandle &handle)
{
  ReflectionCube &cube = cube_map_.lookup_or_add_default(handle.object_key);
  cube.used = true;
  if (handle.recalc != 0 || cube.initialized == false) {
    cube.initialized = true;
    cube.updated = true;
  }
}

void LightProbeModule::sync_plane(const Object *ob, ObjectHandle &handle)
{
  ProbePlane &plane = plane_map_.lookup_or_add_default(handle.object_key);
  plane.used = true;
  if (handle.recalc != 0 || plane.initialized == false) {
    const ::LightProbe *light_probe = (::LightProbe *)ob->data;

    plane.initialized = true;
    plane.updated = true;
    plane.plane_to_world = float4x4(ob->object_to_world);
    plane.plane_to_world.z_axis() = math::normalize(plane.plane_to_world.z_axis()) *
                                    light_probe->distinf;
    plane.world_to_plane = math::invert(plane.plane_to_world);
    plane.clipping_offset = light_probe->clipsta;
    plane.viewport_display = (light_probe->flag & LIGHTPROBE_FLAG_SHOW_DATA) != 0;
  }
}

void LightProbeModule::sync_probe(const Object *ob, ObjectHandle &handle)
{
  const ::LightProbe *lightprobe = static_cast<const ::LightProbe *>(ob->data);
  switch (lightprobe->type) {
    case LIGHTPROBE_TYPE_SPHERE:
      sync_cube(ob, handle);
      return;
    case LIGHTPROBE_TYPE_PLANE:
      sync_plane(ob, handle);
      return;
    case LIGHTPROBE_TYPE_VOLUME:
      sync_grid(ob, handle);
      return;
  }
  BLI_assert_unreachable();
}

void LightProbeModule::end_sync()
{
  /* Check for deleted or updated grid. */
  grid_update_ = false;
  grid_map_.remove_if([&](const Map<ObjectKey, IrradianceGrid>::MutableItem &item) {
    IrradianceGrid &grid = item.value;
    bool remove_grid = !grid.used;
    if (grid.updated || remove_grid) {
      grid_update_ = true;
    }
    grid.updated = false;
    grid.used = false;
    return remove_grid;
  });

  /* Check for deleted or updated cube. */
  cube_update_ = false;
  cube_map_.remove_if([&](const Map<ObjectKey, ReflectionCube>::MutableItem &item) {
    ReflectionCube &cube = item.value;
    bool remove_cube = !cube.used;
    if (cube.updated || remove_cube) {
      cube_update_ = true;
    }
    cube.updated = false;
    cube.used = false;
    return remove_cube;
  });

  /* Check for deleted or updated plane. */
  plane_update_ = false;
  plane_map_.remove_if([&](const Map<ObjectKey, ProbePlane>::MutableItem &item) {
    ProbePlane &plane = item.value;
    bool remove_plane = !plane.used;
    if (plane.updated || remove_plane) {
      plane_update_ = true;
    }
    plane.updated = false;
    plane.used = false;
    return remove_plane;
  });
}

}  // namespace blender::eevee
