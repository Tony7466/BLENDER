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

void LightProbeModule::sync_cube(const Object *ob, ObjectHandle &handle)
{
  ReflectionCube &cube = cube_map_.lookup_or_add_default(handle.object_key);
  cube.used = true;
  if (handle.recalc != 0 || cube.initialized == false) {
    const ::LightProbe &light_probe = *(::LightProbe *)ob->data;

    cube.initialized = true;
    cube.updated = true;
    cube.do_render = true;

    ReflectionProbeModule &probe_module = inst_.reflection_probes;
    eLightProbeResolution probe_resolution = probe_module.reflection_probe_resolution();
    int max_resolution = probe_module.max_resolution_;
    int subdivision_lvl = ReflectionCube::subdivision_level_get(max_resolution, probe_resolution);

    if (cube.atlas_coord.layer_subdivision != subdivision_lvl) {
      cube.atlas_coord = probe_module.find_empty_atlas_region(subdivision_lvl);
      ReflectionProbeData &cube_data = *static_cast<ReflectionProbeData *>(&cube);
      /* Update gpu data sampling coordinates. */
      cube_data.atlas_coord = cube.atlas_coord.as_sampling_coord(max_resolution);
      /* Coordinates have changed. Area might contain random data. Do not use for rendering. */
      cube.use_for_render = false;
    }

    bool use_custom_parallax = (light_probe.flag & LIGHTPROBE_FLAG_CUSTOM_PARALLAX) != 0;
    float influence_distance = light_probe.distinf;
    float influence_falloff = light_probe.falloff;
    float parallax_distance = light_probe.distpar;
    parallax_distance = use_custom_parallax ? max_ff(parallax_distance, influence_distance) :
                                              influence_distance;

    auto to_eevee_shape = [](int bl_shape_type) {
      return (bl_shape_type == LIGHTPROBE_SHAPE_BOX) ? SHAPE_CUBOID : SHAPE_ELIPSOID;
    };
    cube.influence_shape = to_eevee_shape(light_probe.attenuation_type);
    cube.parallax_shape = to_eevee_shape(light_probe.parallax_type);

    float4x4 object_to_world = math::scale(float4x4(ob->object_to_world),
                                           float3(influence_distance));
    cube.location = object_to_world.location();
    cube.volume = math::abs(math::determinant(object_to_world));
    cube.world_to_probe_transposed = float3x4(math::transpose(math::invert(object_to_world)));
    cube.influence_scale = 1.0 / max_ff(1e-8f, influence_falloff);
    cube.influence_bias = cube.influence_scale;
    cube.parallax_distance = parallax_distance / influence_distance;
    cube.clipping_distances = float2(light_probe.clipsta, light_probe.clipend);

    cube.viewport_display = light_probe.flag & LIGHTPROBE_FLAG_SHOW_DATA;
    cube.viewport_display_size = light_probe.data_display_size;
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
