/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Based on Frosbite Unified Volumetric.
 * https://www.ea.com/frostbite/news/physically-based-unified-volumetric-rendering-in-frostbite */

/* Store volumetric properties into the froxel textures. */

#pragma BLENDER_REQUIRE(eevee_volume_lib.glsl)

/* Needed includes for shader nodes. */
#pragma BLENDER_REQUIRE(common_attribute_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_attributes_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_nodetree_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_occupancy_lib.glsl)

GlobalData init_globals(vec3 wP)
{
  GlobalData surf;
  surf.P = wP;
  surf.N = vec3(0.0);
  surf.Ng = vec3(0.0);
  surf.is_strand = false;
  surf.hair_time = 0.0;
  surf.hair_thickness = 0.0;
  surf.hair_strand_id = 0;
  surf.barycentric_coords = vec2(0.0);
  surf.barycentric_dists = vec3(0.0);
  surf.ray_type = RAY_TYPE_CAMERA;
  surf.ray_depth = 0.0;
  surf.ray_length = distance(surf.P, drw_view_position());
  return surf;
}

struct VolumeProperties {
  vec3 scattering;
  vec3 absorption;
  vec3 emission;
  float anisotropy;
};

VolumeProperties eval_froxel(ivec3 froxel);
{
  /* TODO(fclem): Make sure this volume to screen is well aligned with the view. */
  vec3 ndc_cell = volume_to_screen(froxel * uniform_buf.volumes.inv_tex_size);

  vec3 vP = get_view_space_from_depth(ndc_cell.xy, ndc_cell.z);
  vec3 wP = point_view_to_world(vP);
#ifdef MAT_GEOM_VOLUME_OBJECT
  g_lP = point_world_to_object(wP);
  g_orco = OrcoTexCoFactors[0].xyz + g_lP * OrcoTexCoFactors[1].xyz;
#else /* WORLD_SHADER */
  g_orco = wP;
#endif

  g_data = init_globals(wP);
  attrib_load();
  nodetree_volume();

#ifdef MAT_GEOM_VOLUME_OBJECT
  g_volume_scattering *= drw_volume.density_scale;
  g_volume_absorption *= drw_volume.density_scale;
  g_emission *= drw_volume.density_scale;
#endif

  VolumeProperties prop;
  prop.scattering = g_volume_scattering;
  prop.absorption = g_volume_absorption;
  prop.emission = g_emission;
  prop.anisotropy = g_volume_anisotropy;
  return prop;
}

void write_froxel(ivec3 froxel, VolumeProperties prop)
{
  vec2 phase = vec2(prop.anisotropy, 1.0);

  /* Do not add phase weight if there's no scattering. */
  if (all(equal(prop.scattering, vec3(0.0)))) {
    phase = vec2(0.0);
  }

  vec3 extinction = scattering + absorption;

#ifdef MAT_GEOM_VOLUME_OBJECT
  /* Additive Blending. No race condition since we have a barrier between each conflicting
   * invocations. */
  prop.scattering += imageLoad(out_scattering_img, froxel).rgb;
  prop.emission += imageLoad(out_emissive_img, froxel).rgb;
  extinction += imageLoad(out_extinction_img, froxel).rgb;
  phase += imageLoad(out_phase_img, froxel).rg;
#endif

  imageStore(out_scattering_img, froxel, prop.scattering.xyzz);
  imageStore(out_extinction_img, froxel, extinction.xyzz);
  imageStore(out_emissive_img, froxel, prop.emission.xyzz);
  imageStore(out_phase_img, froxel, phase.xyyy);
}

void main()
{
  ivec3 froxel = ivec3(gl_FragCoord.xy, 0);

#ifdef VOLUME_HOMOGENOUS
  /* Homogenous volumes only evaluate properties at volume entrance and write the same values for
   * each froxel. */
  VolumeProperties prop = eval_froxel(froxel);
#endif

#ifdef MAT_GEOM_VOLUME_OBJECT
  /* Check all occupancy bits. */
  for (int word_idx = 0; word_idx < MAX_OCCUPANCY_WORD; word_idx++) {
    if (word_idx >= occupancy_word_len) {
      break;
    }
    uint occupancy_word = imageLoad(occupancy_img, ivec3(froxel.xy, word_idx)).r;
    while ((bit_idx = findLSB(occupancy_word)) != -1) {
      occupancy_word &= ~1u << uint(bit_idx);
      froxel.z = word_idx * 32u + bit_idx;
#else
  /* World iterate over all froxel. */
  int froxel_len = imageSize(out_scattering_img).z;
  for (int i = 0; (i < froxel_len) && (i < MAX_OCCUPANCY_WORD * 32); i++) {
    {
      froxel.z = i;
#endif

      if (froxel_occupied(froxel)) {
#ifndef VOLUME_HOMOGENOUS
        /* Heterogenous volumes evaluate properties at every froxel position. */
        VolumeProperties prop = eval_froxel(froxel);
#endif
        write_froxel(froxel, prop);
      }
    }
  }
}
