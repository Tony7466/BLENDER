/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Shader to convert cube-map to octahedral projection. */

#pragma BLENDER_REQUIRE(eevee_reflection_probe_mapping_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_colorspace_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)

/* Should be 16K. But could be lowered by doing reducion in more than one step. */
shared vec4 local_sh_coefs[gl_WorkGroupSize.x * gl_WorkGroupSize.y][4];

void spherical_harmonic_lds_store(uint index, SphericalHarmonicL1 sh)
{
  local_sh_coefs[index][0] = sh.L0.M0;
  local_sh_coefs[index][1] = sh.L1.Mn1;
  local_sh_coefs[index][2] = sh.L1.M0;
  local_sh_coefs[index][3] = sh.L1.Mp1;
}

SphericalHarmonicL1 spherical_harmonic_lds_load(uint index)
{
  SphericalHarmonicL1 sh;
  sh.L0.M0 = local_sh_coefs[index][0];
  sh.L1.Mn1 = local_sh_coefs[index][1];
  sh.L1.M0 = local_sh_coefs[index][2];
  sh.L1.Mp1 = local_sh_coefs[index][3];
  return sh;
}

void main()
{
  SphereProbeUvArea world_coord = reinterpret_as_atlas_coord(world_coord_packed);
  SphereProbeUvArea sample_coord = reinterpret_as_atlas_coord(probe_coord_packed);
  SphereProbePixelArea write_coord = reinterpret_as_write_coord(write_coord_packed);

  /* Texel in probe. */
  ivec2 local_texel = ivec2(gl_GlobalInvocationID.xy);

  vec2 wrapped_uv;
  vec3 direction = sphere_probe_texel_to_direction(
      local_texel, write_coord, sample_coord, wrapped_uv);
  vec4 radiance_and_transmittance = texture(cubemap_tx, direction);
  vec3 radiance = radiance_and_transmittance.xyz;

  float opacity = 1.0 - radiance_and_transmittance.a;

  /* Composite world into reflection probes. */
  bool is_world = all(equal(probe_coord_packed, world_coord_packed));
  if (!is_world && opacity != 1.0) {
    vec2 world_uv = wrapped_uv * world_coord.scale + world_coord.offset;
    vec4 world_radiance = textureLod(atlas_tx, vec3(world_uv, world_coord.layer), 0.0);
    radiance.rgb = mix(world_radiance.rgb, radiance.rgb, opacity);
  }

  radiance = colorspace_brightness_clamp_max(radiance, probe_brightness_clamp);

  if (!any(greaterThanEqual(local_texel, ivec2(write_coord.extent)))) {
    ivec3 texel = ivec3(local_texel + write_coord.offset, write_coord.layer);
    imageStore(atlas_img, texel, vec4(radiance, 1.0));
  }

  if (extract_sh) {
    /* TODO(fclem): Do not include the 1px border in the SH processing. */

    /* Convert radiance to spherical harmonics. */
    SphericalHarmonicL1 sh;
    sh.L0.M0 = vec4(0.0);
    sh.L1.Mn1 = vec4(0.0);
    sh.L1.M0 = vec4(0.0);
    sh.L1.Mp1 = vec4(0.0);
    /* TODO(fclem): Texel solid angle. */
    float sample_weight = 1.0;
    /* TODO(fclem): Cleanup: Should spherical_harmonics_encode_signal_sample return a new sh
     * instead of adding to it? */
    spherical_harmonics_encode_signal_sample(direction, vec4(radiance, 1.0) * sample_weight, sh);

    spherical_harmonic_lds_store(gl_LocalInvocationIndex, sh);

    barrier();
    if (gl_LocalInvocationIndex == 0u) {
      /* Join results. */
      for (uint i = 1; i < gl_WorkGroupSize.x * gl_WorkGroupSize.y; i++) {
        sh = spherical_harmonics_add(sh, spherical_harmonic_lds_load(i));
      }
      SphereProbeHarmonic sphere_probe_sh;
      sphere_probe_sh.L0_M0 = sh.L0.M0;
      sphere_probe_sh.L1_Mn1 = sh.L1.Mn1;
      sphere_probe_sh.L1_M0 = sh.L1.M0;
      sphere_probe_sh.L1_Mp1 = sh.L1.Mp1;

      uint work_group_index = gl_NumWorkGroups.x * gl_WorkGroupID.y + gl_WorkGroupID.x;
      out_sh[work_group_index] = sphere_probe_sh;
    }
  }
}
