/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Sum all spherical harmonic coefficients extracting during remapping to octahedral map.
 * Dispatch only one thread-group that sums. */

#pragma BLENDER_REQUIRE(eevee_reflection_probe_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_reflection_probe_mapping_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)

shared vec3 local_radiance[gl_WorkGroupSize.x];
shared vec3 local_direction[gl_WorkGroupSize.x];
shared float local_solid_angle[gl_WorkGroupSize.x];

void main()
{
  SphereProbeSunLight sun;
  sun.radiance = vec3(0.0);
  sun.direction = vec3(0.0);
  sun.solid_angle = 0.0;

  /* First sum onto the local memory. */
  uint valid_data_len = probe_remap_dispatch_size.x * probe_remap_dispatch_size.y;
  const uint iter_count = uint(SPHERE_PROBE_MAX_HARMONIC) / gl_WorkGroupSize.x;
  for (uint i = 0; i < iter_count; i++) {
    uint index = gl_WorkGroupSize.x * i + gl_LocalInvocationIndex;
    if (index >= valid_data_len) {
      break;
    }
    sun.radiance += in_sun[index].radiance;
    sun.direction += in_sun[index].direction;
    sun.solid_angle += in_sun[index].solid_angle;
  }

  /* Then sum across invocations. */
  const uint local_index = gl_LocalInvocationIndex;
  local_radiance[local_index] = sun.radiance;
  local_direction[local_index] = sun.direction;
  local_solid_angle[local_index] = sun.solid_angle;

  /* Parallel sum. */
  const uint group_size = gl_WorkGroupSize.x * gl_WorkGroupSize.y;
  for (uint stride = group_size / 2; stride > 0; stride /= 2) {
    barrier();
    if (local_index < stride) {
      local_radiance[local_index] += local_radiance[local_index + stride];
      local_direction[local_index] += local_direction[local_index + stride];
      local_solid_angle[local_index] += local_solid_angle[local_index + stride];
    }
  }

  barrier();
  if (gl_LocalInvocationIndex == 0u) {
    out_sun.radiance = local_radiance[0];
    out_sun.direction = normalize(local_direction[0]);
    out_sun.solid_angle = local_solid_angle[0];
  }
}
