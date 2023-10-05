/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_octahedron_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)

#ifdef REFLECTION_PROBE
vec4 reflection_probes_sample(vec3 L, float lod, ReflectionProbeData probe_data)
{
  vec2 octahedral_uv_packed = octahedral_uv_from_direction(L);
  vec2 texel_size = vec2(1.0 / float(1 << (11 - probe_data.layer_subdivision)));
  vec2 octahedral_uv = octahedral_uv_to_layer_texture_coords(
      octahedral_uv_packed, probe_data, texel_size);
  return textureLod(reflection_probes_tx, vec3(octahedral_uv, probe_data.layer), lod);
}

vec3 reflection_probes_world_sample(vec3 L, float lod)
{
  ReflectionProbeData probe_data = reflection_probe_buf[0];
  return reflection_probes_sample(L, lod, probe_data).rgb;
}
#endif

float3x4 reflection_probes_spherical_harmonic_encode(SphericalHarmonicL1 sh)
{
  /* To avoid color shift, only store the sum of coeficient.
   * Do not use average to avoid another division in the evaluation shader. */
  float3x4 sh_encoded;
  sh_encoded[0] = vec4(sh.L0.M0.rgb, sh.L1.Mp1.r);
  sh_encoded[1] = vec4(sh.L1.Mn1.rgb, sh.L1.Mp1.g);
  sh_encoded[2] = vec4(sh.L1.M0.rgb, sh.L1.Mp1.b);
  return sh_encoded;
}

SphericalHarmonicL1 reflection_probes_spherical_harmonic_decode(float3x4 sh)
{
  SphericalHarmonicL1 sh_decoded;
  sh_decoded.L0.M0 = vec4(sh[0].xyz, 0.0);
  sh_decoded.L1.Mn1 = vec4(sh[1].xyz, 0.0);
  sh_decoded.L1.M0 = vec4(sh[2].xyz, 0.0);
  sh_decoded.L1.Mp1 = vec4(sh[0].w, sh[1].w, sh[2].w, 0.0);
  return sh_decoded;
}
