/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_octahedron_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)

#ifdef SPHERE_PROBE

/* Apply correct bias and scale for the given level of detail. */
vec2 reflection_probes_miplvl_scale_bias(float mip_lvl, SphereProbeUvArea uv_area, vec2 uv)
{
  float pixel_count_mip_0 = textureSize(reflection_probes_tx, 0).x * uv_area.scale;
  float pixel_count = floor(pixel_count_mip_0 / exp2(mip_lvl));
  float scale = (pixel_count - 1.0) / pixel_count;
  float offset = 0.5 / pixel_count;
  return uv * scale + offset;
}

vec4 reflection_probes_sample(vec3 L, float lod, SphereProbeUvArea uv_area)
{
  float lod_min = floor(lod);
  float lod_max = ceil(lod);
  float mix_fac = lod - lod_min;

  /* We place texel centers at the edges of the octahedron, to avoid artifacts caused by
   * interpolating across the edges.
   *
   * This is a diagram of a 5px^2 map with 2 mips. 0 and 1 denote the pixels centers of each
   * mips. The lines shows the octahedron edges.
   *
   * 1---0---1---0---1 <
   * |     / | \     |  |
   * 0   0   0   0   0  |
   * | /     |     \ |  |
   * 1---0---1---0---1  | sampling area
   * | \     |     / |  |
   * 0   0   0   0   0  |
   * |     \ | /     |  |
   * 1---0---1---0---1 <
   * ^---------------^
   *       sampling area
   *
   * To do this, we made all mip levels have an odd number of pixel, by sizing the atlas
   * appropriately. But this makes the interpolation between mips not aligning and we need to use a
   * different uv coordinate for each level of detail. Fortunately, this avoids us to have to worry
   * about padding texels all together.
   */
  vec2 octahedral_uv = octahedral_uv_from_direction(L);
  vec2 local_uv_min = reflection_probes_miplvl_scale_bias(lod_min, uv_area, octahedral_uv);
  vec2 local_uv_max = reflection_probes_miplvl_scale_bias(lod_max, uv_area, octahedral_uv);
  /* Remap into atlas location. */
  vec2 altas_uv_min = local_uv_min * uv_area.scale + uv_area.offset;
  vec2 altas_uv_max = local_uv_max * uv_area.scale + uv_area.offset;
  vec4 color_min = textureLod(reflection_probes_tx, vec3(altas_uv_min, uv_area.layer), lod_min);
  vec4 color_max = textureLod(reflection_probes_tx, vec3(altas_uv_max, uv_area.layer), lod_max);
  return mix(color_min, color_max, mix_fac);
}
#endif

ReflectionProbeLowFreqLight reflection_probes_extract_low_freq(SphericalHarmonicL1 sh)
{
  /* To avoid color shift and negative values, we reduce saturation and directionality. */
  ReflectionProbeLowFreqLight result;
  result.ambient = sh.L0.M0.r + sh.L0.M0.g + sh.L0.M0.b;

  mat3x4 L1_per_band;
  L1_per_band[0] = sh.L1.Mn1;
  L1_per_band[1] = sh.L1.M0;
  L1_per_band[2] = sh.L1.Mp1;

  mat4x3 L1_per_comp = transpose(L1_per_band);
  result.direction = L1_per_comp[0] + L1_per_comp[1] + L1_per_comp[2];

  return result;
}

vec3 reflection_probes_normalization_eval(vec3 L,
                                          ReflectionProbeLowFreqLight numerator,
                                          ReflectionProbeLowFreqLight denominator)
{
  /* TODO(fclem): Adjusting directionality is tricky.
   * Needs to be revisited later on. For now only use the ambient term. */
  return vec3(numerator.ambient * safe_rcp(denominator.ambient));
}
