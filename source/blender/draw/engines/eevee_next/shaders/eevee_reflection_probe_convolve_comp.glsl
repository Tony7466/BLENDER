/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Shader to convert cube-map to octahedral projection. */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_matrix_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_reflection_probe_mapping_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)

float cone_cosine_from_roughness(float linear_roughness)
{
  float m = linear_roughness;
  /* Jimenez 2016 in Practical Realtime Strategies for Accurate Indirect Occlusion. (eq 26) */
  // return exp2(-3.32193 * square(m));
  /* Using the same derivation as above but with u = 0.244 (lobe fitting cone). */
  return exp2(-1.017523474 * square(m));
}

float sample_weight(vec3 out_direction, vec3 in_direction, float linear_roughness)
{
  out_direction = normalize(out_direction);
  in_direction = normalize(in_direction);

  float cos_theta = saturate(dot(out_direction, in_direction));

  /* From linear roughness to GGX roughness input. */
  float m = square(linear_roughness);
  /* Map GGX roughness to spherical gaussian sharpness.
   * From "SG Series Part 4: Specular Lighting From an SG Light Source" by MJP
   * https://therealmjp.github.io/posts/sg-series-part-4-specular-lighting-from-an-sg-light-source/
   */
  vec3 N = out_direction;
  vec3 H = normalize(out_direction + in_direction);
  float NH = saturate(dot(N, H));
  /* GGX. */
  // return exp(-square(acos(NH) / m));
  /* Spherical Gaussian. */
  return exp(2.0 * (NH - 1.0) / square(m));
}

mat3x3 tangent_basis(vec3 N)
{
  /* TODO(fclem): This create a discontinuity at Z=0. */
  return from_up_axis(N);
}

void main()
{
  SphereProbeUvArea sample_coord = reinterpret_as_atlas_coord(probe_coord_packed);
  SphereProbePixelArea out_texel_area = reinterpret_as_write_coord(write_coord_packed);
  SphereProbePixelArea in_texel_area = reinterpret_as_write_coord(read_coord_packed);

  /* Texel in probe. */
  ivec2 out_local_texel = ivec2(gl_GlobalInvocationID.xy);

  /* Exit when pixel being written doesn't fit in the area reserved for the probe. */
  if (any(greaterThanEqual(out_local_texel, ivec2(out_texel_area.extent)))) {
    return;
  }

  /* From mip to linear roughness (same as UI). */
  float mip_roughness = sphere_probe_lod_to_roughness(float(read_lod + 1));
  /* Clamp to avoid numerical imprecision. */
  mip_roughness = max(mip_roughness, BSDF_ROUGHNESS_THRESHOLD);
  float cone_cos = cone_cosine_from_roughness(mip_roughness);

  vec3 out_direction = sphere_probe_texel_to_direction(
      out_local_texel, out_texel_area, sample_coord);
  out_direction = normalize(out_direction);

  mat3x3 basis = tangent_basis(out_direction);

  ivec2 out_texel = out_texel_area.offset + out_local_texel;

  float weight_accum = 0.0;
  vec4 radiance_accum = vec4(0.0);

  int sample_count = 1024;
  for (int i = 0; i < sample_count; i++) {
    vec2 rand = hammersley_2d(i, sample_count);
    vec3 in_direction = basis * sample_uniform_cone(rand, cone_cos);

#if 0
    vec2 in_uv = sphere_probe_direction_to_uv(in_direction, float(read_lod), sample_coord);
    vec4 radiance = texture(in_atlas_mip_tx, vec3(in_uv, sample_coord.layer));
#else /* For reference and debugging.  */
    vec4 radiance = texture(cubemap_tx, in_direction);
#endif

    float weight = sample_weight(out_direction, in_direction, mip_roughness);
    radiance_accum += radiance * weight;
    weight_accum += weight;
  }
  vec4 out_radiance = radiance_accum * safe_rcp(weight_accum);

#if 0 /* Debugging texel alignment. */
  ivec2 a = out_texel % 2;
  out_radiance = vec4(a.x == a.y);
#endif

  imageStore(out_atlas_mip_img, ivec3(out_texel, out_texel_area.layer), out_radiance);
}
