/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Shader to convert cube-map to octahedral projection. */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_matrix_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_reflection_probe_mapping_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)

float sample_weight(vec3 out_direction, vec3 in_direction)
{
  out_direction = normalize(out_direction);
  in_direction = normalize(in_direction);

  float cos_theta = clamp(dot(out_direction, in_direction), 0.0, 1.0);

  /* From "Moving Frostbite to Physically Based Rendering 3.0" eq 53 (inversed). */
  float mip_ratio = float(read_lod + 1) / float(SPHERE_PROBE_MIPMAP_LEVELS - 1);
  /* Use 0.75 for last mip as we fade towards volume probes for higher roughness. */
  /* From mip ratio to linear roughness (same as UI). */
  float mip_roughness = square(mip_ratio * 0.75);
  /* Clamp to avoid numerical imprecision. */
  mip_roughness = max(mip_roughness, BSDF_ROUGHNESS_THRESHOLD);
  /* From linear roughness to GGX roughness input. */
  float m = square(mip_roughness);
  /* Map GGX roughness to spherical gaussian sharpness.
   * From "SG Series Part 4: Specular Lighting From an SG Light Source" by MJP
   * https://therealmjp.github.io/posts/sg-series-part-4-specular-lighting-from-an-sg-light-source/
   */
  vec3 N = out_direction;
  vec3 H = normalize(out_direction + in_direction);
  float NH = clamp(dot(N, H), 0.0, 1.0);
  /* GGX. */
  // return exp(-square(acos(NH) / m));
  /* Spherical Gaussian. */
  return exp(2.0 * (NH - 1.0) / square(m));
}

void convolve_sample(vec3 in_direction,
                     vec3 out_direction,
                     SphereProbeUvArea sample_coord,
                     inout vec4 radiance_accum,
                     inout float weight_accum)
{
  vec2 in_uv = sphere_probe_direction_to_uv(in_direction, float(read_lod), sample_coord);
  vec2 in_texel = in_uv * float(imageSize(in_atlas_mip_img).x);

  float weight = sample_weight(out_direction, in_direction);
  vec4 radiance = imageLoad(in_atlas_mip_img, ivec3(in_texel, sample_coord.layer));
#if 0 /* For reference and debugging.  */
  vec4 radiance = texture(cubemap_tx, in_direction);
#endif

  radiance_accum += radiance * weight;
  weight_accum += weight;
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

  int out_atlas_mip_size = imageSize(in_atlas_mip_img).x / 2;
  vec3 out_direction = sphere_probe_texel_to_direction(
      out_local_texel, out_texel_area, sample_coord, out_atlas_mip_size);
  out_direction = normalize(out_direction);

  mat3x3 basis = from_up_axis(out_direction);

  ivec2 out_texel = out_texel_area.offset + out_local_texel;

  float weight = 0.0;
  vec4 radiance = vec4(0.0);

  int sample_count = 1024;
  for (int i = 0; i < sample_count; i++) {
    vec2 rand = hammersley_2d(i, sample_count);
    vec3 in_direction = basis * normalize(sample_uniform_cone(rand, 0.9));
    convolve_sample(in_direction, out_direction, sample_coord, radiance, weight);
  }
  radiance *= safe_rcp(weight);

  imageStore(out_atlas_mip_img, ivec3(out_texel, out_texel_area.layer), radiance);
}
