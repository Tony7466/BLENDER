/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)

float bilateral_depth_weight(vec3 center_N, vec3 center_P, vec3 sample_P)
{
  vec4 center_plane_eq = vec4(center_N, -dot(center_N, center_P));
  /* Only compare distance to the center plane formed by the normal. */
  float depth_delta = dot(center_plane_eq, vec4(sample_P, 1.0));
  /* TODO(fclem): Scene parameter. This is dependent on scene scale. */
  const float scale = 100.0;
  float weight = exp2(-scale * square(depth_delta));
  return weight;
}

float bilateral_spatial_weight(float sigma, vec2 offset_from_center)
{
  /* From https://github.com/tranvansang/bilateral-filter/blob/master/fshader.frag */
  float fac = -1.0 / square(sigma);
  /* Take two standard deviation. */
  fac *= 2.0;
  float weight = exp2(fac * length_squared(offset_from_center));
  return weight;
}

float bilateral_normal_weight(vec3 center_N, vec3 sample_N)
{
  float facing_ratio = dot(center_N, sample_N);
  float weight = saturate(pow8f(facing_ratio));
  return weight;
}

vec3 sample_normal_get(ivec2 texel, out bool is_processed)
{
  vec4 normal = texelFetch(screen_normal_tx, texel, 0);
  is_processed = (normal.w != 0.0);
  return normal.xyz * 2.0 - 1.0;
}

float sample_weight_get(
    vec3 center_N, vec3 center_P, ivec2 sample_texel, vec2 sample_uv, ivec2 sample_offset)
{
  ivec2 sample_texel_fullres = sample_texel * uniform_buf.raytrace.horizon_resolution_scale +
                               uniform_buf.raytrace.horizon_resolution_bias;
  float sample_depth = texelFetch(hiz_tx, sample_texel_fullres, 0).r;

  bool is_valid;
  vec3 sample_N = sample_normal_get(sample_texel, is_valid);
  vec3 sample_P = drw_point_screen_to_world(vec3(sample_uv, sample_depth));

  if (!is_valid) {
    return 0.0;
  }

  float depth_weight = bilateral_depth_weight(center_N, center_P, sample_P);
  float spatial_weight = bilateral_spatial_weight(1.5, vec2(sample_offset));
  float normal_weight = bilateral_normal_weight(center_N, sample_N);

  return depth_weight * spatial_weight * normal_weight;
}

void main()
{
  const uint tile_size = RAYTRACE_GROUP_SIZE;
  uvec2 tile_coord = unpackUvec2x16(tiles_coord_buf[gl_WorkGroupID.x]);
  ivec2 texel = ivec2(gl_LocalInvocationID.xy + tile_coord * tile_size);

  vec2 texel_size = 1.0 / vec2(textureSize(in_sh_0_tx, 0).xy);
  ivec2 texel_fullres = texel * uniform_buf.raytrace.horizon_resolution_scale +
                        uniform_buf.raytrace.horizon_resolution_bias;

  bool is_valid;
  float center_depth = texelFetch(hiz_tx, texel_fullres, 0).r;
  vec2 center_uv = vec2(texel) * texel_size;
  vec3 center_P = drw_point_screen_to_world(vec3(center_uv, center_depth));
  vec3 center_N = sample_normal_get(texel, is_valid);

  if (!is_valid) {
#if 0 /* This is not needed as the next stage doesn't do bilinear filtering. */
    imageStore(out_sh_0_img, texel, vec4(0.0));
    imageStore(out_sh_1_img, texel, vec4(0.0));
    imageStore(out_sh_2_img, texel, vec4(0.0));
    imageStore(out_sh_3_img, texel, vec4(0.0));
#endif
    return;
  }

  SphericalHarmonicL1 accum_sh;
  /* Center texel. */
  accum_sh.L0.M0 = texelFetch(in_sh_0_tx, texel, 0);
  accum_sh.L1.Mn1 = texelFetch(in_sh_1_tx, texel, 0);
  accum_sh.L1.M0 = texelFetch(in_sh_2_tx, texel, 0);
  accum_sh.L1.Mp1 = texelFetch(in_sh_3_tx, texel, 0);
  float accum_weight = 1.0;

  /* Sample around the center texel, sampling in pairs of adjacent texels. */
  ivec2 sample_offset = ivec2(1, 0);
  for (int i = 0; i < 4; i++) {
    sample_offset = orthogonal(sample_offset);
    ivec2 sample_offset_0 = sample_offset;
    ivec2 sample_offset_1 = sample_offset + orthogonal(sample_offset);
    ivec2 sample_texel_0 = texel + sample_offset_0;
    ivec2 sample_texel_1 = texel + sample_offset_1;
    vec2 sample_uv_0 = (vec2(sample_texel_0) + 0.5) * texel_size;
    vec2 sample_uv_1 = (vec2(sample_texel_1) + 0.5) * texel_size;

    /* Compute weight for this sample pair. */
    float sample_weight_0 = sample_weight_get(
        center_N, center_P, sample_texel_0, sample_uv_0, sample_offset_0);
    float sample_weight_1 = sample_weight_get(
        center_N, center_P, sample_texel_1, sample_uv_1, sample_offset_1);
    float sample_weight_combined = sample_weight_0 + sample_weight_1;
    float sample_mix = sample_weight_1 * safe_rcp(sample_weight_combined);

    /* Custom "linear" fetch to reduce texture fetches by two. */
    vec2 sample_uv = mix(sample_uv_0, sample_uv_1, sample_mix);

    SphericalHarmonicL1 sample_sh;
    sample_sh.L0.M0 = texture(in_sh_0_tx, sample_uv);
    sample_sh.L1.Mn1 = texture(in_sh_1_tx, sample_uv);
    sample_sh.L1.M0 = texture(in_sh_2_tx, sample_uv);
    sample_sh.L1.Mp1 = texture(in_sh_3_tx, sample_uv);

    accum_sh = spherical_harmonics_madd(sample_sh, sample_weight_combined, accum_sh);
    accum_weight += sample_weight_combined;
  }
  accum_sh = spherical_harmonics_mul(accum_sh, safe_rcp(accum_weight));

  imageStore(out_sh_0_img, texel, accum_sh.L0.M0);
  imageStore(out_sh_1_img, texel, accum_sh.L1.Mn1);
  imageStore(out_sh_2_img, texel, accum_sh.L1.M0);
  imageStore(out_sh_3_img, texel, accum_sh.L1.Mp1);
}
