/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_lightprobe_eval_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_closure_lib.glsl)

float bilateral_depth_weight(vec3 center_N, vec3 center_P, vec3 sample_P)
{
  vec4 center_plane_eq = vec4(center_N, -dot(center_N, center_P));
  /* Only compare distance to the center plane formed by the normal. */
  float depth_delta = dot(center_plane_eq, vec4(sample_P, 1.0));
  /* TODO(fclem): Scene parameter. This is dependent on scene scale. */
  const float scale = 10000.0;
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

/* In order to remove some more fireflies, "tone-map" the color samples during the accumulation. */
vec3 to_accumulation_space(vec3 color)
{
  /* This 4 factor is to avoid killing too much energy. */
  /* TODO(fclem): Parameter? */
  color /= 4.0;
  color = color / (1.0 + reduce_add(color));
  return color;
}
vec3 from_accumulation_space(vec3 color)
{
  color = color / (1.0 - reduce_add(color));
  color *= 4.0;
  return color;
}

vec3 load_normal(ivec2 texel)
{
  return gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_normal_tx, texel).surface_N;
}

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);

  vec3 center_P = drw_point_screen_to_world(vec3(center_uv, center_depth));
  vec3 center_N = gbuf.surface_N;

  SphericalHarmonicL1 accum_sh = spherical_harmonics_L1_new();

  ivec2 sample_offset = ivec2(1, 0);
  int sample_i = 0;
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++, sample_i++) {
      sample_offset = orthogonal(sample_offset);
      ivec2 sample_offset_pair = sample_offset + orthogonal(sample_offset);

      /* Compute weight for this sample pair. */
      float sample_weight_0 = ;
      float sample_weight_1 = ;
      float sample_mix = sample_weight_1 * safe_rcp(sample_weight_0 + sample_weight_1);

      /* Custom "linear" fetch to reduce texture fetches. */
      vec2 sample_uv = mix(sample_uv_0, sample_uv_1, sample_mix);
      SphericalHarmonicL1 sample_sh;
      sample_sh.L0.M0 = texelFetch(horizon_radiance_0_tx, sample_texel, 0);
      sample_sh.L1.Mn1 = texelFetch(horizon_radiance_1_tx, sample_texel, 0) * 2.0 - 1.0;
      sample_sh.L1.M0 = texelFetch(horizon_radiance_2_tx, sample_texel, 0) * 2.0 - 1.0;
      sample_sh.L1.Mp1 = texelFetch(horizon_radiance_3_tx, sample_texel, 0) * 2.0 - 1.0;
    }
  }

  accum_sh = spherical_harmonics_mul(accum_sh, safe_rcp(accum_weight));
}
