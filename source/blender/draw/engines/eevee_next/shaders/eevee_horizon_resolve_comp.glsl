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

vec3 load_normal(ivec2 texel)
{
  return gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_normal_tx, texel).surface_N;
}

void main()
{
  ivec2 texel_fullres = ivec2(gl_GlobalInvocationID.xy);
  ivec2 texel = texel_fullres / uniform_buf.raytrace.resolution_scale;

  ivec2 extent = textureSize(gbuf_header_tx, 0).xy;
  if (any(greaterThanEqual(texel_fullres, extent))) {
    return;
  }

  vec2 center_uv = (vec2(texel_fullres) + 0.5) * uniform_buf.raytrace.full_resolution_inv;
  float center_depth = texelFetch(depth_tx, texel_fullres, 0).r;
  vec3 center_P = drw_point_screen_to_world(vec3(center_uv, center_depth));

  if (center_depth == 1.0) {
    /* Do not trace for background */
    return;
  }

  GBufferReader gbuf = gbuffer_read(
      gbuf_header_tx, gbuf_closure_tx, gbuf_normal_tx, texel_fullres);

  if (gbuf.header == 0u) {
    return;
  }

  vec3 center_N = gbuf.surface_N;

  /* TODO(fclem): Bilateral weighting. */
  SphericalHarmonicL1 accum_sh;
  accum_sh.L0.M0 = texture(horizon_radiance_0_tx, center_uv);
  accum_sh.L1.Mn1 = texture(horizon_radiance_1_tx, center_uv);
  accum_sh.L1.M0 = texture(horizon_radiance_2_tx, center_uv);
  accum_sh.L1.Mp1 = texture(horizon_radiance_3_tx, center_uv);

  vec3 P = center_P;
  vec3 Ng = center_N;
  vec3 V = drw_world_incident_vector(P);

  LightProbeSample samp = lightprobe_load(P, Ng, V);

  for (int i = 0; i < GBUFFER_LAYER_MAX && i < gbuf.closure_count; i++) {
    ClosureUndetermined cl = gbuffer_closure_get(gbuf, i);

    float roughness = closure_apparent_roughness_get(cl);

    float mix_fac = saturate(roughness * uniform_buf.raytrace.roughness_mask_scale -
                             uniform_buf.raytrace.roughness_mask_bias);
    bool use_raytrace = mix_fac < 1.0;
    bool use_horizon = mix_fac > 0.0;

    if (!use_horizon) {
      continue;
    }

    vec3 N = cl.N;
    vec3 vN = drw_normal_world_to_view(cl.N);

    /* Evaluate lighting from horizon scan. */
    /* TODO(fclem): Evaluate depending on BSDF. */
    vec3 radiance = spherical_harmonics_evaluate_lambert(vN, accum_sh);

    /* Evaluate visibility from horizon scan. */
    SphericalHarmonicL1 sh_visibility = spherical_harmonics_swizzle_wwww(accum_sh);
    float occlusion = spherical_harmonics_evaluate_lambert(vN, sh_visibility).x;
    /* FIXME(fclem): Tried to match the old occlusion look. I don't know why it's needed. */
    occlusion *= 0.5;
    /* TODO(fclem): Ideally, we should just combine both local and distant irradiance and evaluate
     * once. Unfortunately, I couldn't find a way to do the same (1.0 - occlusion) with the
     * spherical harmonic coefficients. */
    float visibility = saturate(1.0 - occlusion);

    /* Apply missing distant lighting. */
    vec3 radiance_probe = spherical_harmonics_evaluate_lambert(N, samp.volume_irradiance);
    radiance += visibility * radiance_probe;

    vec4 radiance_horizon = vec4(radiance, 0.0);
    vec4 radiance_raytrace = use_raytrace ? imageLoad(closure0_img, texel_fullres) : vec4(0.0);

    vec4 radiance_mixed = mix(radiance_raytrace, radiance_horizon, mix_fac);

    /* TODO(fclem): Layered texture. */
    int layer_index = gbuffer_closure_get_bin_index(gbuf, i);
    if (layer_index == 0) {
      imageStore(closure0_img, texel_fullres, radiance_mixed);
    }
    else if (layer_index == 1) {
      imageStore(closure1_img, texel_fullres, radiance_mixed);
    }
    else if (layer_index == 2) {
      imageStore(closure2_img, texel_fullres, radiance_mixed);
    }
  }
}
