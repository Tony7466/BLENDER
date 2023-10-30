/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_horizon_scan_eval_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_lightprobe_eval_lib.glsl)

void main()
{
  const uint tile_size = RAYTRACE_GROUP_SIZE;
  uvec2 tile_coord = unpackUvec2x16(tiles_coord_buf[gl_WorkGroupID.x]);
  ivec2 texel = ivec2(gl_LocalInvocationID.xy + tile_coord * tile_size);

  ivec2 extent = textureSize(gbuf_header_tx, 0).xy;
  if (any(greaterThanEqual(texel, extent))) {
    return;
  }

  vec2 uv = (vec2(texel) + 0.5) * uniform_buf.raytrace.full_resolution_inv;
  float depth = texelFetch(hiz_tx, texel, 0).r;

  if (depth == 1.0) {
    /* Do not trace for background */
    imageStore(out_radiance_img, texel, vec4(0.0));
    return;
  }

  GBufferData gbuf = gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_color_tx, texel);

  HorizonScanContext ctx;
#ifdef HORIZON_DIFFUSE
  if (gbuf.has_diffuse == false) {
    return;
  }
  vec3 Ng = gbuf.diffuse.N;
  ctx.diffuse = gbuf.diffuse;
  ctx.diffuse.N = drw_normal_world_to_view(ctx.diffuse.N);
#endif
#ifdef HORIZON_REFLECT
  if (gbuf.has_reflection == false) {
    return;
  }
  vec3 Ng = gbuf.reflection.N;
  ctx.reflection = gbuf.reflection;
  ctx.reflection.roughness = max(ctx.reflection.roughness, 1e-4);
  ctx.reflection.N = drw_normal_world_to_view(ctx.reflection.N);
#endif
#ifdef HORIZON_REFRACT
  if (gbuf.has_refraction == false) {
    return;
  }
  vec3 Ng = gbuf.refraction.N;
  ctx.refraction = gbuf.refraction;
  ctx.refraction.N = drw_normal_world_to_view(ctx.refraction.N);
#endif

  vec3 vP = drw_point_screen_to_view(vec3(uv, depth));
  vec3 P = drw_point_view_to_world(vP);
  vec3 V = drw_world_incident_vector(P);

  LightProbeSample samp = lightprobe_load(P, Ng, V);
  ctx.irradiance_distant = samp.volume_irradiance;

  vec2 noise = utility_tx_fetch(utility_tx, vec2(texel), UTIL_BLUE_NOISE_LAYER).rg;
  noise = fract(noise + sampling_rng_2D_get(SAMPLING_AO_U));

  horizon_scan_eval(vP,
                    ctx,
                    hiz_tx,
                    noise,
                    uniform_buf.ao.pixel_size,
                    1.0e16,
                    uniform_buf.ao.thickness,
                    uniform_buf.ao.angle_bias,
                    8);

  /* TODO(fclem): Do not load non traced tiles. */
  vec4 radiance_raytrace = imageLoad(out_radiance_img, texel);
  vec4 radiance_horizon = vec4(1.0, 0.0, 1.0, 1.0);
#ifdef HORIZON_DIFFUSE
  radiance_horizon = vec4(ctx.diffuse_result, 1.0);
  float mix_fac = 1.0;
#endif
#ifdef HORIZON_REFLECT
  radiance_horizon = vec4(ctx.reflection_result, 1.0);
  float mix_fac = saturate(ctx.reflection.roughness * 8.0 - 2.0);
#endif
#ifdef HORIZON_REFRACT
  radiance_horizon = vec4(ctx.refraction_result, 1.0);
  float mix_fac = saturate(ctx.refraction.roughness * 8.0 - 2.0);
#endif
  vec4 radiance = mix(radiance_raytrace, radiance_horizon, mix_fac);

  imageStore(out_radiance_img, texel, radiance);
}
