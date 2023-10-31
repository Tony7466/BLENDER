/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_horizon_scan_eval_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)

void main()
{
  const uint tile_size = RAYTRACE_GROUP_SIZE;
  uvec2 tile_coord = unpackUvec2x16(tiles_coord_buf[gl_WorkGroupID.x]);
  ivec2 texel = ivec2(gl_LocalInvocationID.xy + tile_coord * tile_size);

  ivec2 texel_fullres = texel * uniform_buf.raytrace.resolution_scale +
                        uniform_buf.raytrace.resolution_bias;

  ivec2 extent = textureSize(gbuf_header_tx, 0).xy;
  if (any(greaterThanEqual(texel_fullres, extent))) {
    return;
  }

  vec2 uv = (vec2(texel_fullres) + 0.5) * uniform_buf.raytrace.full_resolution_inv;
  float depth = texelFetch(hiz_tx, texel_fullres, 0).r;

  if (depth == 1.0) {
    /* Do not trace for background */
    imageStore(horizon_radiance_img, texel, vec4(FLT_11_11_10_MAX, 0.0));
    return;
  }

  GBufferData gbuf = gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_color_tx, texel_fullres);

  HorizonScanContext ctx;
#ifdef HORIZON_DIFFUSE
  if (gbuf.has_diffuse == false) {
    imageStore(horizon_radiance_img, texel, vec4(0.0));
    return;
  }
  vec3 Ng = gbuf.diffuse.N;
  ctx.diffuse = gbuf.diffuse;
  ctx.diffuse.N = drw_normal_world_to_view(ctx.diffuse.N);
#endif
#ifdef HORIZON_REFLECT
  if (gbuf.has_reflection == false) {
    imageStore(horizon_radiance_img, texel, vec4(0.0));
    return;
  }
  vec3 Ng = gbuf.reflection.N;
  ctx.reflection = gbuf.reflection;
  ctx.reflection.roughness = max(ctx.reflection.roughness, 1e-4);
  ctx.reflection.N = drw_normal_world_to_view(ctx.reflection.N);
#endif
#ifdef HORIZON_REFRACT
  if (gbuf.has_refraction == false) {
    imageStore(horizon_radiance_img, texel, vec4(0.0));
    return;
  }
  vec3 Ng = gbuf.refraction.N;
  ctx.refraction = gbuf.refraction;
  ctx.refraction.N = drw_normal_world_to_view(ctx.refraction.N);
#endif

  vec3 vP = drw_point_screen_to_view(vec3(uv, depth));

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

  float occlusion = 0.0;
  vec4 radiance = vec4(1.0, 0.0, 1.0, 1.0);
#ifdef HORIZON_DIFFUSE
  radiance = vec4(ctx.diffuse_result, 1.0);
  occlusion = horizon_scan_bitmask_to_visibility_uniform(~ctx.diffuse_common.bitmask);
#endif
#ifdef HORIZON_REFLECT
  radiance = vec4(ctx.reflection_result, 1.0);
  occlusion = horizon_scan_bitmask_to_visibility_uniform(~ctx.reflection_common.bitmask);
#endif
#ifdef HORIZON_REFRACT
  radiance = vec4(ctx.refraction_result, 1.0);
  occlusion = horizon_scan_bitmask_to_visibility_uniform(~ctx.refraction_common.bitmask);
#endif

  imageStore(horizon_radiance_img, texel, radiance);
  imageStore(horizon_occlusion_img, texel, vec4(occlusion));
}
