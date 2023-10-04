/* SPDX-FileCopyrightText: 2022-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * The resources expected to be defined are:
 * - light_buf
 * - light_zbin_buf
 * - light_cull_buf
 * - light_tile_buf
 * - shadow_atlas_tx
 * - shadow_tilemaps_tx
 * - utility_tx
 */

#pragma BLENDER_REQUIRE(eevee_shadow_tracing_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_light_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_shadow_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)

#define LIGHT_VISIBILITY_THRESHOLD 1e-6

/* If using compute, the shader should define it's own pixel. */
#if !defined(PIXEL) && defined(GPU_FRAGMENT_SHADER)
#  define PIXEL gl_FragCoord.xy
#endif

uint shadow_pack(float visibility, uint bit_depth, uint shift)
{
  return uint(visibility * float((1u << bit_depth) - 1u)) << shift;
}

float shadow_unpack(uint shadow_bits, uint bit_depth, uint shift)
{
  return float((shadow_bits >> shift) & ~(~0u << bit_depth)) / float((1u << bit_depth) - 1u);
}

void shadow_mask(vec3 P, vec3 Ng, float vPz, float thickness, out uint shadow_bits)
{
  shadow_bits = 0u;
  uint shift = 0u;
  uint ray_count = uniform_buf.shadow.ray_count;
  uint ray_step_count = uniform_buf.shadow.step_count;

  LIGHT_FOREACH_BEGIN_DIRECTIONAL (light_cull_buf, l_idx) {
    LightData light = light_buf[l_idx];

    ShadowEvalResult result = shadow_eval(light, true, P, Ng, ray_count, ray_step_count);

    shadow_bits |= shadow_pack(result.surface_light_visibilty, ray_count, shift);
    shift += ray_count;
  }
  LIGHT_FOREACH_END

  LIGHT_FOREACH_BEGIN_LOCAL (light_cull_buf, light_zbin_buf, light_tile_buf, PIXEL, vPz, l_idx) {
    LightData light = light_buf[l_idx];

    vec3 L;
    float dist;
    light_vector_get(light, P, L, dist);

    if (light_attenuation(light, L, dist) < LIGHT_VISIBILITY_THRESHOLD) {
      return;
    }

    ShadowEvalResult result = shadow_eval(light, false, P, Ng, ray_count, ray_step_count);

    shadow_bits |= shadow_pack(result.surface_light_visibilty, ray_count, shift);
    shift += ray_count;
  }
  LIGHT_FOREACH_END
}

struct ClosureLight {
  /* Shading normal. */
  vec3 N;
  /* LTC matrix. */
  vec4 ltc_mat;
  /* Enum (used as index) telling how to treat the lighting. */
  LightingType type;
  /* Output both shadowed and unshadowed for shadow denoising. */
  vec3 light_shadowed;
  vec3 light_unshadowed;
};

void light_closure_eval(LightData light,
                        vec3 L,
                        float dist,
                        inout ClosureLight cl,
                        float thickness,
                        float visibility,
                        vec3 P,
                        vec3 V)
{
  if (light.power[cl.type] > 0.0) {
    float ltc_result = light_ltc(utility_tx, light, cl.N, V, L, dist, cl.ltc_mat);
    vec3 out_radiance = light.color * light.power[cl.type] * ltc_result;
    cl.light_shadowed += visibility * out_radiance;
    cl.light_unshadowed += out_radiance;
  }
}

void light_eval(ClosureLight cl1,
#if LIGHT_CLOSURE_EVAL_COUNT > 1
                ClosureLight cl2,
#endif
#if LIGHT_CLOSURE_EVAL_COUNT > 2
                ClosureLight cl3,
#endif
#ifdef SHADOW_DEFERRED
                uint shadow_bits,
#endif
                vec3 P,
                vec3 Ng,
                vec3 V,
                float vPz,
                float thickness)
{
  cl1.light_shadowed = cl1.light_unshadowed = vec3(0.0);
#if LIGHT_CLOSURE_EVAL_COUNT > 1
  cl2.light_shadowed = cl2.light_unshadowed = vec3(0.0);
#endif
#if LIGHT_CLOSURE_EVAL_COUNT > 2
  cl3.light_shadowed = cl3.light_unshadowed = vec3(0.0);
#endif

  uint ray_count = uniform_buf.shadow.ray_count;
  uint ray_step_count = uniform_buf.shadow.step_count;
#ifdef SHADOW_DEFERRED
  uint shift = 0u;
#  define shadow_function(light, is_directional, P, Ng, ray_count, ray_step_count) \
    shadow_unpack(shadow_bits, ray_count, shift); \
    shift += ray_count
#else
#  define shadow_function(light, is_directional, P, Ng, ray_count, ray_step_count) \
    shadow_eval(light, is_directional, P, Ng, ray_count, ray_step_count).surface_light_visibilty;
#endif

  LIGHT_FOREACH_BEGIN_DIRECTIONAL (light_cull_buf, l_idx) {
    LightData light = light_buf[l_idx];

    vec3 L;
    float dist;
    light_vector_get(light, P, L, dist);

    float visibility = shadow_function(light, true, P, Ng, ray_count, ray_step_count);

    light_closure_eval(light, L, dist, cl1, thickness, visibility, P, V);
#if LIGHT_CLOSURE_EVAL_COUNT > 1
    light_closure_eval(light, L, dist, cl2, thickness, visibility, P, V);
#endif
#if LIGHT_CLOSURE_EVAL_COUNT > 2
    light_closure_eval(light, L, dist, cl3, thickness, visibility, P, V);
#endif
  }
  LIGHT_FOREACH_END

  LIGHT_FOREACH_BEGIN_LOCAL (light_cull_buf, light_zbin_buf, light_tile_buf, PIXEL, vPz, l_idx) {
    LightData light = light_buf[l_idx];

    vec3 L;
    float dist;
    light_vector_get(light, P, L, dist);

    float visibility = light_attenuation(light, L, dist);
    if (visibility < LIGHT_VISIBILITY_THRESHOLD) {
      return;
    }

    visibility *= shadow_function(light, false, P, Ng, ray_count, ray_step_count);

    light_closure_eval(light, L, dist, cl1, thickness, visibility, P, V);
#if LIGHT_CLOSURE_EVAL_COUNT > 1
    light_closure_eval(light, L, dist, cl2, thickness, visibility, P, V);
#endif
#if LIGHT_CLOSURE_EVAL_COUNT > 2
    light_closure_eval(light, L, dist, cl3, thickness, visibility, P, V);
#endif
  }
  LIGHT_FOREACH_END

#undef shadow_function
}
