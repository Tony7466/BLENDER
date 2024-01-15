/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Compute light objects lighting contribution using Gbuffer data.
 */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_renderpass_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_light_eval_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_thickness_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_subsurface_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_lightprobe_eval_lib.glsl)

ClosureLight closure_light_new(ClosureUndetermined cl, vec3 V)
{
  ClosureLight cl_light;
  cl_light.N = cl.N;
  cl_light.ltc_mat = LTC_LAMBERT_MAT;
  cl_light.type = LIGHT_DIFFUSE;
  cl_light.light_shadowed = vec3(0.0);
  switch (cl.type) {
    case CLOSURE_BSDF_TRANSLUCENT_ID:
      cl_light.N = -cl.N;
      break;
    case CLOSURE_BSSRDF_BURLEY_ID:
    case CLOSURE_BSDF_DIFFUSE_ID:
      break;
    case CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID:
      cl_light.ltc_mat = LTC_GGX_MAT(dot(cl.N, V), cl.data.x);
      cl_light.type = LIGHT_SPECULAR;
      break;
    case CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID:
      cl_light.N = -cl.N;
      cl_light.type = LIGHT_SPECULAR;
      break;
    case CLOSURE_NONE_ID:
      /* TODO(fclem): Assert. */
      break;
  }
  return cl_light;
}

float _light_attenuation_surface(
    LightData light, const bool is_directional, vec3 Ng, bool use_subsurface, LightVector lv)
{
  /* TODO(fclem): add cutoff attenuation when back-facing. For now do nothing with Ng. */
  return /* light_attenuation_common(light, is_directional, lv.L);*/
      light_attenuation_facing(light, lv.L, Ng, use_subsurface);
  /* light_influence_attenuation(lv.dist, light.influence_radius_invsqr_surface); */
}

void _light_eval_single_closure(LightData light,
                                LightVector lv,
                                inout ClosureLight cl,
                                vec3 P,
                                vec3 V,
                                float thickness,
                                float attenuation,
                                float visibility)
{
  if (light.power[cl.type] > 0.0) {
    float ltc_result = light_ltc(utility_tx, light, cl.N, V, lv, cl.ltc_mat);
    vec3 out_radiance = light.color * light.power[cl.type] * ltc_result;

    cl.light_shadowed += visibility * out_radiance;
    cl.light_unshadowed += attenuation * out_radiance;
  }
}

void _light_eval_single(uint l_idx,
                        const bool is_directional,
                        inout ClosureLightStack stack,
                        vec3 P,
                        vec3 Ng,
                        vec3 V,
                        float thickness,
                        uint packed_shadows,
                        inout uint shift)
{
  LightData light = light_buf[l_idx];
  int ray_count = uniform_buf.shadow.ray_count;
  int ray_step_count = uniform_buf.shadow.step_count;

  bool use_subsurface = thickness > 0.0;
  LightVector lv = light_vector_get(light, is_directional, P);
  float attenuation = _light_attenuation_surface(light, is_directional, Ng, use_subsurface, lv);
  if (attenuation < LIGHT_ATTENUATION_THRESHOLD) {
    return;
  }
  float shadow = 1.0;
  if (light.tilemap_index != LIGHT_NO_SHADOW) {
#ifdef SHADOW_DEFERRED
    shadow = shadow_unpack(packed_shadows, ray_count, shift);
    shift += ray_count;
#else
    ShadowEvalResult result = shadow_eval(
        light, is_directional, P, Ng, thickness, ray_count, ray_step_count);
    shadow = result.light_visibilty;
#endif
  }
  float visibility = attenuation * shadow;

  /* WATCH(@fclem): Might have to manually unroll for best performance. */
  for (int i = 0; i < LIGHT_CLOSURE_EVAL_COUNT; i++) {
    _light_eval_single_closure(light, lv, stack.cl[i], P, V, thickness, attenuation, visibility);
  }
}

void _light_eval(inout ClosureLightStack stack,
                 vec3 P,
                 vec3 Ng,
                 vec3 V,
                 float vPz,
                 float thickness,
                 uint packed_shadows)
{

  for (int i = 0; i < LIGHT_CLOSURE_EVAL_COUNT; i++) {
    stack.cl[i].light_shadowed = vec3(0.0);
    stack.cl[i].light_unshadowed = vec3(0.0);
  }

  uint shift = 0u;
  /*
  LIGHT_FOREACH_BEGIN_DIRECTIONAL (light_cull_buf, l_idx) {
    _light_eval_single(l_idx, true, stack, P, Ng, V, thickness, packed_shadows, shift);
  }
  LIGHT_FOREACH_END

  LIGHT_FOREACH_BEGIN_LOCAL (light_cull_buf, light_zbin_buf, light_tile_buf, PIXEL, vPz, l_idx) {
    _light_eval_single(l_idx, false, stack, P, Ng, V, thickness, packed_shadows, shift);
  }
  LIGHT_FOREACH_END
  */
  _light_eval_single(0, false, stack, P, Ng, V, thickness, packed_shadows, shift);
}

/* Variations that have less arguments. */

#if !defined(SHADOW_DEFERRED)
void _light_eval(
    inout ClosureLightStack stack, vec3 P, vec3 Ng, vec3 V, float vPz, float thickness)
{
  _light_eval(stack, P, Ng, V, vPz, thickness, 0u);
}

#  if !defined(SHADOW_SUBSURFACE) && defined(LIGHT_ITER_FORCE_NO_CULLING)
void _light_eval(inout ClosureLightStack stack, vec3 P, vec3 Ng, vec3 V)
{
  _light_eval(stack, P, Ng, V, 0.0, 0.0, 0u);
}
#  endif

#endif

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);

  float depth = texelFetch(hiz_tx, texel, 0).r;
  GBufferReader gbuf = gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_normal_tx, texel);

  vec3 P = drw_point_screen_to_world(vec3(uvcoordsvar.xy, depth));
  vec3 Ng = gbuf.surface_N;
  vec3 V = drw_world_incident_vector(P);
  float vPz = dot(drw_view_forward(), P) - dot(drw_view_forward(), drw_view_position());

  ClosureLightStack stack;
  for (int i = 0; i < LIGHT_CLOSURE_EVAL_COUNT && i < gbuf.closure_count; i++) {
    stack.cl[i] = closure_light_new(gbuffer_closure_get(gbuf, i), V);
  }

  /* TODO(fclem): Split thickness computation. */
  float thickness = gbuf.thickness;
#ifdef MAT_SUBSURFACE
  /* NOTE: BSSRDF is supposed to always be the first closure. */
  bool has_sss = gbuffer_closure_get(gbuf, 0).type == CLOSURE_BSSRDF_BURLEY_ID;
  if (has_sss) {
    float shadow_thickness = thickness_from_shadow(P, Ng, vPz);
    thickness = (shadow_thickness != THICKNESS_NO_VALUE) ? max(shadow_thickness, gbuf.thickness) :
                                                           gbuf.thickness;

    /* Add one translucent closure for all SSS closure. Reuse the same lighting. */
    ClosureLight cl_light;
    cl_light.N = -Ng;
    cl_light.ltc_mat = LTC_LAMBERT_MAT;
    cl_light.type = LIGHT_DIFFUSE;
    stack.cl[gbuf.closure_count] = cl_light;
  }
#endif

  light_eval(stack, P, Ng, V, vPz, thickness);
#ifdef MAT_SUBSURFACE
  if (has_sss) {
    /* Add to diffuse light for processing inside the Screen Space SSS pass.
     * The translucent light is not outputted as a separate quantity because
     * it is over the closure_count. */
    vec3 sss_profile = subsurface_transmission(gbuffer_closure_get(gbuf, 0).data.rgb, thickness);
    stack.cl[0].light_shadowed += stack.cl[gbuf.closure_count].light_shadowed * sss_profile;
    stack.cl[0].light_unshadowed += stack.cl[gbuf.closure_count].light_unshadowed * sss_profile;
  }
#endif

  if (render_pass_shadow_enabled) {
    vec3 radiance_shadowed = vec3(0);
    vec3 radiance_unshadowed = vec3(0);
    for (int i = 0; i < LIGHT_CLOSURE_EVAL_COUNT && i < gbuf.closure_count; i++) {
      radiance_shadowed += stack.cl[i].light_shadowed;
      radiance_unshadowed += stack.cl[i].light_unshadowed;
    }
    /* TODO(fclem): Change shadow pass to be colored. */
    vec3 shadows = radiance_shadowed * safe_rcp(radiance_unshadowed);
    output_renderpass_value(uniform_buf.render_pass.shadow_id, average(shadows));
  }

  if (use_lightprobe_eval) {
    LightProbeSample samp = lightprobe_load(P, Ng, V);

    for (int i = 0; i < LIGHT_CLOSURE_EVAL_COUNT && i < gbuf.closure_count; i++) {
      ClosureUndetermined cl = gbuffer_closure_get(gbuf, i);
      switch (cl.type) {
        case CLOSURE_BSDF_TRANSLUCENT_ID:
          /* TODO: Support in ray tracing first. Otherwise we have a discrepancy. */
          stack.cl[i].light_shadowed += lightprobe_eval(samp, to_closure_translucent(cl), P, V);
          break;
        case CLOSURE_BSSRDF_BURLEY_ID:
          /* TODO: Support translucency in ray tracing first. Otherwise we have a discrepancy. */
        case CLOSURE_BSDF_DIFFUSE_ID:
          stack.cl[i].light_shadowed += lightprobe_eval(samp, to_closure_diffuse(cl), P, V);
          break;
        case CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID:
          stack.cl[i].light_shadowed += lightprobe_eval(samp, to_closure_reflection(cl), P, V);
          break;
        case CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID:
          /* TODO(fclem): Add instead of replacing when we support correct refracted light. */
          stack.cl[i].light_shadowed = lightprobe_eval(samp, to_closure_refraction(cl), P, V);
          break;
        case CLOSURE_NONE_ID:
          /* TODO(fclem): Assert. */
          break;
      }
    }
  }
  for (int i = 0; i < LIGHT_CLOSURE_EVAL_COUNT && i < gbuf.closure_count; i++) {
    /* TODO(fclem): Layered texture. */
    if (i == 0) {
      imageStore(direct_radiance_1_img, texel, vec4(stack.cl[i].light_shadowed, 1.0));
    }
    else if (i == 1) {
      imageStore(direct_radiance_2_img, texel, vec4(stack.cl[i].light_shadowed, 1.0));
    }
    else if (i == 2) {
      imageStore(direct_radiance_3_img, texel, vec4(stack.cl[i].light_shadowed, 1.0));
    }
  }
}
