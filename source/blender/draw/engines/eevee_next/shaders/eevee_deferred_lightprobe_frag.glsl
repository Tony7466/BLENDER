/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Compute light objects lighting contribution using Gbuffer data.
 */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_lightprobe_eval_lib.glsl)

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);

  float depth = texelFetch(hiz_tx, texel, 0).r;
  GBufferReader gbuf = gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_normal_tx, texel);

  if (gbuf.closure_count == 0) {
    return;
  }

  vec3 P = drw_point_screen_to_world(vec3(uvcoordsvar.xy, depth));
  vec3 Ng = gbuf.data.surface_N;
  vec3 V = drw_world_incident_vector(P);
  float vPz = dot(drw_view_forward(), P) - dot(drw_view_forward(), drw_view_position());

  vec2 noise_probe = interlieved_gradient_noise(gl_FragCoord.xy, vec2(0, 1), vec2(0.0));
  LightProbeSample samp = lightprobe_load(P, Ng, V);

  /* TODO(fclem): Arbitrary closures. */
  vec3 out_diffuse = vec3(1.0, 0.0, 1.0);
  vec3 out_reflection = vec3(1.0, 0.0, 1.0);
  vec3 out_refraction = vec3(1.0, 0.0, 1.0);

  for (int i = 0; i < GBUFFER_LAYER_MAX && i < gbuf.closure_count; i++) {
    switch (gbuf.closures[i].type) {
      case CLOSURE_BSDF_TRANSLUCENT_ID:
        /* TODO: Support in ray tracing first. Otherwise we have a discrepancy. */
        break;
      case CLOSURE_BSSRDF_BURLEY_ID:
        /* TODO: Support translucency in ray tracing first. Otherwise we have a discrepancy. */
      case CLOSURE_BSDF_DIFFUSE_ID:
        out_diffuse = lightprobe_eval(
            samp, to_closure_diffuse(gbuf.closures[i]), P, V, noise_probe);
        break;
      case CLOSURE_BSDF_MICROFACET_GGX_REFLECTION_ID:
        out_reflection = lightprobe_eval(
            samp, to_closure_reflection(gbuf.closures[i]), P, V, noise_probe);
        break;
      case CLOSURE_BSDF_MICROFACET_GGX_REFRACTION_ID:
        out_refraction = lightprobe_eval(
            samp, to_closure_refraction(gbuf.closures[i]), P, V, noise_probe);
        break;
      case CLOSURE_NONE_ID:
        /* TODO(fclem): Assert. */
        break;
    }
  }

  /* TODO(fclem): Layered texture. */
  if (gbuf.has_diffuse) {
    imageStore(indirect_diffuse_img, texel, vec4(out_diffuse, 1.0));
  }
  if (gbuf.has_reflection) {
    imageStore(indirect_reflection_img, texel, vec4(out_reflection, 1.0));
  }
  if (gbuf.has_refraction) {
    imageStore(indirect_refraction_img, texel, vec4(out_refraction, 1.0));
  }
}
