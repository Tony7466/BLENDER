/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Compute light objects lighting contribution using captured Gbuffer data.
 */

#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_light_eval_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_lightprobe_eval_lib.glsl)

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);

  float depth = texelFetch(hiz_tx, texel, 0).r;

  GBufferData gbuf = gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_color_tx, texel);

  ClosureLight cl_diff;
  cl_diff.N = gbuf.has_diffuse ? gbuf.diffuse.N : gbuf.reflection.N;
  cl_diff.ltc_mat = LTC_LAMBERT_MAT;
  cl_diff.type = LIGHT_DIFFUSE;

  vec3 P = get_world_space_from_depth(uvcoordsvar.xy, depth);
  vec3 Ng = gbuf.diffuse.N;
  vec3 V = cameraVec(P);
  float vPz = dot(cameraForward, P) - dot(cameraForward, cameraPos);
  light_eval(cl_diff, P, Ng, V, vPz, gbuf.thickness);

  vec3 albedo = gbuf.diffuse.color + gbuf.reflection.color + gbuf.refraction.color;

  out_radiance = vec4(cl_diff.light_shadowed * albedo, 0.0);
}
