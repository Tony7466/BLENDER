/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Combine light passes to the combined color target and apply surface colors.
 * This also fills the different render passes.
 */

#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_renderpass_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_debug_gradients_lib.glsl)

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);

  uint header = texelFetch(gbuf_header_tx, texel, 0).r;

  if (header == 0u) {
    discard;
    return;
  }

  uvec4 closure_types = (uvec4(header) >> uvec4(0u, 4u, 8u, 12u)) & 15u;
  float storage_cost = reduce_add(vec4(not(equal(closure_types, uvec4(0u)))));
  float evaluation_cost = reduce_add(vec4(equal(closure_types, uvec4(GBUF_REFLECTION)))) * 1.0 +
                          reduce_add(vec4(equal(closure_types, uvec4(GBUF_REFRACTION)))) * 1.0 +
                          reduce_add(vec4(equal(closure_types, uvec4(GBUF_DIFFUSE)))) * 1.0 +
                          reduce_add(vec4(equal(closure_types, uvec4(GBUF_SSS)))) * 1.0;

  switch (eDebugMode(debug_mode)) {
    default:
    case DEBUG_GBUFFER_STORAGE:
      out_color_add = vec4(pow(heatmap_gradient((storage_cost + 0.5) / 4.0), vec3(2.2)), 0.0);
      break;
    case DEBUG_GBUFFER_EVALUATION:
      out_color_add = vec4(pow(heatmap_gradient((evaluation_cost + 0.5) / 4.0), vec3(2.2)), 0.0);
      break;
  }

  out_color_mul = vec4(0.5);
}
