/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(npr_nodetree_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_deferred_combine_lib.glsl)

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);
  if (texelFetch(npr_index_tx, texel, 0).r != npr_index) {
    /* TODO(NPR): Convert NPR index to depth and use depth testing instead. */
    discard;
    return;
  }

  DeferredCombine dc = deferred_combine(texel);
  deferred_combine_clamp(dc);

  g_combined_color = deferred_combine_final_output(dc);
  g_combined_color.a = saturate(1.0 - g_combined_color.a);
  g_diffuse_color = vec4(dc.diffuse_color, 1.0);
  g_diffuse_direct = vec4(dc.diffuse_direct, 1.0);
  g_diffuse_indirect = vec4(dc.diffuse_indirect, 1.0);
  g_specular_color = vec4(dc.specular_color, 1.0);
  g_specular_direct = vec4(dc.specular_direct, 1.0);
  g_specular_indirect = vec4(dc.specular_indirect, 1.0);

  out_color = nodetree_npr().rgb;
}
