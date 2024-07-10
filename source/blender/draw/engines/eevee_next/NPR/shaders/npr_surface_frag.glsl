/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(npr_nodetree_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);
  if (texelFetch(npr_index_tx, texel, 0).r != npr_index) {
    /* TODO(NPR): Convert NPR index to depth and use depth testing instead. */
    discard;
    return;
  }

  g_combined_color = texelFetch(combined_tx, texel, 0);

  out_color = nodetree_npr().rgb;
}
