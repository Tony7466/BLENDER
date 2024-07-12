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
  /* Copy of common_fullscreen_vert.
   * We need to declare the vertex shader with the node tree lib includes because the node
   * functions are appended in the vertex shader too. */
  /* TODO: Don't append node functions to the vertex shader. */
  int v = gl_VertexID % 3;
  float x = -1.0 + float((v & 1) << 2);
  float y = -1.0 + float((v & 2) << 1);
  gl_Position = vec4(x, y, 1.0, 1.0);
  uvcoordsvar = vec4((gl_Position.xy + 1.0) * 0.5, 0.0, 0.0);
}
