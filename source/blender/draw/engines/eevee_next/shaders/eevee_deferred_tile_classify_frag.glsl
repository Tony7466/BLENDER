/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * This pass load Gbuffer data and output a mask of tiles to process.
 * This mask is then processed by the compaction phase.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)

void main()
{
  GBufferReader gbuf = gbuffer_read_header_closure_types(in_gbuffer_header);

#ifdef USE_STENCIL_EXPORT
  gl_FragStencilRefARB = gbuf.closure_count;
#else
  /* Instead of setting the stencil at once, we do it (literally) bit by bit.
   * Discard fragments that do not have a number of closure whose bit-pattern overlap the current
   * bit. */
  if ((current_bit & int(gbuf.closure_count)) == 0) {
    discard;
    return;
  }
#endif
}
