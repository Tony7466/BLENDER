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
  if (all(equal(gl_LocalInvocationID, uvec3(0)))) {
    ivec2 tile_mask_size = imageSize(tile_mask_img);
    /* Generate one quad for each tile on screen.
     * Invalid tiles will produce no fragment. */
    light_draw_buf.vertex_len = 6;
    light_draw_buf.instance_len = (tile_mask_size.x * tile_mask_size.y);
    light_draw_buf.vertex_first = 0u;
    light_draw_buf.instance_first_array = 0u;
  }

  ivec2 texel = ivec2(gl_FragCoord.xy);

  GBufferData gbuf = gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_color_tx, texel);

  eClosureBits closure_mask = eClosureBits(0u);

  if (gbuf.has_diffuse) {
    closure_mask |= CLOSURE_DIFFUSE;
  }
  if (gbuf.has_reflection) {
    closure_mask |= CLOSURE_REFLECTION;
  }
  if (gbuf.has_refraction) {
    closure_mask |= CLOSURE_REFRACTION;
  }

  ivec2 tile_co = texel >> uniform_buf.pipeline.tile_size_shift;

  imageAtomicXor(tile_mask_img, tile_co, uvec4(closure_mask));
}
