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
  ivec2 texel = ivec2(gl_FragCoord.xy);

  GBufferData gbuf = gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_color_tx, texel);

  eClosureBits closure_mask = eClosureBits(0u);

  if (gbuf.has_diffuse) {
    closure_mask |= CLOSURE_DIFFUSE;
  }
  if (gbuf.has_reflection) {
    closure_mask |= CLOSURE_REFLECTION;
  }
  // if (gbuf.has_refraction) {
  //   closure_mask |= CLOSURE_REFRACTION;
  // }

  if (closure_mask != eClosureBits(0u)) {
    ivec2 tile_co = texel >> closure_tile_size_shift;
    int tile_index = tile_co.x + tile_co.y * closure_tile_per_row;

    uint original_value = atomicOr(tile_mask_buf[tile_index], closure_mask);
    uint new_bits = ~original_value & closure_mask;
    if (original_value == 0u) {
      // if (new_bits != 0u) {
      /* First encountered pixel for some lit closure.
       * Request a tile for the new closure flags. */
      // if (flag_test(new_bits, CLOSURE_DIFFUSE)) {
      uint tile_index = atomicAdd(closure_diffuse_draw_buf.vertex_len, 6u) / 6u;
      closure_diffuse_tile_buf[tile_index] = packUvec2x16(uvec2(tile_co));
      if (tile_index == 0u) {
        closure_diffuse_draw_buf.instance_len = 1u;
      }
      // }
      // if (flag_test(new_bits, CLOSURE_REFLECTION)) {
      //   int tile_index = atomicAdd(light_reflection_draw_buf.vertex_len, 6u) / 6u;
      //   light_reflection_tile_buf[tile_index] = packUvec2x16(uvec2(tile_co));
      //   if (tile_index == 0) {
      //     light_reflection_draw_buf.instance_len = 1;
      //   }
      // }
      // if (flag_test(new_bits, CLOSURE_REFRACTION)) {
      //   int tile_index = atomicAdd(light_draw_refraction_buf.vertex_len, 6u) / 6u;
      //   if (tile_index == 0) {
      //     light_draw_buf.instance_len = 1;
      //   }
      // }
    }
  }
}
