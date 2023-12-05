/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Convert the tile classification texture into streams of tiles of each types.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)

void main()
{
  /* Doesn't matter. Doesn't get rasterized. */
  gl_Position = vec4(0.0);

  ivec2 tile_coord = ivec2(gl_VertexID / 128, gl_VertexID % 128);

  if (gl_VertexID == 0) {
    closure_diffuse_draw_buf.instance_len = 1u;
  }

  if (!in_texture_range(tile_coord, tile_mask_tx)) {
    return;
  }

  bool has_diffuse = (texelFetch(tile_mask_tx, ivec3(tile_coord, 0), 0).r != 0u);
  bool has_reflection = (texelFetch(tile_mask_tx, ivec3(tile_coord, 1), 0).r != 0u);
  bool has_refraction = (texelFetch(tile_mask_tx, ivec3(tile_coord, 2), 0).r != 0u);

  if (!has_diffuse && !has_reflection /* && !has_refraction */) {
    /* TODO: More granular selection. */
    return;
  }

  if (has_diffuse || has_reflection) {
    uint tile_index = atomicAdd(closure_diffuse_draw_buf.vertex_len, 6u) / 6u;
    closure_diffuse_tile_buf[tile_index] = packUvec2x16(uvec2(tile_coord));
  }
}
