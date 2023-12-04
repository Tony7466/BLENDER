/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * This pass load Gbuffer data and output a mask of tiles to process.
 * This mask is then processed by the compaction phase.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(draw_view_lib.glsl)

void main()
{
  int tile_size = (1 << closure_tile_size_shift);
  int tile_id = gl_VertexID / 6;
  int vertex_id = gl_VertexID % 6;
  uvec2 tile_coord = unpackUvec2x16(closure_tile_buf[tile_id]);

  /* Generate Quad with 2 triangle with same winding.
   * This way the can be merged on some hardware. */
  int v = (vertex_id > 2) ? (3 - (vertex_id - 3)) : vertex_id;
  ivec2 tile_corner = ivec2(v & 1, v >> 1);

  vec2 ss_coord = vec2((tile_coord + tile_corner) * tile_size) /
                  vec2(imageSize(out_direct_radiance_img));
  vec2 ndc_coord = drw_screen_to_ndc(ss_coord);

  /* gl_Position expects Homogenous space coord. But this is the same thing as NDC in 2D mode. */
  gl_Position = vec4(ndc_coord, 1.0, 1.0);
  interp.uv = ss_coord;
}
