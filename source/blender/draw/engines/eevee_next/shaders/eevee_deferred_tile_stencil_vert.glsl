/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Load tile classification data and mark stencil areas.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)

void main()
{
  int tile_id = gl_VertexID / 6;
  int vertex_id = gl_VertexID % 6;
  uvec2 tile_coord = uvec2(tile_id / 128, tile_id % 128);

  if (vertex_id == 0 || vertex_id == 5) {
    /* Doing the loads only for 2 vertex is faster. */
    bool has_diffuse = (texelFetch(tile_mask_tx, ivec3(tile_coord, 0), 0).r != 0u);
    bool has_reflection = (texelFetch(tile_mask_tx, ivec3(tile_coord, 1), 0).r != 0u);
    bool has_refraction = (texelFetch(tile_mask_tx, ivec3(tile_coord, 2), 0).r != 0u);

    if (!has_diffuse && !has_reflection /* && !has_refraction */) {
      /* TODO: More granular selection. */
      gl_Position = vec4(0.0);
      return;
    }
  }

  /* Generate Quad with 2 triangles with same winding.
   * This way it can be merged on some hardware. */
  int v = (vertex_id > 2) ? (3 - (vertex_id - 3)) : vertex_id;
  ivec2 tile_corner = ivec2(v & 1, v >> 1);

  int tile_size = (1 << closure_tile_size_shift);
  vec2 ss_coord = vec2((tile_coord + tile_corner) * tile_size) /
                  vec2(imageSize(out_direct_radiance_img));
  vec2 ndc_coord = ss_coord * 2.0 - 1.0;

  /* gl_Position expects Homogenous space coord. But this is the same thing as NDC in 2D mode. */
  gl_Position = vec4(ndc_coord, 1.0, 1.0);
}
