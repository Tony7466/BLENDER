/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Virtual shadow-mapping: Update tagging
 *
 * Any updated shadow caster needs to tag the shadow map tiles it was in and is now into.
 * This is done in 2 pass of this same shader. One for past object bounds and one for new object
 * bounds. The bounding boxes are rasterized and each fragment shader invocation tags the
 * appropriate tiles.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_shadow_tilemap_lib.glsl)

void main()
{
  ShadowTileMapData tilemap = tilemaps_buf[tilemap_index];

  uvec2 texel = uvec2(gl_FragCoord.xy);
  for (int lod = 0; lod <= SHADOW_TILEMAP_LOD; lod++) {
    /* NOTE: Can't reject any thread in LODs because we need to be conservative.
     * This can create some atomic contention but for now we live with that. */
    int tile_index = shadow_tile_offset(texel >> lod, tilemap.tiles_index, lod);
    atomicOr(tiles_buf[tile_index], uint(SHADOW_DO_UPDATE));
  }
}
