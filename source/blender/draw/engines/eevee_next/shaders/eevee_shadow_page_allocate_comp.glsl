/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Virtual shadow-mapping: Allocation.
 *
 * Allocates pages to tiles needing them.
 * Note that allocation can fail, in this case the tile is left with no page.
 */

#pragma BLENDER_REQUIRE(eevee_shadow_page_ops_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_shadow_tilemap_lib.glsl)

void main()
{
  ShadowTileMapData tilemap_data = tilemaps_buf[gl_GlobalInvocationID.z];

  int tile_start = tilemap_data.tiles_index;
  for (int lod = 0; lod <= SHADOW_TILEMAP_LOD; lod++) {
    int lod_len = SHADOW_TILEMAP_LOD0_LEN >> (lod * 2);
    int local_tile = int(gl_LocalInvocationID.x);
    if (local_tile < lod_len) {
      int tile_index = tile_start + local_tile;

      ShadowTileData tile = shadow_tile_unpack(tiles_buf[tile_index]);
      if (tile.is_used && !tile.is_allocated) {
        shadow_page_alloc(tile);
        tile.lod = lod;
        tiles_buf[tile_index] = shadow_tile_pack(tile);
      }

      if (tile.is_used) {
        atomicAdd(statistics_buf.page_used_count, 1);
      }
      if (tile.is_used && tile.do_update) {
        atomicAdd(statistics_buf.page_update_count, 1);
      }
      if (tile.is_allocated) {
        atomicAdd(statistics_buf.page_allocated_count, 1);
      }
    }
    tile_start += lod_len;
  }

#ifdef SHADOW_UPDATE_TBDR_ROG
  if (gl_GlobalInvocationID.z == 0) {
    /* Temp: Clear render_map_buf in Metal, as we need to ensure that only the tiles updated in the
     * current pass are marked with valid IDs. */
    int start = SHADOW_VIEW_MAX * gl_GlobalInvocationID.x;
    for (int i = start; i < start + SHADOW_VIEW_MAX; i++) {
      render_map_buf[i] = 0xFFFFFFFFu;
    }
  }

  /* Temp: Prepare maximal draw call for tile clear and storage passes. */
  DrawCommand tile_pass_cmd;
  tile_pass_cmd.vertex_len = 6u;
  tile_pass_cmd.instance_len = (SHADOW_TILEMAP_RES * SHADOW_TILEMAP_RES) * SHADOW_VIEW_MAX;
  tile_pass_cmd.vertex_first = 0u;
  tile_pass_cmd.base_index = 0u;
  tile_page_pass_buf = tile_pass_cmd;
#endif
}
