/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Virtual shadow-mapping: Usage un-tagging
 *
 * Remove used tag from masked tiles (LOD overlap).
 */

#pragma BLENDER_REQUIRE(eevee_shadow_tilemap_lib.glsl)

/* Reuse the same enum values for these transient flag during the amend phase.
 * They are never written to the tile data SSBO. */
#define SHADOW_TILE_AMENDED SHADOW_IS_RENDERED
/* Visibility value to write back. */
#define SHADOW_TILE_MASKED SHADOW_IS_ALLOCATED

shared uint tiles[SHADOW_TILEDATA_PER_TILEMAP];
shared uint levels_rendered = 0u;

int shadow_tile_offset_lds(ivec2 tile, int lod)
{
  return shadow_tile_offset(tile, 0, lod);
}

/* Deactivate threads that are not part of this LOD. Will only let pass threads which tile
 * coordinate fits the given tilemap LOD. */
bool thread_mask(ivec2 tile_co, int lod)
{
  const uint lod_size = uint(SHADOW_TILEMAP_RES);
  return all(lessThan(tile_co, ivec2(lod_size >> lod)));
}

void main()
{
  ivec2 tile_co = ivec2(gl_GlobalInvocationID.xy);
  uint tilemap_index = gl_GlobalInvocationID.z;
  ShadowTileMapData tilemap = tilemaps_buf[tilemap_index];

  if (tilemap.projection_type == SHADOW_PROJECTION_CUBEFACE) {

    /* Load all data to LDS. Allows us to do some modification on the flag bits and only flush to
     * main memory the usage bit. */
    for (int lod = 0; lod <= SHADOW_TILEMAP_LOD; lod++) {
      if (thread_mask(tile_co, lod)) {
        int tile_offset = shadow_tile_offset(tile_co, tilemap.tiles_index, lod);
        int tile_lds = shadow_tile_offset_lds(tile_co, lod);
        ShadowTileDataPacked tile_data = tiles_buf[tile_offset];

        if ((tile_data & SHADOW_IS_USED) == 0) {
          /* Do not consider this tile as going to be rendered if it is not used.
           * Simplify checks later. This is a local modification. */
          tile_data &= ~SHADOW_DO_UPDATE;
        }
        /* Clear these flags as they could contain any values. */
        tile_data &= ~(SHADOW_TILE_AMENDED | SHADOW_TILE_MASKED);

        tiles[tile_lds] = tile_data;
      }
    }

    barrier();

    /* For each level collect the number of used (or masked) tile that are covering the tile from
     * the level underneath. If this adds up to 4 the underneath tile is flag unused as its data
     * is not needed for rendering.
     *
     * This is because 2 receivers can tag "used" the same area of the shadow-map but with
     * different LODs. */
    for (int lod = 1; lod <= SHADOW_TILEMAP_LOD; lod++) {
      if (thread_mask(tile_co, lod)) {
        ivec2 tile_co_prev_lod = tile_co * 2;
        int prev_lod = lod - 1;

        int tile_0 = shadow_tile_offset_lds(tile_co_prev_lod + ivec2(0, 0), prev_lod);
        int tile_1 = shadow_tile_offset_lds(tile_co_prev_lod + ivec2(1, 0), prev_lod);
        int tile_2 = shadow_tile_offset_lds(tile_co_prev_lod + ivec2(0, 1), prev_lod);
        int tile_3 = shadow_tile_offset_lds(tile_co_prev_lod + ivec2(1, 1), prev_lod);
        /* Is masked if all tiles from the previous level were tagged as used. */
        bool is_masked = ((tiles[tile_0] & tiles[tile_1] & tiles[tile_2] & tiles[tile_3]) &
                          SHADOW_IS_USED) != 0;

        int tile_offset = shadow_tile_offset_lds(tile_co, lod);
        if (is_masked) {
          /* Consider this tile occluding lower levels. Use SHADOW_IS_USED flag for that. */
          tiles[tile_offset] |= SHADOW_IS_USED;
          /* Do not consider this tile when checking which tilemap level to render in next loop. */
          tiles[tile_offset] &= ~SHADOW_DO_UPDATE;
          /* Tag as modified so that we can amend it inside the `tiles_buf`. */
          tiles[tile_offset] |= SHADOW_TILE_AMENDED;
          /* Visibility value to write back. */
          tiles[tile_offset] |= SHADOW_TILE_MASKED;
        }
      }
      barrier();
    }

    /* Find list of LOD that contain tiles to render. */
    for (int lod = 0; lod <= SHADOW_TILEMAP_LOD; lod++) {
      if (thread_mask(tile_co, lod)) {
        int tile_offset = shadow_tile_offset_lds(tile_co, lod);
        if ((tiles[tile_offset] & SHADOW_DO_UPDATE) != 0) {
          atomicOr(levels_rendered, 1u << lod);
        }
      }
    }

    barrier();

    /* If there is more LODs to update than the load balancing heuristic allows. */
    const int max_lod_rendered_per_tilemap = 1;
    if (bitCount(levels_rendered) > max_lod_rendered_per_tilemap) {
      /* Find the cutoff LOD that contain tiles to render. */
      int max_lod = findLSB(levels_rendered);
      /* Allow more than one level. */
      for (int i = 1; i < max_lod_rendered_per_tilemap; i++) {
        max_lod = findLSB(levels_rendered >> (max_lod + 1));
      }
      /* Collapse all bits to highest level. */
      for (int lod = 0; lod < max_lod; lod++) {
        if (thread_mask(tile_co, lod)) {
          int tile_offset = shadow_tile_offset_lds(tile_co, lod);
          if ((tiles[tile_offset] & SHADOW_DO_UPDATE) != 0) {
            /* This tile is now masked and not considered for rendering. */
            tiles[tile_offset] |= SHADOW_TILE_MASKED | SHADOW_TILE_AMENDED;
            /* Tag the associated tile in max_lod to be used as it contains the shadowmap area
             * covered by this collapsed tile.  */
            ivec2 tile_to_co = tile_co >> (max_lod - lod);
            int tile_to_offset = shadow_tile_offset_lds(tile_to_co, max_lod);
            tiles[tile_to_offset] |= SHADOW_IS_USED | SHADOW_TILE_AMENDED;
          }
        }
      }
    }

    barrier();

    /* Flush back visibility bits to the tile SSBO. */
    for (int lod = 0; lod <= SHADOW_TILEMAP_LOD; lod++) {
      if (thread_mask(tile_co, lod)) {
        int tile_offset = shadow_tile_offset(tile_co, tilemap.tiles_index, lod);
        if ((tiles[tile_offset] & SHADOW_TILE_AMENDED) != 0) {
          /* Note that we only flush the visibility so that cached pages can be reused. */
          if ((tiles[tile_offset] & SHADOW_TILE_MASKED) != 0) {
            tiles_buf[tile_offset] &= ~SHADOW_IS_USED;
          }
          else {
            tiles_buf[tile_offset] |= SHADOW_IS_USED;
          }
        }
      }
    }
  }
}
