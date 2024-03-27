/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Virtual shadow-mapping: Amend sampling tile atlas.
 *
 * In order to support sampling different LOD for clipmap shadow projections, we need to scan
 * through the LOD tilemaps from lowest LOD to highest LOD, gathering the last valid tile along the
 * way for the current destination tile. For each new level we gather the previous level tiles from
 * local memory using the correct relative offset from the previous level as they might not be
 * aligned.
 *
 * TODO(fclem): This shader **should** be dispatched for one thread-group per directional light.
 * Currently this shader is dispatched with one thread-group for all directional light.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_matrix_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_light_iter_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_shadow_tilemap_lib.glsl)

ShadowSamplingTile tiles_local[SHADOW_TILEMAP_RES][SHADOW_TILEMAP_RES];

void main()
{
  ivec2 tile_co = ivec2(gl_GlobalInvocationID.xy);

  LIGHT_FOREACH_BEGIN_DIRECTIONAL (light_cull_buf, l_idx) {
    LightData light = light_buf[l_idx];

    int index_start = light.tilemap_index;
    int index_end = light_tilemap_max_get(light);

    for (int tilemap_index = index_start; tilemap_index <= index_end; tilemap_index++) {
      ShadowTileMapData tilemap_data = tilemaps_buf[tilemap_index];

      if (tilemap_data.projection_type != SHADOW_PROJECTION_CLIPMAP) {
        break;
      }

      ivec2 atlas_texel = shadow_tile_coord_in_atlas(tile_co, tilemap_index);
      ShadowSamplingTile tile;
      tile.is_valid = false;

      imageStore(tilemaps_img, atlas_texel, uvec4(shadow_sampling_tile_pack(tile)));
    }
  }
  LIGHT_FOREACH_END
}
