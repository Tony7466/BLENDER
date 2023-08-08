
/**
 * Virtual shadowmapping: Tilemap to texture conversion.
 *
 * For all visible light tilemaps, copy page coordinate to a texture.
 * This avoids one level of indirection when evaluating shadows and allows
 * to use a sampler instead of a SSBO bind.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_matrix_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_shadow_tilemap_lib.glsl)

void main()
{
  int tilemap_index = int(gl_GlobalInvocationID.z);
  ivec2 tile_co = ivec2(gl_GlobalInvocationID.xy);

  ivec2 atlas_texel = shadow_tile_coord_in_atlas(tile_co, tilemap_index);

  ShadowTileMapData tilemap_data = tilemaps_buf[tilemap_index];
  int lod_max = (tilemap_data.projection_type == SHADOW_PROJECTION_CUBEFACE) ? SHADOW_TILEMAP_LOD :
                                                                               0;
  int lod_valid = 0;
  uvec3 page_valid;
  /* With all threads (LOD0 size dispatch) load each lod tile from the highest lod
   * to the lowest, keeping track of the lowest one allocated which will be use for shadowing.
   * Also save which page are to be updated. */
  for (int lod = lod_max; lod >= 0; lod--) {
    ivec2 tile_co_lod = tile_co >> lod;
    int tile_index = shadow_tile_offset(tile_co_lod, tilemap_data.tiles_index, lod);

    ShadowTileData tile = shadow_tile_unpack(tiles_buf[tile_index]);

    bool lod_valid_thread = all(equal(tile_co, tile_co_lod << lod));
    if (tile.is_used && tile.do_update && lod_valid_thread) {
      int view_index = atomicAdd(pages_infos_buf.view_count, 1);
      if (view_index < SHADOW_VIEW_MAX) {
        render_map_buf[view_index] = shadow_page_pack(tile.page);
        /* Add page to clear list. */
        atomicAdd(clear_dispatch_buf.num_groups_z, 1u);
        /* Tag tile as rendered. */
        atomicOr(tiles_buf[tile_index], SHADOW_IS_RENDERED);
        /* Statistics. */
        atomicAdd(statistics_buf.page_rendered_count, 1);

        /* Setup the view. */
        view_infos_buf[view_index].viewmat = tilemap_data.viewmat;
        view_infos_buf[view_index].viewinv = inverse(tilemap_data.viewmat);

        float lod_res = float(SHADOW_TILEMAP_RES >> lod);
        vec2 tile_start = (vec2(tile_co_lod) / lod_res) * 2.0 - 1.0;
        vec2 tile_end = (vec2(tile_co_lod + 1u) / lod_res) * 2.0 - 1.0;

        int clip_index = tilemap_data.clip_data_index;
        float clip_far = tilemaps_clip_buf[clip_index].clip_far_stored;
        float clip_near = tilemaps_clip_buf[clip_index].clip_near_stored;

        mat4x4 winmat;
        if (tilemap_data.projection_type != SHADOW_PROJECTION_CUBEFACE) {
          tile_start *= tilemap_data.half_size;
          tile_end *= tilemap_data.half_size;
          tile_start += tilemap_data.center_offset;
          tile_end += tilemap_data.center_offset;

          winmat = projection_orthographic(
              tile_start.x, tile_end.x, tile_start.y, tile_end.y, clip_near, clip_far);
        }
        else {
          tile_start *= clip_near;
          tile_end *= clip_near;

          winmat = projection_perspective(
              tile_start.x, tile_end.x, tile_start.y, tile_end.y, clip_near, clip_far);
        }

        view_infos_buf[view_index].winmat = winmat;
        view_infos_buf[view_index].wininv = inverse(winmat);
      }
    }

    /* Save highest lod for this thread. */
    if (tile.is_used && lod > 0) {
      /* Reload the page in case there was an allocation in the valid thread. */
      page_valid = tile.page;
      lod_valid = lod;
    }
    else if (lod == 0 && lod_valid != 0 && !tile.is_allocated) {
      /* If the tile is not used, store the valid LOD level in LOD0. */
      tile.page = page_valid;
      tile.lod = lod_valid;
      /* This is not a real ownership. It is just a tag so that the shadowing is deemed correct. */
      tile.is_allocated = true;
    }

    if (lod == 0) {
      imageStore(tilemaps_img, atlas_texel, uvec4(shadow_tile_pack(tile)));
    }
  }

  if (all(equal(gl_GlobalInvocationID, uvec3(0)))) {
    /* Clamp it as it can underflow if there is too much tile present on screen. */
    pages_infos_buf.page_free_count = max(pages_infos_buf.page_free_count, 0);
  }
}
