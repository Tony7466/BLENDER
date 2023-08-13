
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

shared uvec2 rect_min;
shared uvec2 rect_max;
shared int view_index;

void main()
{
  int tilemap_index = int(gl_GlobalInvocationID.z);
  ivec2 tile_co = ivec2(gl_GlobalInvocationID.xy);

  ivec2 atlas_texel = shadow_tile_coord_in_atlas(tile_co, tilemap_index);

  ShadowTileMapData tilemap_data = tilemaps_buf[tilemap_index];
  bool is_cubemap = (tilemap_data.projection_type == SHADOW_PROJECTION_CUBEFACE);
  int lod_max = is_cubemap ? SHADOW_TILEMAP_LOD : 0;
  int lod_valid = 0;
  uvec3 page_valid;
  /* With all threads (LOD0 size dispatch) load each lod tile from the highest lod
   * to the lowest, keeping track of the lowest one allocated which will be use for shadowing.
   * This guarantee a O(1) lookup time.
   * Add one render view per LOD that has tiles to be rendered. */
  for (int lod = lod_max; lod >= 0; lod--) {
    ivec2 tile_co_lod = tile_co >> lod;
    int tile_index = shadow_tile_offset(tile_co_lod, tilemap_data.tiles_index, lod);

    ShadowTileData tile = shadow_tile_unpack(tiles_buf[tile_index]);

    /* Compute update area. */
    if (all(equal(gl_LocalInvocationID, uvec3(0)))) {
      rect_min = uvec2(SHADOW_TILEMAP_RES);
      rect_max = uvec2(0u);
      view_index = -1;
    }

    barrier();

    bool lod_valid_thread = all(equal(tile_co, tile_co_lod << lod));
    bool do_page_render = tile.is_used && tile.do_update && lod_valid_thread;
    if (do_page_render) {
      atomicMin(rect_min.x, tile_co_lod.x);
      atomicMin(rect_min.y, tile_co_lod.y);
      atomicMax(rect_max.x, tile_co_lod.x + 1);
      atomicMax(rect_max.y, tile_co_lod.y + 1);
      /* Tag tile as rendered. There is a barrier after the read. So it is safe. */
      tiles_buf[tile_index] |= SHADOW_IS_RENDERED;
    }

    barrier();

    /* Issue one view if there is an update in the LOD. */
    if (all(equal(gl_LocalInvocationID, uvec3(0)))) {
      bool lod_has_update = rect_min.x < rect_max.x;
      if (lod_has_update) {
        view_index = atomicAdd(pages_infos_buf.view_count, 1);
        if (view_index < SHADOW_VIEW_MAX) {
          /* Setup the view. */
          view_lod_buf[view_index] = lod;

          view_infos_buf[view_index].viewmat = tilemap_data.viewmat;
          view_infos_buf[view_index].viewinv = inverse(tilemap_data.viewmat);

          float lod_res = float(SHADOW_TILEMAP_RES >> lod);
          /* TODO(fclem): These should be the culling planes, not the view projection. */
          // vec2 view_start = (vec2(rect_min) / lod_res) * 2.0 - 1.0;
          // vec2 view_end = (vec2(rect_max) / lod_res) * 2.0 - 1.0;
          vec2 view_start = vec2(-1.0);
          vec2 view_end = vec2(1.0);

          int clip_index = tilemap_data.clip_data_index;
          float clip_far = tilemaps_clip_buf[clip_index].clip_far_stored;
          float clip_near = tilemaps_clip_buf[clip_index].clip_near_stored;

          mat4x4 winmat;
          if (tilemap_data.projection_type != SHADOW_PROJECTION_CUBEFACE) {
            view_start *= tilemap_data.half_size;
            view_end *= tilemap_data.half_size;
            view_start += tilemap_data.center_offset;
            view_end += tilemap_data.center_offset;

            winmat = projection_orthographic(
                view_start.x, view_end.x, view_start.y, view_end.y, clip_near, clip_far);
          }
          else {
            view_start *= clip_near;
            view_end *= clip_near;

            winmat = projection_perspective(
                view_start.x, view_end.x, view_start.y, view_end.y, clip_near, clip_far);
          }

          view_infos_buf[view_index].winmat = winmat;
          view_infos_buf[view_index].wininv = inverse(winmat);
        }
      }
    }

    barrier();

    if ((view_index >= 0) && (view_index < SHADOW_VIEW_MAX) && lod_valid_thread) {
      uint page_packed = shadow_page_pack(tile.page);
      /* Add page to render map. */
      int render_page_index = shadow_render_page_index_get(view_index, tile_co_lod);
      render_map_buf[render_page_index] = do_page_render ? page_packed : 0xFFFFFFFFu;

      if (do_page_render) {
        /* Add page to clear list. */
        uint clear_page_index = atomicAdd(clear_dispatch_buf.num_groups_z, 1u);
        clear_list_buf[clear_page_index] = page_packed;
        /* Statistics. */
        atomicAdd(statistics_buf.page_rendered_count, 1);
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
