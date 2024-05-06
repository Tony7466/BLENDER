/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_matrix_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_shadow_tilemap_lib.glsl)

#define EEVEE_SHADOW_LIB

#ifdef SHADOW_READ_ATOMIC
#  define SHADOW_ATLAS_TYPE usampler2DArrayAtomic
#else
#  define SHADOW_ATLAS_TYPE usampler2DArray
#endif

float shadow_read_depth(SHADOW_ATLAS_TYPE atlas_tx,
                        usampler2D tilemaps_tx,
                        int tilemap_index,
                        vec2 tilemap_uv)
{
  /* Prevent out of bound access. Assumes the input is already non negative. */
  tilemap_uv = min(tilemap_uv, vec2(0.99999));

  ivec2 texel_coord = ivec2(tilemap_uv * float(SHADOW_MAP_MAX_RES));
  /* Using bitwise ops is way faster than integer ops. */
  const int page_shift = SHADOW_PAGE_LOD;
  const int page_mask = ~(0xFFFFFFFF << SHADOW_PAGE_LOD);

  ivec2 tile_coord = texel_coord >> page_shift;
  ShadowSamplingTile tile = shadow_tile_load(tilemaps_tx, tile_coord, tilemap_index);

  if (!tile.is_valid) {
    return -1.0;
  }
  /* Shift LOD0 pixels so that they get wrapped at the right position for the given LOD. */
  /* TODO convert everything to uint to avoid signed int operations. */
  texel_coord += ivec2(tile.lod_offset << SHADOW_PAGE_LOD);
  /* Scale to LOD pixels (merge LOD0 pixels together) then mask to get pixel in page. */
  ivec2 texel_page = (texel_coord >> int(tile.lod)) & page_mask;
  ivec3 texel = ivec3((ivec2(tile.page.xy) << page_shift) | texel_page, tile.page.z);

  return uintBitsToFloat(texelFetch(atlas_tx, texel, 0).r);
}

/* ---------------------------------------------------------------------- */
/** \name Shadow Sampling Functions
 * \{ */

float shadow_punctual_sample_get(SHADOW_ATLAS_TYPE atlas_tx,
                                 usampler2D tilemaps_tx,
                                 LightData light,
                                 vec3 P)
{
  vec3 lP = transform_point_inversed(light.object_to_world, P);
  int face_id = shadow_punctual_face_index_get(lP);
  lP = shadow_punctual_local_position_to_face_local(face_id, lP);
  ShadowCoordinates coord = shadow_punctual_coordinates(light, lP, face_id);

  float radial_dist = shadow_read_depth(atlas_tx, tilemaps_tx, coord.tilemap_index, coord.uv);
  if (radial_dist == -1.0) {
    return 1e10;
  }
  float receiver_dist = length(lP);
  float occluder_dist = radial_dist;
  return receiver_dist - occluder_dist;
}

float shadow_directional_sample_get(SHADOW_ATLAS_TYPE atlas_tx,
                                    usampler2D tilemaps_tx,
                                    LightData light,
                                    vec3 P)
{
  vec3 lP = transform_direction(light.object_to_world, P);
  ShadowCoordinates coord = shadow_directional_coordinates(light, lP);

  float depth = shadow_read_depth(atlas_tx, tilemaps_tx, coord.tilemap_index, coord.uv);
  if (depth == -1.0) {
    return 1e10;
  }
  float receiver_dist = lP.z;
  float occluder_dist = -depth - orderedIntBitsToFloat(light.clip_near);
#ifdef GPU_FRAGMENT_SHADER
  if (ivec2(gl_FragCoord.xy) == ivec2(500)) {
    drw_print(occluder_dist);
    drw_print(receiver_dist);
    drw_print(occluder_dist - receiver_dist);
  }
#endif
  return occluder_dist - receiver_dist;
}

float shadow_sample(const bool is_directional,
                    SHADOW_ATLAS_TYPE atlas_tx,
                    usampler2D tilemaps_tx,
                    LightData light,
                    vec3 P)
{
  if (is_directional) {
    return shadow_directional_sample_get(atlas_tx, tilemaps_tx, light, P);
  }
  else {
    return shadow_punctual_sample_get(atlas_tx, tilemaps_tx, light, P);
  }
}

/** \} */
