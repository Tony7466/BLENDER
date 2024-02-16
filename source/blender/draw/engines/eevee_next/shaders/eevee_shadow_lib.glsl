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

struct ShadowSampleParams {
  vec3 lP;
  vec3 uv;
  int tilemap_index;
  float z_range;
};

ShadowTileData shadow_tile_data_get(usampler2D tilemaps_tx, ShadowSampleParams params)
{
  /* Prevent out of bound access. Assumes the input is already non negative. */
  vec2 tilemap_uv = min(params.uv.xy, vec2(0.99999));

  ivec2 texel_coord = ivec2(tilemap_uv * float(SHADOW_MAP_MAX_RES));
  /* Using bitwise ops is way faster than integer ops. */
  const int page_shift = SHADOW_PAGE_LOD;

  ivec2 tile_coord = texel_coord >> page_shift;
  return shadow_tile_load(tilemaps_tx, tile_coord, params.tilemap_index);
}

float shadow_read_depth(SHADOW_ATLAS_TYPE atlas_tx,
                        usampler2D tilemaps_tx,
                        ShadowSampleParams params)
{
  /* Prevent out of bound access. Assumes the input is already non negative. */
  vec2 tilemap_uv = min(params.uv.xy, vec2(0.99999));

  ivec2 texel_coord = ivec2(tilemap_uv * float(SHADOW_MAP_MAX_RES));
  /* Using bitwise ops is way faster than integer ops. */
  const int page_shift = SHADOW_PAGE_LOD;

  ivec2 tile_coord = texel_coord >> page_shift;
  ShadowTileData tile = shadow_tile_load(tilemaps_tx, tile_coord, params.tilemap_index);

  if (!tile.is_allocated) {
    return -1.0;
  }

  int page_mask = ~(0xFFFFFFFF << (SHADOW_PAGE_LOD + int(tile.lod)));
  ivec2 texel_page = (texel_coord & page_mask) >> int(tile.lod);
  ivec3 texel = ivec3((ivec2(tile.page.xy) << page_shift) | texel_page, tile.page.z);

  return uintBitsToFloat(texelFetch(atlas_tx, texel, 0).r);
}

struct ShadowEvalResult {
  /* Visibility of the light. */
  float light_visibilty;
  /* Average occluder distance. In world space linear distance. */
  float occluder_distance;
};

/* ---------------------------------------------------------------------- */
/** \name Shadow Sampling Functions
 * \{ */

/* TODO(fclem): Remove. Only here to avoid include order hell with common_math_lib. */
mat4x4 shadow_projection_perspective(
    float left, float right, float bottom, float top, float near_clip, float far_clip)
{
  float x_delta = right - left;
  float y_delta = top - bottom;
  float z_delta = far_clip - near_clip;

  mat4x4 mat = mat4x4(1.0);
  if (x_delta != 0.0 && y_delta != 0.0 && z_delta != 0.0) {
    mat[0][0] = near_clip * 2.0 / x_delta;
    mat[1][1] = near_clip * 2.0 / y_delta;
    mat[2][0] = (right + left) / x_delta; /* NOTE: negate Z. */
    mat[2][1] = (top + bottom) / y_delta;
    mat[2][2] = -(far_clip + near_clip) / z_delta;
    mat[2][3] = -1.0;
    mat[3][2] = (-2.0 * near_clip * far_clip) / z_delta;
    mat[3][3] = 0.0;
  }
  return mat;
}

/**
 * Convert occluder distance in shadow space to world space distance.
 * Assuming the occluder is above the shading point in direction to the shadow projection center.
 */
float shadow_linear_occluder_distance(LightData light,
                                      const bool is_directional,
                                      vec3 lP,
                                      float occluder)
{
  float near = orderedIntBitsToFloat(light.clip_near);
  float far = orderedIntBitsToFloat(light.clip_far);

  float occluder_z = (is_directional) ? (occluder * (far - near) + near) :
                                        ((near * far) / (occluder * (near - far) + far));
  float receiver_z = (is_directional) ? -lP.z : max(abs(lP.x), max(abs(lP.y), abs(lP.z)));

  return receiver_z - occluder_z;
}

ShadowSampleParams shadow_punctual_sample_params_get(usampler2D tilemaps_tx,
                                                     LightData light,
                                                     vec3 P)
{
  vec3 lP = (P - light._position) * mat3(light.object_mat);

  int face_id = shadow_punctual_face_index_get(lP);
  /* Local Light Space > Face Local (View) Space. */
  lP = shadow_punctual_local_position_to_face_local(face_id, lP);
  /* Face Local (View) Space > Clip Space. */
  float clip_far = intBitsToFloat(light.clip_far);
  float clip_near = intBitsToFloat(light.clip_near);
  float clip_side = light.clip_side;
  /* TODO: Could be simplified since frustum is completely symmetrical. */
  mat4 winmat = shadow_projection_perspective(
      -clip_side, clip_side, -clip_side, clip_side, clip_near, clip_far);
  vec3 clip_P = project_point(winmat, lP);
  /* Clip Space > UV Space. */
  vec3 uv_P = saturate(clip_P * 0.5 + 0.5);

  ShadowSampleParams result;
  result.lP = lP;
  result.uv = uv_P;
  result.tilemap_index = light.tilemap_index + face_id;
  result.z_range = 1.0;
  return result;
}

ShadowEvalResult shadow_punctual_sample_get(SHADOW_ATLAS_TYPE atlas_tx,
                                            usampler2D tilemaps_tx,
                                            LightData light,
                                            vec3 P)
{
  ShadowSampleParams params = shadow_punctual_sample_params_get(tilemaps_tx, light, P);

  float depth = shadow_read_depth(atlas_tx, tilemaps_tx, params);

  ShadowEvalResult result;
  result.light_visibilty = float(params.uv.z < depth);
  result.occluder_distance = shadow_linear_occluder_distance(light, false, params.lP, depth);
  return result;
}

ShadowSampleParams shadow_directional_sample_params_get(usampler2D tilemaps_tx,
                                                        LightData light,
                                                        vec3 P)
{
  vec3 lP = P * mat3(light.object_mat);
  ShadowCoordinates coord = shadow_directional_coordinates(light, lP);

  float clip_near = orderedIntBitsToFloat(light.clip_near);
  float clip_far = orderedIntBitsToFloat(light.clip_far);
  /* Assumed to be non-null. */
  float z_range = clip_far - clip_near;
  float dist_to_near_plane = -lP.z - clip_near;

  int level = shadow_directional_level(light, lP - light._position);
  /* This difference needs to be less than 32 for the later shift to be valid.
   * This is ensured by ShadowDirectional::clipmap_level_range(). */
  int level_relative = level - light.clipmap_lod_min;

  int lod_relative = (light.type == LIGHT_SUN_ORTHO) ? light.clipmap_lod_min : level;

  vec2 clipmap_origin = vec2(light._clipmap_origin_x, light._clipmap_origin_y);
  vec2 clipmap_pos = lP.xy - clipmap_origin;
  vec2 tilemap_uv = clipmap_pos * exp2(-float(lod_relative)) + 0.5;

  /* Compute offset in tile. */
  ivec2 clipmap_offset = shadow_decompress_grid_offset(
      light.type, light.clipmap_base_offset, level_relative);
  /* Translate tilemap UVs to its origin. */
  tilemap_uv -= vec2(clipmap_offset) / float(SHADOW_TILEMAP_RES);
  /* Clamp to avoid out of tilemap access. */
  tilemap_uv = saturate(tilemap_uv);

  ShadowSampleParams result;
  result.lP = lP;
  result.uv = vec3(tilemap_uv, dist_to_near_plane);
  result.tilemap_index = light.tilemap_index + level_relative;
  result.z_range = z_range;
  return result;
}

ShadowEvalResult shadow_directional_sample_get(SHADOW_ATLAS_TYPE atlas_tx,
                                               usampler2D tilemaps_tx,
                                               LightData light,
                                               vec3 P)
{
  ShadowSampleParams params = shadow_directional_sample_params_get(tilemaps_tx, light, P);

  float depth = shadow_read_depth(atlas_tx, tilemaps_tx, params);

  ShadowEvalResult result;
  result.light_visibilty = float(params.uv.z < depth * params.z_range);
  result.occluder_distance = shadow_linear_occluder_distance(light, true, params.lP, depth);
  return result;
}

ShadowEvalResult shadow_sample(const bool is_directional,
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
