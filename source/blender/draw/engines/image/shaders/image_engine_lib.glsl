#define Z_DEPTH_IMAGE 0.75
#define Z_DEPTH_BORDER 1.0

/* Keep in sync with image_engine.c */
#define IMAGE_DRAW_FLAG_SHOW_ALPHA (1 << 0)
#define IMAGE_DRAW_FLAG_APPLY_ALPHA (1 << 1)
#define IMAGE_DRAW_FLAG_SHUFFLING (1 << 2)
#define IMAGE_DRAW_FLAG_DEPTH (1 << 3)

#define FAR_DISTANCE farNearDistances.x
#define NEAR_DISTANCE farNearDistances.y

vec4 image_engine_apply_parameters(vec4 color,
                                   int flags,
                                   bool premultiplied,
                                   vec4 shuffle_color,
                                   float far_distance,
                                   float near_distance)
{
  vec4 result = color;
  if ((flags & IMAGE_DRAW_FLAG_APPLY_ALPHA) != 0) {
    if (!premultiplied) {
      result.rgb *= result.a;
    }
  }
  if ((flags & IMAGE_DRAW_FLAG_DEPTH) != 0) {
    result = smoothstep(far_distance, near_distance, result);
  }

  if ((flags & IMAGE_DRAW_FLAG_SHUFFLING) != 0) {
    result = vec4(dot(result, shuffle_color));
  }
  if ((flags & IMAGE_DRAW_FLAG_SHOW_ALPHA) == 0) {
    result.a = 1.0;
  }
  return result;
}

bool node_tex_tile_lookup(inout vec3 co, sampler2DArray ima, sampler1DArray map)
{
  vec2 tile_pos = floor(co.xy);

  if (tile_pos.x < 0 || tile_pos.y < 0 || tile_pos.x >= 10) {
    return false;
  }

  float tile = 10.0 * tile_pos.y + tile_pos.x;
  if (tile >= textureSize(map, 0).x) {
    return false;
  }

  /* Fetch tile information. */
  float tile_layer = texelFetch(map, ivec2(tile, 0), 0).x;
  if (tile_layer < 0.0) {
    return false;
  }

  vec4 tile_info = texelFetch(map, ivec2(tile, 1), 0);

  co = vec3(((co.xy - tile_pos) * tile_info.zw) + tile_info.xy, tile_layer);
  return true;
}
