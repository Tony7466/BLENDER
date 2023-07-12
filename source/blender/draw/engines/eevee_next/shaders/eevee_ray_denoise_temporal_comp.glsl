
/**
 * Temporal Reprojection and accumulation of denoised raytraced radiance.
 *
 * Dispatched at fullres using a tile list.
 *
 * Input: Spatialy denoised radiance, Variance, Hit depth
 * Ouput: Stabilized Radiance, Stabilized Variance
 *
 * Following "Stochastic All The Things: Raytracing in Hybrid Real-Time Rendering"
 * by Tomasz Stachowiak
 * https://www.ea.com/seed/news/seed-dd18-presentation-slides-raytracing
 */

#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_matrix_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_lib.glsl)

struct LocalStatistics {
  vec3 mean;
  vec3 moment;
  vec3 variance;
  vec3 deviation;
  vec3 clamp_min;
  vec3 clamp_max;
};

LocalStatistics local_statistics_get(ivec2 texel, vec3 center_radiance)
{
  /* Build Local statistics (slide 46). */
  LocalStatistics result;
  result.mean = center_radiance;
  result.variance = center_radiance;
  float weight_accum = 1.0;

  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      if (x == 0 && y == 0) {
        continue;
      }

      ivec2 neighbor_texel = texel + ivec2(x, y);
      if (!in_image_range(neighbor_texel, in_radiance_img)) {
        continue;
      }
      vec3 radiance = imageLoad(in_radiance_img, neighbor_texel).rgb;
      /* Exclude unprocessed pixels. */
      if (all(equal(radiance, FLT_11_11_10_MAX))) {
        continue;
      }

      /* Weight corners less to avoid box artifacts.
       * Same idea as in "High Quality Temporal Supersampling" by Brian Karis at Siggraph 2014
       * (Slide 32) Simple clamp to min/max of 8 neighbors results in 3x3 box artifacts. */
      float weight = (x == y) ? 0.25 : 1.0;
      result.mean += radiance.rgb;
      result.moment += square_f(radiance.rgb);
      weight_accum += 1.0;
    }
  }
  float inv_weight = safe_rcp(weight_accum);
  result.mean *= inv_weight;
  result.moment *= inv_weight;
  result.variance = abs(result.moment - square_f(result.mean));
  result.deviation = sqrt(result.variance);
  result.clamp_min = result.mean - result.deviation;
  result.clamp_max = result.mean + result.deviation;
  return result;
}

vec4 radiance_history_sample(vec3 P, LocalStatistics local)
{
  vec2 uv = project_point(raytrace_buf.history_persmat, P).xy * 0.5 + 0.5;

  if (!in_range_exclusive(uv, vec2(0.0), vec2(1.0))) {
    /* Out of history view. Return sample without weight. */
    return vec4(0.0);
  }

  vec3 history_radiance = texture(radiance_history_tx, uv).rgb;
  /* Exclude unprocessed pixels. */
  /* TODO(fclem): this doesn't work with bilinear filtering. */
  if (all(equal(history_radiance, FLT_11_11_10_MAX))) {
    return vec4(0.0);
  }

  /* Weighted contribution (slide 46). */
  vec3 dist = abs(history_radiance - local.mean) / local.deviation;
  float weight = exp2(-10.0 * dot(dist, vec3(1.0 / 3.0)));

  ivec2 history_texel = ivec2(floor(uv * vec2(textureSize(radiance_history_tx, 0).xy)));
  ivec2 history_tile = history_texel / RAYTRACE_GROUP_SIZE;
  /* Fetch previous tilemask to avoid loading invalid data. */
  bool is_valid_history = texelFetch(tilemask_history_tx, history_tile, 0).r != 0;
  /* TODO(fclem): this doesn't work with bilinear filtering. */
  weight = is_valid_history ? weight : 0.0;

  /* Clamp resulting history radiance (slide 47). */
  history_radiance = clamp(history_radiance, local.clamp_min, local.clamp_max);

  return vec4(history_radiance * weight, weight);
}

vec2 variance_history_sample(vec3 P)
{
  vec2 uv = project_point(raytrace_buf.history_persmat, P).xy * 0.5 + 0.5;

  if (!in_range_exclusive(uv, vec2(0.0), vec2(1.0))) {
    /* Out of history view. Return sample without weight. */
    return vec2(0.0);
  }

  float history_variance = texture(variance_history_tx, uv).r;

  ivec2 history_texel = ivec2(floor(uv * vec2(textureSize(variance_history_tx, 0).xy)));
  ivec2 history_tile = history_texel / RAYTRACE_GROUP_SIZE;
  /* Fetch previous tilemask to avoid loading invalid data. */
  bool is_valid_history = texelFetch(tilemask_history_tx, history_tile, 0).r != 0;

  if (is_valid_history) {
    return vec2(history_variance, 1.0);
  }
  return vec2(0.0);
}

void main()
{
  const uint tile_size = RAYTRACE_GROUP_SIZE;
  uvec2 tile_coord = unpackUvec2x16(tiles_coord_buf[gl_WorkGroupID.x]);
  ivec2 texel_fullres = ivec2(gl_LocalInvocationID.xy + tile_coord * tile_size);
  vec2 uv = (vec2(texel_fullres) + 0.5) * raytrace_buf.full_resolution_inv;

  float in_variance = imageLoad(in_variance_img, texel_fullres).r;
  vec3 in_radiance = imageLoad(in_radiance_img, texel_fullres).rgb;

  if (all(equal(in_radiance, FLT_11_11_10_MAX))) {
    /* Early out on pixels that were marked unprocessed by the previous pass. */
    imageStore(out_radiance_img, texel_fullres, vec4(FLT_11_11_10_MAX, 0.0));
    imageStore(out_variance_img, texel_fullres, vec4(0.0));
    return;
  }

  LocalStatistics local = local_statistics_get(texel_fullres, in_radiance);

  /* Radiance. */

  /* Surface reprojection. */
  float scene_depth = texelFetch(hiz_tx, texel_fullres, 0).r;
  vec3 P = get_world_space_from_depth(uv, scene_depth);
  vec4 history_radiance = radiance_history_sample(P, local);
  /* Reflection reprojection. */
  float hit_depth = imageLoad(hit_depth_img, texel_fullres).r;
  vec3 P_hit = get_world_space_from_depth(uv, hit_depth);
  history_radiance += radiance_history_sample(P_hit, local);
  /* Finalize accumulation. */
  history_radiance *= safe_rcp(history_radiance.w);
  /* Blend history with new radiance. */
  float mix_fac = (history_radiance.w == 0.0) ? 0.0 : 0.97;
  /* Reduce blend factor to improve low rougness reflections. Use variance instead of roughness. */
  mix_fac *= mix(0.75, 1.0, saturate(in_variance / 1e-3));
  vec3 out_radiance = mix(safe_color(in_radiance), safe_color(history_radiance.rgb), mix_fac);
  /* This is feedback next frame as radiance_history_tx. */
  imageStore(out_radiance_img, texel_fullres, vec4(out_radiance, 0.0));

  /* Variance. */

  /* Reflection reprojection. */
  vec2 history_variance = variance_history_sample(P_hit);
  /* Blend history with new variance. */
  float mix_variance_fac = (history_variance.y == 0.0) ? 0.0 : 0.90;
  float out_variance = mix(in_variance, history_variance.x, mix_variance_fac);
  /* This is feedback next frame as variance_history_tx. */
  imageStore(out_variance_img, texel_fullres, vec4(out_variance));
}