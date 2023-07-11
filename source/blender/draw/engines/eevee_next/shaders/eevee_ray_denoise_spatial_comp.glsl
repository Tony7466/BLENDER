
/**
 * Spatial ray reuse. Denoise raytrace result using ratio estimator.
 *
 * Input: Ray direction * hit time, Ray radiance, Ray hit depth
 * Ouput: Ray radiance reconstructed, Mean Ray hit depth, Radiance Variance
 *
 * Shader is specialized depending on the type of ray to denoise.
 *
 * Following "Stochastic All The Things: Raytracing in Hybrid Real-Time Rendering"
 * by Tomasz Stachowiak
 * https://www.ea.com/seed/news/seed-dd18-presentation-slides-raytracing
 */

#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_bxdf_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

void gbuffer_load_closure_data(sampler2DArray gbuffer_closure_tx,
                               ivec2 texel,
                               out ClosureDiffuse closure)
{
  vec4 data_in = texelFetch(gbuffer_closure_tx, ivec3(texel, 1), 0);

  closure.N = gbuffer_normal_unpack(data_in.xy);
}

void gbuffer_load_closure_data(sampler2DArray gbuffer_closure_tx,
                               ivec2 texel,
                               out ClosureRefraction closure)
{
  vec4 data_in = texelFetch(gbuffer_closure_tx, ivec3(texel, 1), 0);

  closure.N = gbuffer_normal_unpack(data_in.xy);
  if (gbuffer_is_refraction(data_in)) {
    /* NOTE: Roughness is squared here. */
    closure.roughness = max(1e-3, sqr(data_in.z));
    closure.ior = gbuffer_ior_unpack(data_in.w);
  }
  else {
    closure.roughness = 1.0;
    closure.ior = 1.1;
  }
}

void gbuffer_load_closure_data(sampler2DArray gbuffer_closure_tx,
                               ivec2 texel,
                               out ClosureReflection closure)
{
  vec4 data_in = texelFetch(gbuffer_closure_tx, ivec3(texel, 0), 0);

  closure.N = gbuffer_normal_unpack(data_in.xy);
  /* NOTE: Roughness is squared now. */
  closure.roughness = max(1e-3, sqr(data_in.z));
}

float bxdf_eval(ClosureDiffuse closure, vec3 L, vec3 V)
{
  return bsdf_lambert(closure.N, L);
}

float bxdf_eval(ClosureRefraction closure, vec3 L, vec3 V)
{
  return btdf_ggx(closure.N, L, V, closure.roughness, closure.ior);
}

float bxdf_eval(ClosureReflection closure, vec3 L, vec3 V)
{
  return bsdf_ggx(closure.N, L, V, closure.roughness);
}

#if defined(RAYTRACE_DIFFUSE)
#  define ClosureT ClosureDiffuse
#  define CLOSURE_ACTIVE eClosureBits(CLOSURE_REFLECTION)
#elif defined(RAYTRACE_REFRACT)
#  define ClosureT ClosureRefraction
#  define CLOSURE_ACTIVE eClosureBits(CLOSURE_REFLECTION)
#elif defined(RAYTRACE_REFLECT)
#  define ClosureT ClosureReflection
#  define CLOSURE_ACTIVE eClosureBits(CLOSURE_REFLECTION)
#else
#  error
#endif

void resolve_sample(ivec2 sample_texel,
                    ClosureT closure,
                    vec3 V,
                    inout float closest_hit_time,
                    inout vec3 radiance_accum,
                    inout float weight_accum,
                    inout vec3 rgb_moment)
{
  vec4 ray_data = imageLoad(ray_data_img, sample_texel);
  float ray_time = imageLoad(ray_time_img, sample_texel).r;
  vec4 ray_radiance = imageLoad(ray_radiance_img, sample_texel);

  vec3 ray_direction = ray_data.xyz;
  float ray_pdf_inv = ray_data.w;
  /* Skip invalid pixels. */
  if (ray_pdf_inv == 0.0) {
    return;
  }

  closest_hit_time = min(closest_hit_time, ray_time);

  /* Slide 54. */
  /* TODO(fclem): Apparently, ratio estimator should be pdf_bsdf / pdf_ray. */
  float weight = bxdf_eval(closure, ray_direction, V) * ray_pdf_inv;

  radiance_accum += ray_radiance.rgb * weight;
  weight_accum += weight;

  rgb_moment += sqr(ray_radiance.rgb) * weight;
}

void main()
{
  const uint tile_size = RAYTRACE_GROUP_SIZE;
  uvec2 tile_coord = unpackUvec2x16(tiles_coord_buf[gl_WorkGroupID.x]);
  /* TODO(fclem): This is wrong. This is dispatched at fullres. */
  ivec2 texel = ivec2(gl_LocalInvocationID.xy + tile_coord * tile_size);
  ivec2 texel_fullres = texel * raytrace_buf.resolution_scale + raytrace_buf.resolution_bias;

  if (raytrace_buf.skip_denoise) {
    imageStore(out_radiance_img, texel_fullres, imageLoad(ray_radiance_img, texel));
    return;
  }

  /* Clear neighbor tiles that will not be processed. */
  /* TODO(fclem): Optimize this. We don't need to clear the whole ring. This adds a ~8% cost  */
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      if (x == 0 && y == 0) {
        continue;
      }

      ivec2 tile_coord_neighbor = ivec2(tile_coord) + ivec2(x, y);
      if (!in_image_range(tile_coord_neighbor, tile_mask_img)) {
        continue;
      }

      bool tile_is_unused = imageLoad(tile_mask_img, tile_coord_neighbor).r == 0;
      if (tile_is_unused) {
        ivec2 texel_fullres_neighbor = texel_fullres + ivec2(x, y) * int(tile_size);

        imageStore(out_radiance_img, texel_fullres_neighbor, vec4(0.0));
        imageStore(out_variance_img, texel_fullres_neighbor, vec4(0.0));
        imageStore(out_hit_depth_img, texel_fullres_neighbor, vec4(0.0));
      }
    }
  }

  bool valid_texel = in_texture_range(texel_fullres, stencil_tx);
  uint closure_bits = (!valid_texel) ? 0u : texelFetch(stencil_tx, texel_fullres, 0).r;
  if (!flag_test(closure_bits, CLOSURE_ACTIVE)) {
    imageStore(out_radiance_img, texel_fullres, vec4(0.0));
    imageStore(out_variance_img, texel_fullres, vec4(0.0));
    imageStore(out_hit_depth_img, texel_fullres, vec4(0.0));
    return;
  }

  vec2 uv = (vec2(texel_fullres) + 0.5) * raytrace_buf.full_resolution_inv;
  vec3 V = transform_direction(ViewMatrixInverse, get_view_vector_from_screen_uv(uv));

  ClosureT closure;
  gbuffer_load_closure_data(gbuffer_closure_tx, texel_fullres, closure);

  uint sample_count = 16u;
  float filter_size = 9.0;
#if defined(RAYTRACE_REFRACT) || defined(RAYTRACE_REFLECT)
  float filter_size_factor = saturate((closure.roughness - 1e-3) * 8.0);
  sample_count = 1u + uint(12.0 * filter_size_factor + 0.5);
  filter_size = (sample_count == 1u) ? 0.0 : (3.0 + 8.0 * filter_size_factor);
  filter_size *= float(raytrace_buf.resolution_scale);
#endif

  vec2 noise = utility_tx_fetch(utility_tx, vec2(texel_fullres), UTIL_BLUE_NOISE_LAYER).ba;
  noise += sampling_rng_1D_get(SAMPLING_CLOSURE);

  vec3 rgb_moment = vec3(0.0);
  vec3 radiance_accum = vec3(0.0);
  float weight_accum = 0.0;
  float closest_hit_time = 1.0e10;
  /* TODO(fclem) sample center sample */
  for (uint i = 0u; i < sample_count; i++) {
    ivec2 offset = ivec2((fract(hammersley_2d(i, sample_count) + noise) - 0.5) * filter_size);
    ivec2 sample_texel = texel + offset;

    resolve_sample(
        sample_texel, closure, V, closest_hit_time, radiance_accum, weight_accum, rgb_moment);
  }
  float inv_weight = safe_rcp(weight_accum);

  radiance_accum *= inv_weight;
  /* Use radiance sum as signal mean. */
  vec3 rgb_mean = radiance_accum;
  rgb_moment *= inv_weight;

  vec3 rgb_variance = abs(rgb_moment - sqr(rgb_mean));
  float hit_variance = max_v3(rgb_variance);

  float scene_z = get_view_z_from_depth(texelFetch(hiz_tx, texel_fullres, 0).r);
  float hit_depth = get_depth_from_view_z(scene_z - closest_hit_time);

  imageStore(out_radiance_img, texel_fullres, vec4(radiance_accum, 0.0));
  imageStore(out_variance_img, texel_fullres, vec4(hit_variance));
  imageStore(out_hit_depth_img, texel_fullres, vec4(hit_depth));
}