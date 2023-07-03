
/**
 * Spatial ray reuse. Denoise raytrace result using ratio estimator.
 * Also add in temporal reuse.
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

#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_bxdf_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

/* Blue noise categorized into 4 sets of samples.
 * See "Stochastic all the things" presentation slide 32-37. */
#define RESOLVE_SAMPLES_COUNT 9

ivec2 resolve_sample_offsets_get(int index)
{
/* NOTE(Metal): For Apple silicon GPUs executing this particular shader, by default, memory read
 * pressure is high while ALU remains low. Packing the sample data into a smaller format balances
 * this trade-off by reducing local shader register pressure and expensive memory look-ups into
 * spilled local shader memory, resulting in an increase in performance of 20% for this shader. */
#ifdef GPU_METAL
#  define SAMPLE_STORAGE_TYPE uchar
#  define pack_sample(x, y) uchar(((uchar(x + 2)) << uchar(3)) + (uchar(y + 2)))
#  define unpack_sample(x) ivec2((char(x) >> 3) - 2, (char(x) & 7) - 2)
#else
#  define SAMPLE_STORAGE_TYPE ivec2
#  define pack_sample(x, y) SAMPLE_STORAGE_TYPE(x, y)
#  define unpack_sample(x) x
#endif

  /* Note: Reflection samples declared in function scope to avoid per-thread memory pressure on
   * tile-based GPUs e.g. Apple Silicon. */
  const SAMPLE_STORAGE_TYPE resolve_sample_offsets[RESOLVE_SAMPLES_COUNT * 4] =
      SAMPLE_STORAGE_TYPE[RESOLVE_SAMPLES_COUNT * 4](
          /* Set 1. */
          /* First Ring (2x2). */
          /* Second Ring (6x6). */
          pack_sample(0, 0),
          pack_sample(-1, 3),
          pack_sample(1, 3),
          pack_sample(-1, 1),
          pack_sample(3, 1),
          pack_sample(-2, 0),
          pack_sample(3, 0),
          pack_sample(2, -1),
          pack_sample(1, -2),
          /* Set 2. */
          /* First Ring (2x2). */
          pack_sample(1, 1),
          /* Second Ring (6x6). */
          pack_sample(-2, 3),
          pack_sample(3, 3),
          pack_sample(0, 2),
          pack_sample(2, 2),
          pack_sample(-2, -1),
          pack_sample(1, -1),
          pack_sample(0, -2),
          pack_sample(3, -2),
          /* Set 3. */
          /* First Ring (2x2). */
          pack_sample(0, 1),
          /* Second Ring (6x6). */
          pack_sample(0, 3),
          pack_sample(3, 2),
          pack_sample(-2, 1),
          pack_sample(2, 1),
          pack_sample(-1, 0),
          pack_sample(-2, -2),
          pack_sample(0, -1),
          pack_sample(2, -2),
          /* Set 4. */
          /* First Ring (2x2). */
          pack_sample(1, 0),
          /* Second Ring (6x6). */
          pack_sample(2, 3),
          pack_sample(-2, 2),
          pack_sample(-1, 2),
          pack_sample(1, 2),
          pack_sample(2, 0),
          pack_sample(-1, -1),
          pack_sample(3, -1),
          pack_sample(-1, -2));

  return unpack_sample(resolve_sample_offsets[index]);
}

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

void main()
{
  const uint tile_size = RAYTRACE_GROUP_SIZE;
  uvec2 tile_coord = unpackUvec2x16(tiles_coord_buf[gl_WorkGroupID.x]);
  /* TODO(fclem): This is wrong. This is dispatched at fullres. */
  ivec2 texel = ivec2(gl_LocalInvocationID.xy + tile_coord * tile_size);
  ivec2 texel_fullres = texel * raytrace_buf.resolution_scale + raytrace_buf.resolution_bias;

  if (any(greaterThan(texel, raytrace_buf.full_resolution))) {
    return;
  }

  if (raytrace_buf.skip_denoise) {
    imageStore(out_radiance_img, texel_fullres, imageLoad(ray_radiance_img, texel));
    return;
  }

  vec2 uv = (vec2(texel_fullres) + 0.5) * raytrace_buf.full_resolution_inv;
  vec3 V = transform_direction(ViewMatrixInverse, get_view_vector_from_screen_uv(uv));

  int sample_count = RESOLVE_SAMPLES_COUNT;

  /* Blue noise categorised into 4 sets of samples.
   * See "Stochastic all the things" presentation slide 32-37. */
  int sample_pool = int((gl_GlobalInvocationID.x & 1u) + (gl_GlobalInvocationID.y & 1u) * 2u);
  sample_pool = (sample_pool + raytrace_buf.pool_offset) % 4;
  int sample_id = sample_pool * RESOLVE_SAMPLES_COUNT;

#if defined(RAYTRACE_DIFFUSE)
  ClosureDiffuse closure;
#elif defined(RAYTRACE_REFRACT)
  ClosureRefraction closure;
#elif defined(RAYTRACE_REFLECT)
  ClosureReflection closure;
#else
#  error
#endif

  gbuffer_load_closure_data(gbuffer_closure_tx, texel_fullres, closure);

#if defined(RAYTRACE_REFRACT) || defined(RAYTRACE_REFLECT)
  if (closure.roughness == 1e-3) {
    sample_count = 1;
  }
#endif

  float hit_depth_mean = 0.0;
  vec3 rgb_mean = vec3(0.0);
  vec3 rgb_moment = vec3(0.0);
  vec3 radiance_accum = vec3(0.0);
  float weight_accum = 0.0;
  for (int i = 0; i < sample_count; i++, sample_id++) {
    ivec2 sample_texel = texel + resolve_sample_offsets_get(sample_id);

    vec4 ray_data = imageLoad(ray_data_img, sample_texel);
    float ray_time = imageLoad(ray_time_img, sample_texel).r;
    vec4 ray_radiance = imageLoad(ray_radiance_img, sample_texel);

    vec3 ray_direction = ray_data.xyz;
    float ray_pdf_inv = ray_data.w;
    /* Skip invalid pixels. */
    if (ray_pdf_inv == 0.0) {
      continue;
    }

    /* Slide 54. */
    /* TODO(fclem): Apparently, ratio estimator should be pdf_bsdf / pdf_ray. */
    float weight = bxdf_eval(closure, ray_direction, V) * ray_pdf_inv;

    radiance_accum += ray_radiance.rgb * weight;
    weight_accum += weight;

    hit_depth_mean += ray_radiance.a;
    rgb_mean += ray_radiance.rgb;
    rgb_moment += sqr(ray_radiance.rgb);
  }

  radiance_accum *= safe_rcp(weight_accum);

  imageStore(out_radiance_img, texel_fullres, vec4(radiance_accum, 0.0));
}