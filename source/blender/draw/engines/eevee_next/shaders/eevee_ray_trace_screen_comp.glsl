
/**
 * Use Hierarchical-Z buffer to find intersection with the scene.
 */

#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_geom_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_bxdf_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ray_types_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_reflection_probe_lib.glsl)

void main()
{
  const uint tile_size = RAYTRACE_GROUP_SIZE;
  uvec2 tile_coord = unpackUvec2x16(tiles_coord_buf[gl_WorkGroupID.x]);
  ivec2 texel = ivec2(gl_LocalInvocationID.xy + tile_coord * tile_size);

  /* TODO */
  // ivec2 texel_fullres = texel * raytrace_buf.resolution_scale + raytrace_buf.resolution_bias;
  ivec2 texel_fullres = texel;

  bool valid_texel = in_texture_range(texel_fullres, depth_tx);
  float depth = (!valid_texel) ? 0.0 : texelFetch(depth_tx, texel_fullres, 0).r;
  vec2 uv = vec2(texel_fullres) / vec2(textureSize(depth_tx, 0).xy);

  vec4 ray_data = imageLoad(ray_data_img, texel);
  float pdf = ray_data.w;

  if (pdf == 0.0) {
    imageStore(ray_radiance_img, texel, vec4(0.0));
  }

  Ray ray;
  ray.origin = get_view_space_from_depth(uv, depth);
  ray.direction = ray_data.xyz;
  /* Extend the ray to cover the whole view. */

  vec3 radiance = vec3(0.0);
  vec2 noise_offset = sampling_rng_2D_get(SAMPLING_RAYTRACE_W);
  float rand_trace = interlieved_gradient_noise(vec2(texel), 5.0, noise_offset.x);

/* TODO */
#define DO_REFLECTION true
#define DO_REFRACTION false

  const bool discard_backface = DO_REFLECTION;
  const bool allow_self_intersection = DO_REFRACTION;
  /* TODO(fclem): Take IOR into account in the roughness LOD bias. */
  /* TODO(fclem): pdf to roughness mapping is a crude approximation. Find something better. */
  float roughness = saturate(sample_pdf_uniform_hemisphere() / pdf);
  bool hit = false;

  //   ray.direction *= 1e6;
  //   hit = raytrace_screen(raytrace_buf,
  //                         hiz_buf,
  //                         hiz_tx,
  //                         rand_trace,
  //                         roughness,
  //                         discard_backface,
  //                         allow_self_intersection,
  //                         ray);

  if (hit) {
    /* Evaluate radiance at hitpoint. */
    // vec2 hit_uv = get_uvs_from_view(ray.origin + ray.direction);

    // radiance = textureLod(radiance_tx, hit_uv, 0.0).rgb;

    /* Transmit twice if thickness is set and ray is longer than thickness. */
    // if (thickness > 0.0 && length(ray_data.xyz) > thickness) {
    //   ray_radiance.rgb *= color;
    // }
  }
  else {
    /* Fallback to nearest lightprobe. */
    // radiance = lightprobe_cubemap_eval(ray.origin, ray.direction, roughness, rand_probe);
    radiance = light_world_sample(ray.direction, 0.0);
  }

  /* Limit to the smallest non-0 value that the format can encode.
   * Strangely it does not correspond to the IEEE spec. */
  float inv_pdf = (pdf == 0.0) ? 0.0 : max(6e-8, 1.0 / pdf);
  /* Store inverse pdf to speedup denoising. */
  imageStore(ray_data_img, texel, vec4(ray.direction, inv_pdf));
  imageStore(ray_radiance_img, texel, vec4(radiance, 0.0));
}
