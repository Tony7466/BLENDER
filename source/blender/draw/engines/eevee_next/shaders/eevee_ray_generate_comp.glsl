/**
 * Generate Ray direction along with other data that are then used
 * by the next pass to trace the scene.
 */

#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ray_generate_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)

void main()
{
  const uint tile_size = RAYTRACE_GROUP_SIZE;
  uvec2 tile_coord = unpackUvec2x16(tiles_coord_buf[gl_WorkGroupID.x]);
  ivec2 texel = ivec2(gl_LocalInvocationID.xy + tile_coord * tile_size);

  vec4 p = imageLoad(test_img, texel);
  imageStore(test_img, texel, p * vec4(1.0, 0.0, 0.0, 1.0));
#if 0 
  ivec2 texel_fullres = texel * raytrace_buf.resolution_scale + raytrace_buf.resolution_bias;

  bool valid_texel = in_texture_range(texel_fullres, stencil_tx);
  uint local_closure_bits = (!valid_texel) ? 0u : texelFetch(stencil_tx, texel_fullres, 0).r;
  /* TODO(fclem): We could use wave ops instead of a shared variables. */
  atomicOr(closures_bits, local_closure_bits);
  barrier();

  if (flag_test(closures_bits, CLOSURE_DIFFUSE | CLOSURE_REFRACTION | CLOSURE_REFLECTION)) {
    /* Generate ray. */
    vec2 uv = vec2(texel_fullres) / vec2(textureSize(depth_tx, 0).xy);
    float depth = 0.0;
    if (valid_texel) {
      depth = texelFetch(depth_tx, texel_fullres, 0).r;
    }
    vec3 P = get_world_space_from_depth(uv, depth);
    vec3 V = cameraVec(P);
    vec4 noise = utility_tx_fetch(utility_tx, texel, UTIL_BLUE_NOISE_LAYER).gbar;

    if (flag_test(local_closure_bits, CLOSURE_DIFFUSE | CLOSURE_REFRACTION)) {
      vec4 col_in = vec4(0.0); /* UNUSED */
      vec4 tra_nor_in = texelFetch(gbuf_transmit_normal_tx, texel_fullres, 0);
      vec4 tra_dat_in = texelFetch(gbuf_transmit_data_tx, texel_fullres, 0);

      bool is_refraction = (tra_nor_in.z == -1.0);
      if (is_refraction) {
        ClosureRefraction closure = gbuffer_load_refraction_data(col_in, tra_nor_in, tra_dat_in);
        vec4 ray;
        ray.xyz = raytrace_refraction_direction(sampling_buf, noise.xy, closure, V, ray.w);
        imageStore(out_ray_data_refract, texel, ray);
      }
      else {
        ClosureDiffuse closure = gbuffer_load_diffuse_data(col_in, tra_nor_in, tra_dat_in);
        vec4 ray;
        ray.xyz = raytrace_diffuse_direction(sampling_buf, noise.xy, closure, ray.w);
        imageStore(out_ray_data_diffuse, texel, ray);
      }
    }
    else {
      imageStore(out_ray_data_diffuse, texel, vec4(0.0));
      imageStore(out_ray_data_refract, texel, vec4(0.0));
    }

    if (flag_test(local_closure_bits, CLOSURE_REFLECTION)) {
      vec4 col_in = vec4(0.0); /* UNUSED */
      vec4 ref_nor_in = texelFetch(gbuf_reflection_normal_tx, texel_fullres, 0);

      ClosureReflection closure = gbuffer_load_reflection_data(col_in, ref_nor_in);
      vec4 ray;
      ray.xyz = raytrace_reflection_direction(sampling_buf, noise.xy, closure, V, ray.w);
      imageStore(out_ray_data_reflect, texel, ray);
    }
    else {
      imageStore(out_ray_data_reflect, texel, vec4(0.0));
    }
  }
#endif
}
