/**
 * Generate Ray direction along with other data that are then used
 * by the next pass to trace the scene.
 */

#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ray_generate_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)

shared uint closures_bits;

void main()
{
  const uint tile_size = RAYTRACE_GROUP_SIZE;
  uvec2 tile_coord = unpackUvec2x16(tiles_coord_buf[gl_WorkGroupID.x]);
  ivec2 texel = ivec2(gl_LocalInvocationID.xy + tile_coord * tile_size);

  /* TODO */
  // ivec2 texel_fullres = texel * raytrace_buf.resolution_scale + raytrace_buf.resolution_bias;
  ivec2 texel_fullres = texel;

  bool valid_texel = in_texture_range(texel_fullres, stencil_tx);
  uint local_closure_bits = (!valid_texel) ? 0u : texelFetch(stencil_tx, texel_fullres, 0).r;

  /* TODO(fclem): We could use wave ops instead of a shared variables. */
  if (all(equal(gl_LocalInvocationID.xy, uvec2(0)))) {
    closures_bits = 0u;
  }
  barrier();
  atomicOr(closures_bits, local_closure_bits);
  barrier();

  if (!flag_test(closures_bits, CLOSURE_DIFFUSE | CLOSURE_REFRACTION | CLOSURE_REFLECTION)) {
    return;
  }

  /* Generate ray. */
  vec2 uv = vec2(texel_fullres) / vec2(textureSize(depth_tx, 0).xy);
  float depth = 0.0;
  if (valid_texel) {
    depth = texelFetch(depth_tx, texel_fullres, 0).r;
  }
  vec3 P = get_world_space_from_depth(uv, depth);
  vec3 V = cameraVec(P);
  vec2 noise = utility_tx_fetch(utility_tx, texel, UTIL_BLUE_NOISE_LAYER).rg;

  if ((local_closure_bits & eClosureBits(active_closure_type)) == 0) {
    imageStore(out_ray_data_img, texel, vec4(0.0));
    return;
  }

  if (active_closure_type == CLOSURE_REFLECTION) {
    vec4 gbuffer_0_packed = texelFetch(gbuffer_closure_tx, ivec3(texel, 0), 0);

    ClosureReflection closure;
    closure.N = gbuffer_normal_unpack(gbuffer_0_packed.xy);
    closure.roughness = gbuffer_0_packed.z;

    vec4 ray;
    ray.xyz = raytrace_reflection_direction(sampling_buf, noise.xy, closure, V, ray.w);
    imageStore(out_ray_data_img, texel, ray);
  }
  else {
    /* CLOSURE_REFRACTION */
    vec4 gbuffer_1_packed = texelFetch(gbuffer_closure_tx, ivec3(texel, 1), 0);
    if (gbuffer_is_refraction(gbuffer_1_packed)) {
      ClosureRefraction closure;
      closure.N = gbuffer_normal_unpack(gbuffer_1_packed.xy);
      closure.ior = gbuffer_1_packed.z;
      closure.roughness = gbuffer_ior_pack(gbuffer_1_packed.w);

      vec4 ray;
      ray.xyz = raytrace_refraction_direction(sampling_buf, noise.xy, closure, V, ray.w);
      imageStore(out_ray_data_img, texel, ray);
    }
    else {
      imageStore(out_ray_data_img, texel, vec4(0.0));
    }
  }
}
