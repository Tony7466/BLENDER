/**
 * This pass load Gbuffer data and output a mask of tiles to process.
 * This mask is then processed by the compaction phase.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)

shared uint tile_contains_glossy_rays;
shared uint tile_contains_refract_rays;

/* Returns a blend factor between different irradiance fetching method for reflections. */
float ray_glossy_factor(float roughness)
{
  /* TODO */
  return 1.0;
}

void main()
{
  if (all(equal(gl_LocalInvocationID, uvec3(0)))) {
    /* Buffer is cleared to 0 from CPU command so that we can use it as counter.
     * Note that these writes are subject to race condition, but we write the same
     * value from all workgroups. */
    /* TODO(fclem): Move this to tile compaction. */
    dispatch_reflect_buf.num_groups_y = dispatch_reflect_buf.num_groups_z = 1u;

    /* Init shared variables. */
    tile_contains_glossy_rays = 0;
    tile_contains_refract_rays = 0;
  }

  barrier();

  ivec2 texel = min(ivec2(gl_GlobalInvocationID.xy), textureSize(stencil_tx, 0) - 1);

  eClosureBits closure_bits = eClosureBits(texelFetch(stencil_tx, texel, 0).r);

  if (flag_test(closure_bits, CLOSURE_REFLECTION)) {
    vec4 gbuffer_0_packed = texelFetch(gbuffer_closure_tx, ivec3(texel, 0), 0);

    ClosureReflection reflection_data;
    reflection_data.roughness = gbuffer_0_packed.z;

    if (ray_glossy_factor(reflection_data.roughness) > 0.0) {
      /* We don't care about race condition here. */
      tile_contains_glossy_rays = 1;
    }
  }

  if (flag_test(closure_bits, CLOSURE_REFLECTION)) {
    vec4 gbuffer_1_packed = texelFetch(gbuffer_closure_tx, ivec3(texel, 1), 0);

    ClosureRefraction refraction_data;
    refraction_data.roughness = gbuffer_1_packed.z;

    if (ray_glossy_factor(refraction_data.roughness) > 0.0) {
      /* We don't care about race condition here. */
      tile_contains_refract_rays = 1;
    }
  }

  barrier();

  if (all(equal(gl_LocalInvocationID, uvec3(0)))) {
    ivec2 tile_co = ivec2(gl_WorkGroupID.xy);

    uint tile_mask = 0u;
    if (tile_contains_glossy_rays > 0) {
      tile_mask |= CLOSURE_REFLECTION;

      /* TODO(fclem): Move this to tile compaction. */
      uint tile_index = atomicAdd(dispatch_reflect_buf.num_groups_x, 1u);
      tiles_reflect_buf[tile_index] = packUvec2x16(gl_WorkGroupID.xy);
    }

    if (tile_contains_refract_rays > 0) {
      tile_mask |= CLOSURE_REFRACTION;

      /* TODO(fclem): Move this to tile compaction. */
      uint tile_index = atomicAdd(dispatch_refract_buf.num_groups_x, 1u);
      tiles_refract_buf[tile_index] = packUvec2x16(gl_WorkGroupID.xy);
    }

    imageStore(tile_mask_img, tile_co, uvec4(tile_mask));
  }
}