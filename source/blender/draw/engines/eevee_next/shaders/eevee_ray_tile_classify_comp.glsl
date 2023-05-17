/**
 * This pass load Gbuffer data and output a mask of tiles to process.
 * This mask is then processed by the compaction phase.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)

shared uint tile_contains_glossy_rays;

/* Returns a blend factor between different irradiance fetching method for reflections. */
float ray_glossy_factor(float roughness)
{
  /* TODO */
  return 1.0;
}

void main()
{
  if (all(equal(gl_LocalInvocationID, uvec3(0)))) {
    tile_contains_glossy_rays = 0;
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

  barrier();

  if (all(equal(gl_LocalInvocationID, uvec3(0)))) {
    ivec2 tile_co = ivec2(gl_WorkGroupID.xy);

    uint tile_mask = 0u;
    if (tile_contains_glossy_rays > 0) {
      tile_mask |= CLOSURE_REFLECTION;
    }

    imageStore(tile_mask_img, tile_co, uvec4(tile_mask));
  }
}