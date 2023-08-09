
/**
 * Virtual Shadow map output.
 *
 * Meshes are rasterize onto an empty framebuffer. Each generated fragment then checks which
 * virtual page it is supposed to go and load the physical page adress.
 * If a physical page exists, we then use atomicMin to mimic a less-than depth test and write to
 * the destination texel.
 **/

#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_attributes_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_surf_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_nodetree_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_transparency_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)

void main()
{
#ifdef MAT_TRANSPARENT
  init_globals();

  nodetree_surface();

  float noise_offset = sampling_rng_1D_get(SAMPLING_TRANSPARENCY);
  float random_threshold = transparency_hashed_alpha_threshold(1.0, noise_offset, g_data.P);

  float transparency = avg(g_transmittance);
  if (transparency > random_threshold) {
    discard;
    return;
  }
#endif
}
