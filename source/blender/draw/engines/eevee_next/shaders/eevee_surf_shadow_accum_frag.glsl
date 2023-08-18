
/**
 * Virtual Shadow map accumulation storage.
 *
 * On Apple silicon, we can use a two-pass method to store generate shadow depth data.
 * Firstly, eevee_surf_shadow_frag is used to generate depth information which is stored
 * on tile for the closest fragment.
 *
 * This second pass then runs, writing out only the highest-level final pixel to memory,
 * avoiding the requirement for atomic texture operations.
 *
 * Output shadow atlas page indirection is calculated in the vertex shader, which generates
 * a lattice of quads covering the virtual regions which are to be updated.
 **/

#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_shadow_tilemap_lib.glsl)

void main()
{
  /* For Metal accumulation pass, we store the result from depth in tile memory. */
  uint u_depth = floatBitsToUint(in_tile_depth);

  /* Quantization bias. Equivalent to nextafter in C without all the safety. 1 is not enough. */
  u_depth += 2;

  /* Write result to altas. As tiles outside of valid update regions are discarded, we can use the
   * fast variant of store function which does not perform bounds checking.*/
  imageStoreFast(shadow_atlas_img,
                 ivec3(ivec2(out_texel_xy), out_page_z),
                 uvec4(u_depth));
}
