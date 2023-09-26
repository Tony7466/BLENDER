
/**
 * Virtual Shadow map accumulation storage.
 *
 * On Apple silicon, we can use a three-pass method to perform virtual shadow map updates,
 * leveraging efficient use of tile-based GPUs. Shadow updates rasterize geometry for each view in
 * much the same way as a conventional shadow map render, but for the standard path, there is an
 * additional cost of an atomic-min abd store to allow for indirection into the atlas. This setup
 * can lead to excessive overdraw, rasterization and increased complexity in the material depth
 * fragment shader, reducing rendering performance.
 *
 * On a tile-based GPU, as shadow updates are still relative, we can still leverage on-tile depth
 * testing, to avoid atomic-min operations against global memory, and only write out the final
 * depth value stored in each tile. Large memory-less render targets are used to create a virtual
 * render target, where only the updated regions and layers are processed.
 *
 * Firstly, invoke an instance of this shader with PASS_CLEAR to clear the depth values to default
 * for tiles being updated. The first optimization also enables tiles which are not being updated
 * in this pass to be cleared to zero, saving on fragment invocation costs for unused regions of
 * the render target.
 * This allows us to also remove the compute-based tile clear pass in Metal.
 *
 * Secondly, eevee_surf_shadow_frag is used to generate depth information which is stored
 * on tile for the closest fragment. The Metal path uses a simple variant of this shader
 * which just outputs the depth, without any virtual shadow map processing on top.
 *
 * The third pass then runs, writing out only the highest-level final pixel to memory,
 * avoiding the requirement for atomic texture operations.
 *
 * Output shadow atlas page indirection is calculated in the vertex shader, which generates
 * a lattice of quads covering the shadow pages which are to be updated. The quads which
 * belong to shadow pages not being updated in this pass are discarded.
 **/

#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_shadow_tilemap_lib.glsl)

void main()
{
#ifdef PASS_CLEAR
  /* The tile clear pass writes out to tile attachment to ensure raster order groups are satisfied,
   * allowing the clear to be guaranteed to happen first, as it is first in submission order.   */
  out_tile_depth = FLT_MAX;
#endif

#ifdef PASS_DEPTH_STORE
  /* Write result to altas. As tiles outside of valid update regions are discarded, we can use the
   * NOTE: As this shader is only used in Metal, we can use the fastest possible write function
   * without any parameter wrapping or conversion.*/
  
  /* For Metal accumulation pass, we store the result from depth in tile memory. */
  uint u_depth = floatBitsToUint(in_tile_depth);

  /* Quantization bias. Equivalent to nextafter in C without all the safety. 1 is not enough. */
  u_depth += 2;
  shadow_atlas_img.texture->write(u_depth, ushort2(out_texel_xy), out_page_z);
#endif
}
