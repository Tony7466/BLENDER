
/**
 * Virtual shadowmapping: Page Clear.
 *
 * Equivalent to a framebuffer depth clear but only for pages pushed to the clear_page_buf.
 */

#pragma BLENDER_REQUIRE(common_math_lib.glsl)

void main()
{
  uvec2 page_co = unpackUvec2x16(clear_page_buf[gl_GlobalInvocationID.z]);
  uvec2 page_texel = page_co * pages_infos_buf.page_size + gl_GlobalInvocationID.xy;

  /* Clear to FLT_MAX instead of 1 so the far plane doesn't cast shadows onto farther objects. */
#ifdef GPU_METAL
  /* TODO(Metal): Determine if it is more efficient to clear shadow page via imageStore or backing
   * buffer update. */
  uint out_index = page_texel.x + page_texel.y * atlas_img.texture->get_width();
  atlas_buf[out_index] = floatBitsToUint(FLT_MAX);
#else
  imageStore(atlas_img, ivec2(page_texel), uvec4(floatBitsToUint(FLT_MAX)));
#endif
}
