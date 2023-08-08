
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

void write_depth(ivec2 texel_co, float depth)
{
  uint page_packed = render_map_buf[shadow_interp.view_id];
  ivec3 page = ivec3(shadow_page_unpack(page_packed));
  ivec2 out_texel = page.xy * pages_infos_buf.page_size + texel_co;

  uint u_depth = floatBitsToUint(depth);
  /* Quantization bias. Equivalent to nextafter in C without all the safety. 1 is not enough. */
  u_depth += 2;

  /* TOOD(Metal): For Metal, textures will need to be viewed as buffers to workaround missing image
   * atomics support. */
  imageAtomicMin(shadow_atlas_img, ivec3(out_texel, page.z), u_depth);
}

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

  drw_view_id = shadow_interp.view_id;

  ivec2 texel_co = ivec2(gl_FragCoord.xy);

  float depth = gl_FragCoord.z;
  float slope_bias = fwidth(depth);
  write_depth(texel_co, depth + slope_bias);
}
