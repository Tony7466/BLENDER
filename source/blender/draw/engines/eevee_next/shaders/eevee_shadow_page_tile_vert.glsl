
/**
 * Virtual Shadow map tile shader.
 *
 * See fragment shader for more infos.
 */
#pragma BLENDER_REQUIRE(eevee_shadow_tilemap_lib.glsl)

void main()
{
  int tile_id = gl_InstanceID;
  /* Generate Quad with 2 triangle with same winding.
   * This way the can be merged on some hardware. */
  int v = (gl_VertexID > 2) ? (3 - (gl_VertexID - 3)) : gl_VertexID;
  vec2 tile_corner = vec2(v & 1, v >> 1);

#ifdef PASS_DEPTH_STORE
  /* Load where fragment should write the tile data. */
  uvec3 dst_page_co = dst_coord_buf[tile_id];
  /* Interpolate output texel  */
  out_texel_xy = (vec2(dst_page_co.xy) + tile_corner) * vec2(SHADOW_PAGE_RES);
  out_page_z = dst_page_co.z;
#endif

  /* Load where the quad should be positioned. */
  uvec3 src_page_co = src_coord_buf[tile_id];

  vec2 uv_pos = (tile_corner + vec2(src_page_co.xy)) / float(SHADOW_TILEMAP_RES);
  vec2 ndc_pos = uv_pos * 2.0 - 1.0;
  /* We initially clear depth to 1.0 only for update fragments.
   * Non-updated tile depth will remain at 0.0 to ensure fragments are discarded. */
  gl_Position = vec4(ndc_pos.x, ndc_pos.y, 1.0, 1.0);
  gpu_Layer = int(src_page_co.z);
  /* Assumes last viewport will always cover the whole framebuffer. */
  gpu_ViewportIndex = 15;
}
