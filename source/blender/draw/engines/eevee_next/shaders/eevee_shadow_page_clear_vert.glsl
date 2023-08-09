
/**
 * Virtual shadowmapping: Page Clear.
 *
 * Equivalent to a framebuffer depth clear but only for pages pushed to the clear_page_buf.
 */

#pragma BLENDER_REQUIRE(common_math_lib.glsl)

void main()
{
  int page = gl_VertexID / 3;
  uint page_packed = render_map_buf[page];
  uvec3 page_co = shadow_page_unpack(page_packed);

  gpu_Layer = int(page_co.z);
  gpu_ViewportIndex = int(page_co.x + page_co.y * 4);

  /* Fullscreen triangle. */
  int v = gl_VertexID % 3;
  float x = -1.0 + float((v & 1) << 2);
  float y = -1.0 + float((v & 2) << 1);
  gl_Position = vec4(x, y, 1.0, 1.0);
}
