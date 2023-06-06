#pragma BLENDER_REQUIRE(common_colormanagement_lib.glsl)
#pragma BLENDER_REQUIRE(image_engine_lib.glsl)

bool is_border(vec2 uv)
{
  return (uv.x < min_max_uv.x || uv.y < min_max_uv.y || uv.x >= min_max_uv.z ||
          uv.y >= min_max_uv.w);
}

void main()
{
  bool border = is_border(uv_image);
  gl_FragDepth = border ? Z_DEPTH_BORDER : Z_DEPTH_IMAGE;
}
