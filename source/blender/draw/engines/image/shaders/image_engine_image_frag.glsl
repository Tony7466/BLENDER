#pragma BLENDER_REQUIRE(image_engine_lib.glsl)

void main()
{
  vec4 tex_color = texture(imageTexture, uv_image, 0);
  fragColor = image_engine_apply_parameters(
      tex_color, drawFlags, imgPremultiplied, shuffle, FAR_DISTANCE, NEAR_DISTANCE);
}