#pragma BLENDER_REQUIRE(common_colormanagement_lib.glsl)
#pragma BLENDER_REQUIRE(image_engine_lib.glsl)

void main()
{
  ivec2 uvs_clamped = ivec2(uv_screen);
  float depth = texelFetch(depth_texture, uvs_clamped, 0).r;
  if (depth == 1.0) {
    discard;
    return;
  }

  vec4 tex_color = texelFetch(imageTexture, uvs_clamped - offset, 0);
  fragColor = image_engine_apply_parameters(
      tex_color, drawFlags, imgPremultiplied, shuffle, FAR_DISTANCE, NEAR_DISTANCE);
}
