#pragma BLENDER_REQUIRE(image_engine_lib.glsl)

void main()
{
  vec4 tex_color = vec4(1.0, 0.0, 1.0, 1.0);
  vec3 co = vec3(uv_image, 0.0);

#ifdef IMAGE_TILED
  if (node_tex_tile_lookup(co, imageTileArray, imageTileData)) {
    tex_color = texture(imageTileArray, co);
  }
#endif

#ifndef IMAGE_TILED
  tex_color = texture(imageTexture, uv_image);
#endif

  fragColor = image_engine_apply_parameters(
      tex_color, drawFlags, imgPremultiplied, shuffle, FAR_DISTANCE, NEAR_DISTANCE);
}