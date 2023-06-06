#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(image_engine_lib.glsl)

void main()
{
  vec3 image_pos = vec3(pos.xy * 0.5 + 0.5, Z_DEPTH_IMAGE) + vec3(tile_offset, 0.0);
  gl_Position = point_world_to_ndc(image_pos);
  uv_image = image_pos.xy;
}