#pragma BLENDER_REQUIRE(common_view_lib.glsl)

#define Z_DEPTH_IMAGE 0.75

void main()
{
  vec3 image_pos = vec3((pos.xy + vec2(1.0)) * vec2(0.5), Z_DEPTH_IMAGE);
  gl_Position = point_world_to_ndc(image_pos + vec3(tile_offset, 0.0));
  uv_image = image_pos.xy;
}