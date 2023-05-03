#pragma BLENDER_REQUIRE(common_view_clipping_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

void main()
{
    interp.color.rgb = vec3(1, 0, 1);
    interp.color.a = 0.5f;
    vec3 world_pos = point_object_to_world(pos);
    gl_Position = point_world_to_ndc(world_pos);
    /* view_clipping_distances(world_pos); */
}