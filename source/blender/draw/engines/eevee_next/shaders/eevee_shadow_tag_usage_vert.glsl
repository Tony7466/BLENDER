
/**
 * Virtual shadowmapping: Usage tagging
 *
 * Shadow pages are only allocated if they are visible.
 * This renders bounding boxes for transparent objects in order to tag the correct shadows.
 */

#pragma BLENDER_REQUIRE(common_view_lib.glsl)

void main()
{
  PASS_RESOURCE_ID

  ObjectBounds bounds = bounds_buf[resource_id];

  interp.P = bounds.bounding_corners[0].xyz;
  interp.P += bounds.bounding_corners[1].xyz * max(0, pos.x);
  interp.P += bounds.bounding_corners[2].xyz * max(0, pos.y);
  interp.P += bounds.bounding_corners[3].xyz * max(0, pos.z);
  interp.vP = point_world_to_view(interp.P);

  gl_Position = point_world_to_ndc(interp.P);
}
