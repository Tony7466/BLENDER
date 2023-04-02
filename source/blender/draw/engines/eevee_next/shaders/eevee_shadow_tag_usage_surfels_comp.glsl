
/**
 * Virtual shadowmapping: Usage tagging
 *
 * Shadow pages are only allocated if they are visible.
 * This pass iterates the surfels buffer and tag all tiles that are needed for light shadowing as
 * needed.
 */

#pragma BLENDER_REQUIRE(eevee_shadow_tag_usage_lib.glsl)

void main()
{
  Surfel surfel = surfels_buf[gl_GlobalInvocationID.x];
  shadow_tag_usage_surfel(surfel, directional_level);
}
