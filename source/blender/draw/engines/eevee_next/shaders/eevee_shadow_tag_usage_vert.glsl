
/**
 * Virtual shadowmapping: Usage tagging
 *
 * Shadow pages are only allocated if they are visible.
 * This renders bounding boxes for transparent objects in order to tag the correct shadows.
 */

#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_intersect_lib.glsl)

/* Inflate bounds by half a pixel as a conservative rasterization alternative */
void inflate_bounds(vec3 ls_center, inout vec3 P, inout vec3 lP)
{
  vec3 vP = point_world_to_view(P);

  float inflate_scale = pixel_world_radius * pow(2, fb_lod);
  bool is_persp = (ProjectionMatrix[3][3] == 0.0);
  if (is_persp) {
    inflate_scale *= -vP.z;
  }
  inflate_scale *= 0.5f; /* Half pixel. */

  vec3 vs_inflate_vector = normal_object_to_view(sign(lP - ls_center));
  vs_inflate_vector.z = 0;
  vs_inflate_vector /= max_v2(abs(vs_inflate_vector));
  vs_inflate_vector *= inflate_scale;

  vP += vs_inflate_vector;
  P = point_view_to_world(vP);
  lP = point_world_to_object(P);
}

void main()
{
  PASS_RESOURCE_ID

  const ObjectBounds bounds = bounds_buf[resource_id];

  /* TODO (Miguel Pozo):
   *This could compute the box planes if the compiler doesn't optimize it out.*/
  IsectBox box = isect_data_setup(bounds.bounding_corners[0].xyz,
                                  bounds.bounding_corners[1].xyz,
                                  bounds.bounding_corners[2].xyz,
                                  bounds.bounding_corners[3].xyz);

  vec3 ws_aabb_min = bounds.bounding_corners[0].xyz;
  vec3 ws_aabb_max = bounds.bounding_corners[0].xyz + bounds.bounding_corners[1].xyz +
                     bounds.bounding_corners[2].xyz + bounds.bounding_corners[3].xyz;

  vec3 ls_center = point_world_to_object((ws_aabb_min + ws_aabb_max) / 2.0f);

  vec3 ls_conservative_min = vec3(FLT_MAX);
  vec3 ls_conservative_max = vec3(-FLT_MAX);

  for (int i = 0; i < 8; i++) {
    vec3 P = box.corners[i];
    vec3 lP = point_world_to_object(P);
    inflate_bounds(ls_center, P, lP);

    ls_conservative_min = min(ls_conservative_min, lP);
    ls_conservative_max = max(ls_conservative_max, lP);
  }

#ifdef DEBUG_CONSERVATIVE_RASTERIZATION
  interp.ls_aabb_min = point_world_to_object(ws_aabb_min);
  interp.ls_aabb_max = point_world_to_object(ws_aabb_max);
#else
  interp.ls_aabb_min = ls_conservative_min;
  interp.ls_aabb_max = ls_conservative_max;
#endif

  vec3 lP = mix(ls_conservative_min, ls_conservative_max, max(vec3(0), pos));

  interp.P = point_object_to_world(lP);
  interp.vP = point_world_to_view(interp.P);

  gl_Position = point_world_to_ndc(interp.P);
}
