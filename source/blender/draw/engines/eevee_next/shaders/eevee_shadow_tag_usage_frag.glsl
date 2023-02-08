
/**
 * Virtual shadowmapping: Usage tagging
 *
 * Shadow pages are only allocated if they are visible.
 * This pass scan the depth buffer and tag all tiles that are needed for light shadowing as
 * needed.
 */

#pragma BLENDER_REQUIRE(eevee_shadow_tag_usage_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

float ray_aabb(vec3 ray_origin, vec3 ray_direction, vec3 aabb_min, vec3 aabb_max)
{
  /* https://gdbooks.gitbooks.io/3dcollisions/content/Chapter3/raycast_aabb.html */
  vec3 t_mins = (aabb_min - ray_origin) / ray_direction;
  vec3 t_maxs = (aabb_max - ray_origin) / ray_direction;

  float t_min = max_v3(min(t_mins, t_maxs));
  float t_max = min_v3(max(t_mins, t_maxs));

  /* AABB is in the opposite direction. */
  if (t_max < 0.0f) {
    return -1.0f;
  }
  /* No intersection. */
  if (t_min > t_max) {
    return -1.0f;
  }
  /* The ray origin is inside the aabb. */
  if (t_min < 0.0f) {
    /* For regular ray casting we would return t_max here,
     * but we want to ray cast against the box volume, not just the surface. */
    return 0.0f;
  }
  return t_min;
}

void main()
{
  vec2 screen_uv = gl_FragCoord.xy / vec2(textureSize(depth_tx, 0).xy);

  float opaque_depth = texelFetch(depth_tx, int2(gl_FragCoord.xy), 0).r;
  vec3 opaque_ws = get_world_space_from_depth(screen_uv, opaque_depth);

  vec3 near_plane_ws = get_world_space_from_depth(screen_uv, 0);
  vec3 view_direction_ws = normalize(interp.P - near_plane_ws);
  vec3 near_plane_vs = get_view_space_from_depth(screen_uv, 0);
  vec3 view_direction_vs = normalize(interp.vP - near_plane_vs);
  vec3 near_plane_os = point_world_to_object(near_plane_ws);
  vec3 view_direction_os = normalize(point_world_to_object(interp.P) - near_plane_os);

  float near_box_t_os = ray_aabb(
      near_plane_os, view_direction_os, interp.aabb_min_os, interp.aabb_max_os);
  vec3 near_box_os = near_plane_os + view_direction_os * near_box_t_os;
  vec3 near_box_ws = point_object_to_world(near_box_os);

  float near_box_t = distance(near_plane_ws, near_box_ws);
  float far_box_t = distance(near_plane_ws, interp.P);
  /* Depth test. */
  far_box_t = min(far_box_t, distance(near_plane_ws, opaque_ws));

  float step_size = 0.1f;
  for (float t = near_box_t; t <= far_box_t; t += step_size) {
    /* Ensure we don't get past far_box_t. */
    t = min(t, far_box_t);

    vec3 P = near_plane_ws + (view_direction_ws * t);
    vec3 vP = near_plane_vs + (view_direction_vs * t);
    shadow_tag_usage(vP, P, gl_FragCoord.xy);
  }

  outDebug.rgb = near_plane_ws + (view_direction_ws * near_box_t);
  outDebug.a = 1.0f;
}
