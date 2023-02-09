
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
  vec2 screen_uv = gl_FragCoord.xy / vec2(fb_resolution);

  float opaque_depth = texelFetch(hiz_tx, int2(gl_FragCoord.xy), fb_lod).r;
  vec3 ws_opaque = get_world_space_from_depth(screen_uv, opaque_depth);

  vec3 ws_near_plane = get_world_space_from_depth(screen_uv, 0);
  vec3 ws_view_direction = normalize(interp.P - ws_near_plane);
  vec3 vs_near_plane = get_view_space_from_depth(screen_uv, 0);
  vec3 vs_view_direction = normalize(interp.vP - vs_near_plane);
  vec3 ls_near_plane = point_world_to_object(ws_near_plane);
  vec3 ls_view_direction = normalize(point_world_to_object(interp.P) - ls_near_plane);

  float ls_near_box_t = ray_aabb(
      ls_near_plane, ls_view_direction, interp.ls_aabb_min, interp.ls_aabb_max);
  vec3 ls_near_box = ls_near_plane + ls_view_direction * ls_near_box_t;
  vec3 ws_near_box = point_object_to_world(ls_near_box);

  float near_box_t = distance(ws_near_plane, ws_near_box);
  float far_box_t = distance(ws_near_plane, interp.P);
  /* Depth test. */
  far_box_t = min(far_box_t, distance(ws_near_plane, ws_opaque));

  float step_size = 0.1f;
  for (float t = near_box_t; t <= far_box_t; t += step_size) {
    /* Ensure we don't get past far_box_t. */
    t = min(t, far_box_t);

    vec3 P = ws_near_plane + (ws_view_direction * t);
    vec3 vP = vs_near_plane + (vs_view_direction * t);
    /* TODO (Miguel Pozo): Pass step size to ensure conservative enough LOD selection */
    shadow_tag_usage(vP, P, gl_FragCoord.xy * pow(2, fb_lod));

    /* Ensure that step_size is as large as possible,
     * but (hopefully) not larger than the smallest possible page size. */
    step_size = pixel_world_radius * SHADOW_PAGE_RES * 0.5;
    bool is_persp = (ProjectionMatrix[3][3] == 0.0);
    if (is_persp) {
      step_size *= max(0.01f, t);
    }
  }

  outDebug.rgb = ws_near_plane + (ws_view_direction * near_box_t);
  outDebug.a = 1.0f;
}
