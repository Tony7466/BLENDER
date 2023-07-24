
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

/**
 * Return the coresponding list unclamped coordinate for a given world position.
 * Also return the surfel sorting value as `r_ray_distance`.
 */
ivec2 surfel_list_coord_get(vec3 P, out float r_ray_distance)
{
  vec4 hP = point_world_to_ndc(P);
  r_ray_distance = -hP.z;
  vec2 ssP = hP.xy * 0.5 + 0.5;
  ivec2 ray_coord_on_grid = ivec2(ssP * vec2(list_info_buf.ray_grid_size));
  ray_coord_on_grid = clamp(ray_coord_on_grid, ivec2(0), list_info_buf.ray_grid_size - 1);
  return ray_coord_on_grid;
}

/**
 * Variant of `surfel_list_index_get` but with input the list coordinate.
 */
int surfel_list_index_get(ivec2 list_coord_on_grid, int lod)
{
  lod = clamp(lod, 0, list_info_buf.ray_grid_lod_max);
  ivec2 lod_size = list_info_buf.ray_grid_size;
  int lod_offset = 0;
  for (int i = 0; i < lod; i++) {
    lod_offset += lod_size.x * lod_size.y;
    lod_size = lod_size >> 1;
  }
  list_coord_on_grid = clamp(list_coord_on_grid, ivec2(0), lod_size - 1);
  int list_index = list_coord_on_grid.y * lod_size.x + list_coord_on_grid.x + lod_offset;
  return list_index;
}

/**
 * Return the coresponding list index in the `list_start_buf` for a given world position.
 * It will clamp any coordinate outside valid bounds to nearest list.
 * Also return the surfel sorting value as `r_ray_distance`.
 */
int surfel_list_index_get(vec3 P, int lod, out float r_ray_distance)
{
  return surfel_list_index_get(surfel_list_coord_get(P, r_ray_distance), lod);
}
