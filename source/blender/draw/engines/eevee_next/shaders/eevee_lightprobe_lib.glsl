
vec3 lightprobe_irradiance_grid_sample_position(mat4 grid_local_to_world,
                                                ivec3 grid_resolution,
                                                ivec3 cell_coord)
{
  vec3 ls_cell_pos = (vec3(cell_coord) + vec3(0.5)) / vec3(grid_resolution);
  ls_cell_pos = ls_cell_pos * 2.0 - 1.0;
  vec3 ws_cell_pos = (grid_local_to_world * vec4(ls_cell_pos, 1.0)).xyz;
  return ws_cell_pos;
}
