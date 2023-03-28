
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

void main()
{
  /* Constant array moved inside function scope.
   * Minimises local register allocation in MSL. */
  const vec2 pos[6] = vec2[6](vec2(-1.0, -1.0),
                              vec2(1.0, -1.0),
                              vec2(-1.0, 1.0),

                              vec2(1.0, -1.0),
                              vec2(1.0, 1.0),
                              vec2(-1.0, 1.0));

  lP = pos[gl_VertexID % 6];
  cell_index = gl_VertexID / 6;

  /* Keep in sync with update_irradiance_probe. */
  ivec3 cell = ivec3(cell_index / (grid_resolution.z * grid_resolution.y),
                     (cell_index / grid_resolution.z) % grid_resolution.y,
                     cell_index % grid_resolution.z);

  vec3 ls_cell_pos = (vec3(cell) + vec3(0.5)) / vec3(grid_resolution);
  ls_cell_pos = ls_cell_pos * 2.0 - 1.0; /* Remap to (-1 ... +1). */

  vec3 ws_cell_pos = (grid_to_world * vec4(ls_cell_pos, 1.0)).xyz;

  vec3 vs_offset = vec3(lP, 0.0) * sphere_radius;
  vec3 vP = (ViewMatrix * vec4(ws_cell_pos, 1.0)).xyz + vs_offset;

  gl_Position = ProjectionMatrix * vec4(vP, 1.0);
  gl_Position.z += 0.0001; /* Small bias to let the icon draw without zfighting. */
}
