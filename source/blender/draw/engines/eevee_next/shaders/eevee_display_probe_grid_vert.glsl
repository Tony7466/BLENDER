
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_lightprobe_lib.glsl)

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
  int cell_index = gl_VertexID / 6;

  ivec3 grid_resolution = textureSize(irradiance_a_tx, 0);

  cell = ivec3(cell_index / (grid_resolution.z * grid_resolution.y),
               (cell_index / grid_resolution.z) % grid_resolution.y,
               cell_index % grid_resolution.z);

  vec3 ws_cell_pos = lightprobe_irradiance_grid_sample_position(
      grid_to_world, grid_resolution, cell);

  vec3 vs_offset = vec3(lP, 0.0) * sphere_radius;
  vec3 vP = (ViewMatrix * vec4(ws_cell_pos, 1.0)).xyz + vs_offset;

  gl_Position = ProjectionMatrix * vec4(vP, 1.0);
  /* Small bias to let the icon draw without zfighting. */
  gl_Position.z += 0.0001;
}
