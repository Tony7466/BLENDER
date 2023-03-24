
/**
 * Takes scene surfel representation and build list of surfels aligning in a given direction.
 *
 * The lists head are allocated to fit the surfel granularity.
 *
 * Due to alignment the link and list head are split into several int arrays to avoid too much
 * memory waste.
 *
 * Dispatch 1 thread per surfel.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)

void main()
{
  int surfel_index = int(gl_GlobalInvocationID);
  if (surfel_index >= capture_info_buf.surfel_len) {
    return;
  }

  vec4 hP = point_world_to_ndc(surfel_buf[surfel_index].position);

  surfel_buf[surfel_index].ray_distance = -hP.z;

  vec2 ssP_surfel = hP.xy * 0.5 + 0.5;
  ivec2 ray_coord_on_grid = clamp(ivec2(ssP_surfel * vec2(list_info_buf.ray_grid_size)),
                                  ivec2(0),
                                  list_info_buf.ray_grid_size - 1);
  int list_index = ray_coord_on_grid.y * list_info_buf.ray_grid_size.x + ray_coord_on_grid.x;

  /* NOTE: We only need to init the `list_start_buf` to -1 for the whole list to be valid since
   * every surfel will load its `next` value from the list head. */
  surfel_buf[surfel_index].next = atomicExchange(list_start_buf[list_index], surfel_index);
}
