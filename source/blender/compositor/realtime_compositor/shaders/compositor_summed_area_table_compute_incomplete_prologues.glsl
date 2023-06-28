#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

/* See the compute_incomplete_prologues function for a description of this shader. */
void main()
{
  /* Dispatches along the y axis corresponds to groups along the serial axis, while dispatches
   * along the x axis corresponds to individual threads along the parallel axis. */
  int parallel_axis_index = int(gl_GlobalInvocationID.x);
  int serial_axis_group_index = int(gl_GlobalInvocationID.y);

  /* Note that the first prologue is the result of summing a virtual block that is before the first
   * block and we assume that this prologue is all zeros, so we set the prologue to zero as well.
   * This is implemented by setting the accumulation length to zero for the first dispatch. */
  int accumulation_length = serial_axis_group_index == 0 ? 0 : int(gl_WorkGroupSize.x);

  vec4 x_accumulated_color = vec4(0.0);
  vec4 y_accumulated_color = vec4(0.0);
  for (int i = 0; i < accumulation_length; i++) {
    /* We subtract one because the first group is the virtual group as mentioned above. */
    int serial_axis_index = (serial_axis_group_index - 1) * int(gl_WorkGroupSize.x) + i;
    ivec2 x_read_texel = ivec2(serial_axis_index, parallel_axis_index);
    x_accumulated_color += texture_load(input_tx, x_read_texel, vec4(0.0));
    y_accumulated_color += texture_load(input_tx, x_read_texel.yx, vec4(0.0));
  }

  /* Store both prologues with the parallel axis aligned with the horizontal texture axis for
   * better cache locality. */
  ivec2 write_texel = ivec2(parallel_axis_index, serial_axis_group_index);
  imageStore(incomplete_x_prologues_img, write_texel, x_accumulated_color);
  imageStore(incomplete_y_prologues_img, write_texel, y_accumulated_color);
}
