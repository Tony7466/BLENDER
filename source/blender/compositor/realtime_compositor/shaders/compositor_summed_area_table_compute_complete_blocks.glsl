#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

/* An intermediate shared memory where the result of X accumulation will be stored. */
shared vec4 block[gl_WorkGroupSize.x][gl_WorkGroupSize.y];

void main()
{
  int parallel_index = int(gl_GlobalInvocationID.x);
  int serial_group_index = int(gl_GlobalInvocationID.y);

  /* Accumulate the block along the horizontal direction starting from the X prologue value,
   * writing each accumulation step to the intermediate shared memory. */
  ivec2 prologue_texel = ivec2(parallel_index, serial_group_index);
  vec4 x_accumulated_color = texture_load(complete_x_prologues_tx, prologue_texel, vec4(0.0));
  int group_size = int(gl_WorkGroupSize.x);
  for (int i = 0; i < group_size; i++) {
    int serial_index = serial_group_index * group_size + i;
    x_accumulated_color += texture_load(input_tx, ivec2(serial_index, parallel_index), vec4(0.0));
    // block[i][gl_LocalInvocationID.x] = x_accumulated_color;
    imageStore(output_img, ivec2(serial_index, parallel_index), x_accumulated_color);
  }

  /* Make sure the result of X accumulation is completely done. */
  barrier();
  memoryBarrierImage();

  /* Accumulate the block along the vertical direction starting from the Y prologue value,
   * writing each accumulation step to the output image. */
  vec4 y_accumulated_color = texture_load(complete_y_prologues_tx, prologue_texel);
  for (int i = 0; i < group_size; i++) {
    int serial_index = serial_group_index * group_size + i;
    // y_accumulated_color += block[gl_LocalInvocationID.x][i];
    y_accumulated_color += imageLoad(output_img, ivec2(parallel_index, serial_index));
    imageStore(output_img, ivec2(parallel_index, serial_index), y_accumulated_color);
  }
}
