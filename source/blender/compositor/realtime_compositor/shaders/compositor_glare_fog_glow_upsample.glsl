#pragma BLENDER_REQUIRE(gpu_shader_bicubic_sampler_lib.glsl)

void main()
{
  /* Each invocation corresponds to one output pixel, where the output has twice the size of the
   * input. */
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);

  /* Add 0.5 to evaluate the input sampler at the center of the pixel and divide by the image size
   * to get the coordinates into the sampler's expected [0, 1] range. */
  vec2 coordinates = (vec2(texel) + vec2(0.5)) / vec2(imageSize(output_img));

  /* Sample the input using bi-cubic interpolation as a form of smooth upsampling. */
  vec4 upsampled = texture_bicubic(input_tx, coordinates);

  imageStore(output_img, texel, imageLoad(output_img, texel) + upsampled);
}
