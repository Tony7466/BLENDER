#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

void main()
{
  /* Each invocation corresponds to one output pixel, where the output has half the size of the
   * input. */
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);

  /* Add 0.5 to evaluate the input sampler at the center of the pixel and divide by the image size
   * to get the coordinates into the sampler's expected [0, 1] range. */
  vec2 coordinates = (vec2(texel) + vec2(0.5)) / vec2(imageSize(output_img));

  /* Each invocation downsamples four pixels of the input into one output pixel, but instead of
   * sampling at the center of each of the pixels, we sample at the meeting point of each four
   * pixels surrounding the output pixel, which when sampled using bilinear interpolation would
   * produce an average, producing the desired small blur that we desire.
   *
   * This can be illustrated in the following diagram, which shows the boundaries of the input
   * pixels. The diagram also shows the center of one of the output pixels, denoted by the symbol
   * o, which actually occupies the space of the four pixels around it, since the output is half
   * the size of the input. Instead of sampling the aforementioned four pixels at their center, we
   * sample at the locations denoted by the symbol x, which averages their surrounding four pixels
   * due to linear interpolation.
   *
   *      +---+---+---+---+
   *      |   |   |   |   |
   *      +---+---x---+---+
   *      |   |   |   |   |
   *      +---x---o---x---+
   *      |   |   |   |   |
   *      +---+---x---+---+
   *      |   |   |   |   |
   *      +---+---+---+---+
   */
  vec4 pixels[4];
  vec2 offset = 1.0 / vec2(texture_size(input_tx));
  pixels[0] = texture(input_tx, coordinates + vec2(-offset.x, -offset.y));
  pixels[1] = texture(input_tx, coordinates + vec2(offset.x, -offset.y));
  pixels[2] = texture(input_tx, coordinates + vec2(offset.x, offset.y));
  pixels[3] = texture(input_tx, coordinates + vec2(-offset.x, offset.y));

/* Downsampling is done by computing a weighted average of each of the four samples. */
#if defined(BOX_FILTER)
  /* A simple box filter where each sample gets the same weight. */
  vec4 weights = vec4(1.0);
#elif defined(KARIS_FILTER)
  /* Reduce the contributions of fireflies on the result by applying a form of local tone mapping
   * as described by Brian Karis in the article "Graphic Rants: Tone Mapping".
   *
   *   https://graphicrants.blogspot.com/2013/12/tone-mapping.html
   *
   * This needn't be applied on all downsampling passes, but only the first one, since fireflies
   * will not survive the first pass, later passes can use the simpler box filter. */
  vec4 brightness = vec4(
      max_v3(pixels[0]), max_v3(pixels[1]), max_v3(pixels[2]), max_v3(pixels[3]));
  vec4 weights = 1.0 / (brightness + 1.0);
#endif

  vec4 result = vec4(0.0);
  for (int i = 0; i < 4; i++) {
    result += pixels[i] * weights[i];
  }
  result /= sum(weights);

  imageStore(output_img, texel, result);
}
