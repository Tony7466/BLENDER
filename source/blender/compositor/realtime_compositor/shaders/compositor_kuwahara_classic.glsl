#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);

  vec4 sum_of_luminance_of_quadrants = vec4(0.0);
  vec4 sum_of_squared_luminance_of_quadrants = vec4(0.0);
  vec4 sum_of_color_of_quadrants[4] = vec4[](vec4(0.0), vec4(0.0), vec4(0.0), vec4(0.0));

  /* Compute the above statistics for each of the quadrants around the current pixel. */
  for (int q = 0; q < 4; q++) {
    for (int j = 0; j <= radius; j++) {
      for (int i = 0; i <= radius; i++) {
        /* The sign is just a fancy expression for computing the sign of the quadrant q. */
        ivec2 sign = ivec2((q % 2) * 2 - 1, ((q / 2) * 2 - 1));
        ivec2 offset = ivec2(i, j) * sign;

        vec4 color = texture_load(input_tx, texel + offset);
        float luminance = dot(color.rgb, luminance_coefficients);

        sum_of_color_of_quadrants[q] += color;
        sum_of_luminance_of_quadrants[q] += luminance;
        sum_of_squared_luminance_of_quadrants[q] += luminance * luminance;
      }
    }
  }

  /* Find the quadrant which has the minimum variance. */
  float minimum_variance = FLT_MAX;
  int quadrant_with_minimum_variance = 0;
  int quadrant_pixel_count = (radius + 1) * (radius + 1);
  for (int q = 0; q < 4; q++) {
    float luminance_mean = sum_of_luminance_of_quadrants[q] / quadrant_pixel_count;
    float squared_luminance_mean = sum_of_squared_luminance_of_quadrants[q] / quadrant_pixel_count;
    float luminance_variance = squared_luminance_mean - luminance_mean * luminance_mean;

    if (luminance_variance < minimum_variance) {
      quadrant_with_minimum_variance = q;
      minimum_variance = luminance_variance;
    }
  }

  vec4 sum_of_color_of_chosen_quadrant = sum_of_color_of_quadrants[quadrant_with_minimum_variance];
  vec4 mean_color_of_chosen_quadrant = sum_of_color_of_chosen_quadrant / quadrant_pixel_count;

  /* Write the mean color of the quadrant that was chosen. */
  imageStore(output_img, texel, mean_color_of_chosen_quadrant);
}
