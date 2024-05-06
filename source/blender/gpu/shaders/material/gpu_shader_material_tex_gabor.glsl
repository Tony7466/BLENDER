/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_common_hash.glsl)

#define SHD_GABOR_TYPE_1D 0.0
#define SHD_GABOR_TYPE_2D 1.0
#define SHD_GABOR_TYPE_3D 2.0
#define SHD_GABOR_TYPE_4D 3.0
#define SHD_GABOR_TYPE_SURFACE 4.0

#define M_PI 3.14159265358979323846 /* pi */

float square(float v)
{
  return v * v;
}

int poisson(vec2 cell, float mean)
{
  int k = 0;
  float t = hash_vec3_to_float(vec3(cell, k));
  float threshold = exp(-mean);
  while (t > threshold) {
    k++;
    t *= hash_vec3_to_float(vec3(cell, k));
  }
  return k;
}

float compute_gabor_kernel(float frequency, float orientation, vec2 position)
{
  float gaussian_envelop = exp(-M_PI * dot(position, position));
  float sinusoidal_wave = cos(2.0 * M_PI * frequency *
                              ((position.x * cos(orientation)) + (position.y * sin(orientation))));
  return gaussian_envelop * sinusoidal_wave;
}

float compute_2d_gabor_noise_cell(vec2 cell,
                                  vec2 position,
                                  float impulse_density,
                                  float kernel_radius,
                                  float orientation,
                                  float frequency,
                                  float isotropy)

{
  float number_of_impulses_per_cell = impulse_density * square(kernel_radius);
  int number_of_impulses = poisson(cell, number_of_impulses_per_cell);
  float noise = 0.0;
  for (int i = 0; i < number_of_impulses; ++i) {
    float weight = hash_vec3_to_float(vec3(cell, i * 2));
    float orientation_sector = orientation +
                               hash_vec3_to_float(vec3(cell, i * 2 + 1)) * 2.0 * M_PI * isotropy;
    vec2 kernel_center = position - hash_vec3_to_vec2(vec3(cell, i));
    if (dot(kernel_center, kernel_center) < 1.0) {
      noise += weight *
               compute_gabor_kernel(frequency, orientation_sector, kernel_center * kernel_radius);
    }
  }
  return noise;
}

float variance(float frequency, float impulse_density)
{
  float integral_gabor_filter_squared = (1.0 / 4.0) *
                                        (1.0 + exp(-(2.0 * M_PI * square(frequency))));
  return impulse_density * (1.0 / 3.0) * integral_gabor_filter_squared;
}

float compute_2d_gabor_noise(vec2 coordinates,
                             float number_of_impulses_per_kernel,
                             float orientation,
                             float frequency,
                             float isotropy)
{
  float kernel_radius = sqrt(-log(0.05) / M_PI);
  float impulse_density = number_of_impulses_per_kernel / (M_PI * square(kernel_radius));

  vec2 scaled_coordinates = coordinates / kernel_radius;

  vec2 cell_position = floor(scaled_coordinates);
  vec2 local_position = scaled_coordinates - cell_position;

  float sum = 0.0;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      vec2 cell_offset = vec2(i, j);
      vec2 current_cell_position = cell_position + cell_offset;
      vec2 intra_cell_position = local_position - cell_offset;
      sum += compute_2d_gabor_noise_cell(current_cell_position,
                                         intra_cell_position,
                                         impulse_density,
                                         kernel_radius,
                                         orientation,
                                         frequency,
                                         isotropy);
    }
  }

  float scale = 3.0 * sqrt(variance(frequency, impulse_density));
  return (sum / scale) * 0.5 + 0.5;
}

void node_tex_gabor(vec3 coordinates,
                    float scale,
                    float impulses,
                    float orientation,
                    float frequency,
                    float anisotropy,
                    float type,
                    out float output_value)
{
  vec3 scaled_coordinates = coordinates * scale;
  float isotropy = 1.0 - clamp(anisotropy, 0.0, 1.0);

  if (type == SHD_GABOR_TYPE_1D) {
    /* Not yet implemented. */
    output_value = 0.0;
  }
  else if (type == SHD_GABOR_TYPE_2D) {
    output_value = compute_2d_gabor_noise(
        scaled_coordinates.xy, impulses, orientation, frequency, isotropy);
  }
  else if (type == SHD_GABOR_TYPE_3D) {
    /* Not yet implemented. */
    output_value = 0.0;
  }
  else if (type == SHD_GABOR_TYPE_4D) {
    /* Not yet implemented. */
    output_value = 0.0;
  }
  else if (type == SHD_GABOR_TYPE_SURFACE) {
    /* Not yet implemented. */
    output_value = 0.0;
  }
}
