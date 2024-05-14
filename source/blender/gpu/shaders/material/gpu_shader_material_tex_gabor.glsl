/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_common_hash.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)

#define SHD_GABOR_TYPE_2D 1.0
#define SHD_GABOR_TYPE_3D 2.0

float compute_2d_gabor_kernel(vec2 position, float frequency, float orientation)
{
  float distance_squared = length_squared(position);
  if (distance_squared >= 1.0) {
    return 0.0;
  }

  float hann_window = 0.5 + 0.5 * cos(M_PI * distance_squared);
  float gaussian_envelop = exp(-M_PI * distance_squared);
  float windowed_gaussian_envelope = gaussian_envelop * hann_window;

  vec2 frequency_vector = frequency * vec2(cos(orientation), sin(orientation));
  float sinusoidal_wave = sin(2.0 * M_PI * dot(position, frequency_vector));

  return windowed_gaussian_envelope * sinusoidal_wave;
}

int compute_impulses_count_for_2d_cell(vec2 cell, float impulses_count)
{
  return int(impulses_count) + (hash_vec2_to_float(cell) < fract(impulses_count) ? 1 : 0);
}

float compute_2d_gabor_noise_cell(vec2 cell,
                                  vec2 position,
                                  float impulses_count,
                                  float frequency,
                                  float isotropy,
                                  float base_orientation)

{
  int impulses_count_for_cell = compute_impulses_count_for_2d_cell(cell, impulses_count);

  float noise = 0.0;
  for (int i = 0; i < impulses_count_for_cell; ++i) {
    float random_orientation = hash_vec3_to_float(vec3(cell, i * 2 + 1)) * 2.0 * M_PI;
    float orientation = base_orientation + random_orientation * isotropy;
    vec2 kernel_center = position - hash_vec3_to_vec2(vec3(cell, i));
    float weight = hash_vec3_to_float(vec3(cell, i * 2)) < 0.5 ? -1.0 : 1.0;
    noise += weight * compute_2d_gabor_kernel(kernel_center, frequency, orientation);
  }
  return noise;
}

float compute_2d_gabor_noise(vec2 coordinates,
                             float impulses_count,
                             float frequency,
                             float isotropy,
                             float base_orientation)
{
  vec2 cell_position = floor(coordinates);
  vec2 local_position = coordinates - cell_position;

  float sum = 0.0;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      vec2 cell_offset = vec2(i, j);
      vec2 current_cell_position = cell_position + cell_offset;
      vec2 intra_cell_position = local_position - cell_offset;
      sum += compute_2d_gabor_noise_cell(current_cell_position,
                                         intra_cell_position,
                                         impulses_count,
                                         frequency,
                                         isotropy,
                                         base_orientation);
    }
  }

  return sum;
}

float compute_3d_gabor_kernel(vec3 position, float frequency, vec3 orientation)
{
  float distance_squared = length_squared(position);
  if (distance_squared >= 1.0) {
    return 0.0;
  }

  float hann_window = 0.5 + 0.5 * cos(M_PI * distance_squared);
  float gaussian_envelop = exp(-M_PI * distance_squared);
  float windowed_gaussian_envelope = gaussian_envelop * hann_window;

  vec3 frequency_vector = frequency * orientation;
  float sinusoidal_wave = sin(2.0 * M_PI * dot(position, frequency_vector));

  return windowed_gaussian_envelope * sinusoidal_wave;
}

int compute_impulses_count_for_3d_cell(vec3 cell, float impulses_count)
{
  return int(impulses_count) + (hash_vec3_to_float(cell) < fract(impulses_count) ? 1 : 0);
}

vec3 compute_3d_orientation(vec3 cell, int seed, vec3 orientation, float isotropy)
{
  if (isotropy == 0.0) {
    return orientation;
  }

  float inclination = acos(orientation.z);
  float azimuth = sign(orientation.y) * acos(orientation.x / length(orientation.xy));
  vec2 random_angles = hash_vec4_to_vec2(vec4(cell, seed)) * 2.0 * M_PI;
  inclination += random_angles.x * isotropy;
  azimuth += random_angles.y * isotropy;
  return vec3(sin(inclination) * cos(azimuth), sin(inclination) * sin(azimuth), cos(inclination));
}

float compute_3d_gabor_noise_cell(vec3 cell,
                                  vec3 position,
                                  float impulses_count,
                                  float frequency,
                                  float isotropy,
                                  vec3 base_orientation)

{
  int impulses_count_for_cell = compute_impulses_count_for_3d_cell(cell, impulses_count);

  float noise = 0.0;
  for (int i = 0; i < impulses_count_for_cell; ++i) {
    vec3 orientation = compute_3d_orientation(cell, i * 2 + 1, base_orientation, isotropy);
    vec3 kernel_center = position - hash_vec4_to_vec3(vec4(cell, i));
    float weight = hash_vec4_to_float(vec4(cell, i * 2)) < 0.5 ? -1.0 : 1.0;
    noise += weight * compute_3d_gabor_kernel(kernel_center, frequency, orientation);
  }
  return noise;
}

float compute_3d_gabor_noise(
    vec3 coordinates, float impulses_count, float frequency, float isotropy, vec3 base_orientation)
{
  vec3 cell_position = floor(coordinates);
  vec3 local_position = coordinates - cell_position;

  float sum = 0.0;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        vec3 cell_offset = vec3(i, j, k);
        vec3 current_cell_position = cell_position + cell_offset;
        vec3 intra_cell_position = local_position - cell_offset;
        sum += compute_3d_gabor_noise_cell(current_cell_position,
                                           intra_cell_position,
                                           impulses_count,
                                           frequency,
                                           isotropy,
                                           base_orientation);
      }
    }
  }

  return sum;
}

void node_tex_gabor(vec3 coordinates,
                    float scale,
                    float impulses_count,
                    float frequency,
                    float anisotropy,
                    float orientation_2d,
                    vec3 orientation_3d,
                    float type,
                    out float output_value)
{
  vec3 scaled_coordinates = coordinates * scale;
  float isotropy = 1.0 - clamp(anisotropy, 0.0, 1.0);

  if (type == SHD_GABOR_TYPE_2D) {
    output_value = compute_2d_gabor_noise(
        scaled_coordinates.xy, impulses_count, frequency, isotropy, orientation_2d);
  }
  else if (type == SHD_GABOR_TYPE_3D) {
    vec3 orientation = normalize(orientation_3d);
    output_value = compute_3d_gabor_noise(
        scaled_coordinates, impulses_count, frequency, isotropy, orientation);
  }
}
