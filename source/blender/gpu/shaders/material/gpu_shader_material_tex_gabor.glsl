/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_common_hash.glsl)

#define NodeGaborMode int
#define SHD_GABOR_MODE_GABOR 0
#define SHD_GABOR_MODE_RING 1
#define SHD_GABOR_MODE_CROSS 2
#define SHD_GABOR_MODE_SQUARE 3
#define SHD_GABOR_MODE_PHASOR 4
#define SHD_GABOR_MODE_PHASOR_RING 5
#define SHD_GABOR_MODE_PHASOR_CROSS 6
#define SHD_GABOR_MODE_PHASOR_SQUARE 7

float compute_2d_gabor_noise(vec3 coordinates, NodeGaborMode mode)
{
  return 0.2;
}

float compute_3d_gabor_noise(vec3 coordinates, NodeGaborMode mode)
{
  return 0.8;
}

void node_tex_gabor(
    vec3 coordinates, float scale, float dimensions, float mode, out float output_value)
{
  vec3 scaled_coordinates = coordinates * scale;
  NodeGaborMode gabor_mode = NodeGaborMode(mode);

  if (dimensions == 2.0) {
    output_value = compute_2d_gabor_noise(scaled_coordinates, gabor_mode);
  }
  else if (dimensions == 3.0) {
    output_value = compute_3d_gabor_noise(scaled_coordinates, gabor_mode);
  }
}
