/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

#define SAT_BLOCK_SIZE 16

vec4 summed_area_table_value(sampler2D blocks,
                             sampler2D x_prologues,
                             sampler2D y_prologues,
                             ivec2 texel,
                             out vec4 inter_value)
{
  inter_value = vec4(0.0);
  int start_y = (texel.y / SAT_BLOCK_SIZE) * SAT_BLOCK_SIZE;
  for (int i = start_y; i <= texel.y; i++) {
    inter_value += texture_load(x_prologues, ivec2(i, texel.x / SAT_BLOCK_SIZE), vec4(0.0));
  }

  inter_value += texture_load(y_prologues, texel / ivec2(1, SAT_BLOCK_SIZE), vec4(0.0));

  return texture_load(blocks, texel, vec4(0.0));
}

/* Computes the sum of the rectangular region defined by the given lower and upper bounds from the
 * given summed area table. It is assumed that the given upper bound is larger than the given lower
 * bound, otherwise, undefined behavior is invoked. Looking at the diagram below, in order to
 * compute the sum of area X, we sample the table at each of the corners of the area X, to get:
 *
 *   Upper Right -> A + B + C + X      (1)
 *   Upper Left -> A + B               (2)
 *   Lower Right -> B + C              (3)
 *   Lower Left -> B                   (4)
 *
 * We start from (1) and subtract (2) and (3) to get rid of A and C to get:
 *
 *  (A + B + C + X) - (A + B) - (B + C) = (X - B)
 *
 * To get rid of B, we add (4) to get:
 *
 *  (X - B) + B = X
 *
 *         ^
 *         |
 *         +-------+-----+
 *         |       |     |
 *         |   A   |  X  |
 *         |       |     |
 *         +-------+-----+
 *         |       |     |
 *         |   B   |  C  |
 *         |       |     |
 *         o-------+-----+------>
 *
 * The aforementioned equation eliminates the edges between regions X, C, and A since they get
 * subtracted with C and A. To avoid this, we subtract 1 from the lower bound and fallback to zero
 * for out of bound sampling. */
vec4 summed_area_table_sum(sampler2D blocks,
                           sampler2D x_prologues,
                           sampler2D y_prologues,
                           ivec2 lower_bound,
                           ivec2 upper_bound)
{
  ivec2 lower_left_texel = lower_bound - ivec2(1);
  ivec2 upper_right_texel = min(texture_size(blocks) - ivec2(1), upper_bound);
  ivec2 upper_left_texel = ivec2(lower_left_texel.x, upper_right_texel.y);
  ivec2 lower_right_texel = ivec2(upper_right_texel.x, lower_left_texel.y);

  vec4 inter_upper_right, inter_lower_left, inter_upper_left, inter_lower_right;

  vec4 intra_upper_right = summed_area_table_value(
      blocks, x_prologues, y_prologues, upper_right_texel, inter_upper_right);
  vec4 intra_lower_left = summed_area_table_value(
      blocks, x_prologues, y_prologues, lower_left_texel, inter_lower_left);
  vec4 intra_upper_left = summed_area_table_value(
      blocks, x_prologues, y_prologues, upper_left_texel, inter_upper_left);
  vec4 intra_lower_right = summed_area_table_value(
      blocks, x_prologues, y_prologues, lower_right_texel, inter_lower_right);

  vec4 intra_value = (intra_upper_right + intra_lower_left) -
                     (intra_upper_left + intra_lower_right);
  vec4 inter_value = (inter_upper_right + inter_lower_left) -
                     (inter_upper_left + inter_lower_right);
  return intra_value + inter_value;
}
