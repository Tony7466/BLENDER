/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/*
 *
 * Friedman, Jonah, and Andrew C. Jones. "Fully automatic id mattes with support for motion blur
 * and transparency." ACM SIGGRAPH 2015 Posters. 2015. 1-1.
 *
 */

#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  vec4 layer = texture_load(layer_tx, texel);

  vec2 first_rank = layer.xy;
  vec2 second_rank = layer.zw;

  float identifier_of_first_rank = first_rank.x;
  float coverage_of_first_rank = first_rank.y;
  float identifier_of_second_rank = second_rank.x;
  float coverage_of_second_rank = second_rank.y;

  float total_coverage = 0.0;
  for (int i = 0; i < identifiers_count; i++) {
    float identifier = identifiers[i];
    if (identifier_of_first_rank == identifier) {
      total_coverage += coverage_of_first_rank;
    }
    if (identifier_of_second_rank == identifier) {
      total_coverage += coverage_of_second_rank;
    }
  }

  imageStore(matte_img, texel, imageLoad(matte_img, texel) + vec4(total_coverage));
}
