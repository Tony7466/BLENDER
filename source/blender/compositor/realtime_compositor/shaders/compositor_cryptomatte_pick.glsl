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
  vec2 first_rank = texture_load(first_layer_tx, texel).xy;
  float id_of_first_rank = first_rank.x;

  uint hash_value = floatBitsToUint(id_of_first_rank);
  float green = float(hash_value << 8) / float(0xFFFFFFFFu);
  float blue = float(hash_value << 16) / float(0xFFFFFFFFu);

  imageStore(output_img, texel, vec4(id_of_first_rank, green, blue, 1.0));
}
