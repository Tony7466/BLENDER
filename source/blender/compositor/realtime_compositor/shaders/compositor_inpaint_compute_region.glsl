/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_compositor_jump_flooding_lib.glsl)

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);

  vec4 color = texture_load(input_tx, texel);

  /* An opaque pixel, no inpainting needed. */
  if (color.a == 1.0) {
    imageStore(output_img, texel, color);
    return;
  }

  vec4 flooding_value = texture_load(flooded_boundary_tx, texel);
  float distance_to_closest_seed = extract_jump_flooding_distance_to_closest_seed(flooding_value);

  /* Further than the user supplied distance, write a transparent color. */
  if (distance_to_closest_seed > distance) {
    imageStore(output_img, texel, vec4(0.0));
    return;
  }

  ivec2 closest_seed_texel = extract_jump_flooding_closest_seed_texel(flooding_value);
  vec4 closest_seed_color = texture_load(input_tx, closest_seed_texel);

  /* For half transparent pixels, mix with the original color using the alpha. */
  imageStore(output_img, texel, vec4(mix(closest_seed_color, color, color.a).rgb, 1.0));
}
