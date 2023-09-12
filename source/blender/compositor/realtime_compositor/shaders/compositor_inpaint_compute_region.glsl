/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Fill the inpainting region by sampling the color of the nearest boundary pixel if it is not
 * further than the user supplied distance. Additionally, apply a lateral blur in the direction
 * tangent to the inpainting boundary to smooth out the inpainted region. */

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
  float distance_to_boundary = extract_jump_flooding_distance_to_closest_seed(flooding_value);

  /* Further than the user supplied distance, write a transparent color. */
  if (distance_to_boundary > distance) {
    imageStore(output_img, texel, vec4(0.0));
    return;
  }

  /* Compute the normalized tangent to the nearest boundary point. */
  ivec2 boundary_texel = extract_jump_flooding_closest_seed_texel(flooding_value);
  vec2 tangent = normalize(vec2((texel - boundary_texel).yx * ivec2(-1, 1)));

  float accumulated_weight = 0.0;
  vec4 accumulated_color = vec4(0.0);

  /* Accumulate the contribution of the boundary pixel as the center pixel. */
  float weight = texture(gaussian_weights_tx, 0.0).x;
  accumulated_color += texture_load(input_tx, boundary_texel) * weight;
  accumulated_weight += weight;

  /* We set the blur radius to be proportional to the distance to the boundary. */
  int blur_radius = int(ceil(distance_to_boundary));

  /* Accumulate the contributions of the boundary pixels nearest to the pixels to the left and
   * right in the direction of the tangent, noting that the weights texture only stores the weights
   * for the positive half, but since the Gaussian is symmetric, the same weight is used for the
   * negative half and we add both of their contributions. */
  for (int i = 1; i < blur_radius; i++) {
    /* Notice that the weight is used fro both the left and right contributions, hence the multiply
     * by two. */
    weight = texture(gaussian_weights_tx, float(i / (blur_radius - 1))).x;
    accumulated_weight += weight * 2.0;

    flooding_value = texture_load(flooded_boundary_tx, ivec2(vec2(texel) + tangent * i));
    boundary_texel = extract_jump_flooding_closest_seed_texel(flooding_value);
    accumulated_color += texture_load(input_tx, boundary_texel) * weight;

    flooding_value = texture_load(flooded_boundary_tx, ivec2(vec2(texel) - tangent * i));
    boundary_texel = extract_jump_flooding_closest_seed_texel(flooding_value);
    accumulated_color += texture_load(input_tx, boundary_texel) * weight;
  }

  imageStore(output_img, texel, accumulated_color / accumulated_weight);
}
