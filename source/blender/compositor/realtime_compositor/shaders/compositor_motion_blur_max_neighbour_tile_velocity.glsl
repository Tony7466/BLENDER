/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Implements the neighbourhood blurring method as described in section "4.4. Neighbor Blurring" of
 * the paper:
 *
 *   Guertin, Jean-Philippe, Morgan McGuire, and Derek Nowrouzezahrai. "A Fast and Stable
 *   Feature-Aware Motion Blur Filter." High performance graphics. 2014.
 */

#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);

  /* Find the maximum previous and next velocities in the 3x3 neighbourhood according to their
   * magnitude. */
  vec2 maximum_next_velocity = vec2(0.0);
  vec2 maximum_previous_velocity = vec2(0.0);
  float maximum_next_velocity_squared_magnitude = 0.0;
  float maximum_previous_velocity_squared_magnitude = 0.0;
  for (int j = -1; j < 1; j++) {
    for (int i = -1; i < 1; i++) {
      ivec2 offset = ivec2(i, j);

      vec4 velocity = texture_load(input_tx, texel + offset);
      vec2 previous_velocity = velocity.xy;
      vec2 next_velocity = velocity.zw;

      float previous_velocity_squared_magnitude = dot(previous_velocity, previous_velocity);
      float next_velocity_squared_magnitude = dot(next_velocity, next_velocity);

      bool is_corner = all(equal(abs(offset), ivec2(1)));
      if (previous_velocity_squared_magnitude > maximum_previous_velocity_squared_magnitude) {
        /* Corner tile velocities are only considered if they point towards the center tile, which
         * is detected using the dot product. */
        if (!is_corner || dot(vec2(offset), previous_velocity) < 0.0) {
          maximum_previous_velocity_squared_magnitude = previous_velocity_squared_magnitude;
          maximum_previous_velocity = previous_velocity;
        }
      }

      if (next_velocity_squared_magnitude > maximum_next_velocity_squared_magnitude) {
        /* Corner tile velocities are only considered if they point towards the center tile, which
         * is detected using the dot product. */
        if (!is_corner || dot(vec2(offset), next_velocity) < 0.0) {
          maximum_next_velocity_squared_magnitude = next_velocity_squared_magnitude;
          maximum_next_velocity = next_velocity;
        }
      }
    }
  }

  imageStore(output_img, texel, vec4(maximum_previous_velocity, maximum_next_velocity));
}
