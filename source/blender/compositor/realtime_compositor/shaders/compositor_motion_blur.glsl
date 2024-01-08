/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* An implementation of the paper:
 *
 *   Guertin, Jean-Philippe, Morgan McGuire, and Derek Nowrouzezahrai. "A Fast and Stable
 *   Feature-Aware Motion Blur Filter." High performance graphics. 2014.
 *
 * Which is based on the paper:
 *
 *   McGuire, Morgan, et al. "A reconstruction filter for plausible motion blur." Proceedings of
 *   the ACM SIGGRAPH Symposium on Interactive 3D Graphics and Games. 2012.
 */

#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

/* The tiles are square and of size 32, as defined in the compositor_max_velocity shader */
#define TILE_SIZE 32

/* Computes the R2 low discrepancy grid remapped to the [0, 1] range as described in section
 * "Dithering in computer graphics" of the article:
 *
 *   https://extremelearning.com.au/unreasonable-effectiveness-of-quasirandom-sequences/
 *
 * This is a better alternative to the halton sequence proposed by the motion blur paper. */
float r2_low_discrepancy_grid(ivec2 p)
{
  float x = p.x / 1.32471795724474602596;
  float y = p.y / (1.32471795724474602596 * 1.32471795724474602596);
  return fract(x + y) * 2.0 - 1.0;
}

/* Find the tile that the given texel belongs to, which is just the tile divided by the tile size.
 * However, to avoid visible discontinuities, we stochastically offset into neighbouring tile at
 * tile boundaries as described in section "4.2. Tile Boundary Discontinuities" of the Guertin
 * paper.
 *
 * TODO: Only offset into vertical and horizontal tiles. Remove the trigonometric functions. */
ivec2 compute_tile_texel(ivec2 texel, float quasirandom_value)
{
  const ivec2 tile_size = ivec2(32);
  float angle = quasirandom_value * M_PI;
  ivec2 offset = ivec2(vec2(cos(angle), sin(angle)) * tile_size / 4.0);
  return (texel + offset) / tile_size;
}

/* Returns 1 if z_a is less than z_b, that is, in terms of depth, closer to the camera than z_b.
 * Returns zero otherwise. However, the values gradually decreases from 1 to zero in regions where
 * both values are close. This is described in section "5. Implementation and Results" of the
 * Guertin paper. */
float soft_depth_compare(float z_a, float z_b)
{
  return clamp(1.0 - (z_a - z_b) / min(z_a, z_b), 0.0, 1.0);
}

/* A cone shaped window function as described in "Figure 4" of the McGuire paper. A unit dirac
 * delta function when the scale is 0. */
float cone_window_function(float x, float scale)
{
  if (scale == 0.0) {
    return x == 0.0 ? 1.0 : 0.0;
  }
  return clamp(1.0 - x / scale, 0.0, 1.0);
}

/* A cylinder shaped window function as described in "Figure 4" of the McGuire paper. A unit dirac
 * delta function when the scale is 0. */
float cylinder_window_function(float x, float scale)
{
  if (scale == 0.0) {
    return x == 0.0 ? 1.0 : 0.0;
  }
  return 1.0 - smoothstep(0.95 * scale, 1.05 * scale, x);
}

/* A translation of the psudo code in "Appendix A: Pseudocode" of the Guertin paper. */
void main()
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);

  float quasirandom_value = r2_low_discrepancy_grid(texel);
  ivec2 tile_texel = compute_tile_texel(texel, quasirandom_value);

  vec2 max_velocity = texture_load(max_velocity_tx, tile_texel).xy * shutter_speed;
  float max_velocity_length = length(max_velocity);
  max_velocity /= max_velocity_length != 0.0 ? max_velocity_length : 1.0;

  vec4 pixel_color = texture_load(input_tx, texel);
  if (max_velocity_length <= 0.5) {
    imageStore(output_img, texel, pixel_color);
    return;
  }

  vec2 pixel_velocity = texture_load(velocity_tx, texel).xy * shutter_speed;
  float pixel_velocity_length = length(pixel_velocity);
  pixel_velocity /= pixel_velocity_length != 0.0 ? pixel_velocity_length : 1.0;

  vec2 perpendicular_max_velocity = max_velocity.yx * vec2(-1.0, 1.0);
  bool should_invert = dot(perpendicular_max_velocity, pixel_velocity) < 0.0;
  perpendicular_max_velocity *= should_invert ? -1.0 : 1.0;

  const float velocity_threshold = 1.5;
  float center_mix_factor = clamp((pixel_velocity_length - 0.5) / velocity_threshold, 0.0, 1.0);
  vec2 center_velocity = mix(perpendicular_max_velocity, pixel_velocity, center_mix_factor);
  float center_velocity_length = length(center_velocity);
  center_velocity /= center_velocity_length != 0.0 ? center_velocity_length : 1.0;

  float pixel_depth = texture_load(depth_tx, texel).x;

  float center_bias = 40.0;
  float center_weight = samples_count / (center_bias * max(1.0, pixel_velocity_length));

  float accumulated_weight = center_weight;
  vec4 accumulated_color = pixel_color * center_weight;
  for (int i = 0; i < samples_count; i++) {
    const float max_jitter = 0.95;
    float jitter = quasirandom_value * max_jitter;
    float parameter = mix(-1.0, 1.0, (i + 1 + jitter) / (samples_count + 1));
    float velocity_parameter = parameter * max_velocity_length;
    float velocity_magnitude = abs(velocity_parameter);

    bool use_center_velocity = i % 2 == 0;
    vec2 velocity = use_center_velocity ? center_velocity : max_velocity;

    vec2 sample_position = texel + velocity_parameter * velocity;
    vec2 sample_sampler_position = (sample_position + 0.5) / texture_size(input_tx);

    float sample_depth = texture(velocity_tx, sample_sampler_position).x;

    vec2 sample_velocity = texture(velocity_tx, sample_sampler_position).xy * shutter_speed;
    float sample_velocity_length = length(sample_velocity);
    sample_velocity /= sample_velocity_length != 0.0 ? sample_velocity_length : 1.0;

    float foreground = soft_depth_compare(pixel_depth, sample_depth);
    float background = soft_depth_compare(sample_depth, pixel_depth);

    float foreground_weight = max(0.0, dot(velocity, sample_velocity));
    float background_weight = max(0.0, dot(velocity, center_velocity));

    float weight = 0.0;
    weight += foreground * cone_window_function(velocity_magnitude, sample_velocity_length) *
              foreground_weight;
    weight += background * cone_window_function(velocity_magnitude, pixel_velocity_length) *
              background_weight;
    weight += cylinder_window_function(velocity_magnitude,
                                       min(sample_velocity_length, pixel_velocity_length)) *
              max(foreground_weight, background_weight) * 2.0;

    vec4 color = texture(input_tx, sample_sampler_position);

    accumulated_weight += weight;
    accumulated_color += color * weight;
  }

  accumulated_color /= accumulated_weight != 0.0 ? accumulated_weight : 1.0;
  imageStore(output_img, texel, accumulated_color);
}
