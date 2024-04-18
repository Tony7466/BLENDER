/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

#define FILTER_ORDER 4

void main()
{
  int y = int(gl_GlobalInvocationID.x);
  int width = texture_size(input_tx).x;

  /* **** Causal Filter **** */

  vec4 boundary = texture_load(input_tx, ivec2(0, y));
  vec4 causal_inputs[FILTER_ORDER] = vec4[](boundary, boundary, boundary, boundary);
  vec4 causal_outputs[FILTER_ORDER + 1] = vec4[](boundary, boundary, boundary, boundary, boundary);

  for (int x = 0; x < width; x++) {
    ivec2 texel = ivec2(x, y);
    causal_inputs[0] = texture_load(input_tx, texel);

    causal_outputs[0] = vec4(0.0);
    for (int i = 0; i < FILTER_ORDER; i++) {
      causal_outputs[0] += causal_feedforward_coefficients[i] * causal_inputs[i];
      causal_outputs[0] -= feedback_coefficients[i] * causal_outputs[i + 1];
    }

    imageStore(output_img, texel.yx, causal_outputs[0]);

    for (int i = FILTER_ORDER - 1; i >= 1; i--) {
      causal_inputs[i] = causal_inputs[i - 1];
    }

    for (int i = FILTER_ORDER; i >= 1; i--) {
      causal_outputs[i] = causal_outputs[i - 1];
    }
  }

  /* **** Non Causal Filter **** */

  boundary = texture_load(input_tx, ivec2(width - 1, y));
  vec4 non_causal_inputs[FILTER_ORDER + 1] = vec4[](
      boundary, boundary, boundary, boundary, boundary);
  vec4 non_causal_outputs[FILTER_ORDER + 1] = vec4[](
      boundary, boundary, boundary, boundary, boundary);

  for (int x = width - 1; x >= 0; x--) {
    ivec2 texel = ivec2(x, y);
    non_causal_inputs[0] = texture_load(input_tx, texel);

    non_causal_outputs[0] = vec4(0.0);
    for (int i = 0; i < FILTER_ORDER; i++) {
      non_causal_outputs[0] += non_causal_feedforward_coefficients[i] * non_causal_inputs[i + 1];
      non_causal_outputs[0] -= feedback_coefficients[i] * non_causal_outputs[i + 1];
    }

    vec4 causal_output = imageLoad(output_img, texel.yx);
    imageStore(output_img, texel.yx, non_causal_outputs[0] + causal_output);

    for (int i = FILTER_ORDER; i >= 1; i--) {
      non_causal_inputs[i] = non_causal_inputs[i - 1];
    }

    for (int i = FILTER_ORDER; i >= 1; i--) {
      non_causal_outputs[i] = non_causal_outputs[i - 1];
    }
  }
}
