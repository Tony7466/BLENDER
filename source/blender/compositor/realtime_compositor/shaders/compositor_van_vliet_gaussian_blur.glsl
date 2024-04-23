/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

#define FILTER_ORDER 2

void main()
{
  int y = int(gl_GlobalInvocationID.x);
  int width = texture_size(input_tx).x;

  /* **** First Causal Filter **** */

  {
    vec4 input_boundary = texture_load(input_tx, ivec2(0, y));
    vec4 inputs[FILTER_ORDER] = vec4[](input_boundary, input_boundary);

    vec4 output_boundary = input_boundary * boundary_coefficient;
    vec4 outputs[FILTER_ORDER + 1] = vec4[](output_boundary, output_boundary, output_boundary);

    for (int x = 0; x < width; x++) {
      ivec2 texel = ivec2(x, y);
      inputs[0] = texture_load(input_tx, texel);

      outputs[0] = vec4(0.0);
      for (int i = 0; i < FILTER_ORDER; i++) {
        outputs[0] += first_causal_feedforward_coefficients[i] * inputs[i];
        outputs[0] -= first_feedback_coefficients[i] * outputs[i + 1];
      }

      imageStore(output_img, texel.yx, outputs[0]);

      for (int i = FILTER_ORDER - 1; i >= 1; i--) {
        inputs[i] = inputs[i - 1];
      }

      for (int i = FILTER_ORDER; i >= 1; i--) {
        outputs[i] = outputs[i - 1];
      }
    }
  }

  /* **** First Non Causal Filter **** */

  {
    vec4 input_boundary = texture_load(input_tx, ivec2(width - 1, y));
    vec4 inputs[FILTER_ORDER + 1] = vec4[](input_boundary, input_boundary, input_boundary);

    vec4 output_boundary = input_boundary * boundary_coefficient;
    vec4 outputs[FILTER_ORDER + 1] = vec4[](output_boundary, output_boundary, output_boundary);

    for (int x = width - 1; x >= 0; x--) {
      ivec2 texel = ivec2(x, y);
      inputs[0] = texture_load(input_tx, texel);

      outputs[0] = vec4(0.0);
      for (int i = 0; i < FILTER_ORDER; i++) {
        outputs[0] += first_non_causal_feedforward_coefficients[i] * inputs[i + 1];
        outputs[0] -= first_feedback_coefficients[i] * outputs[i + 1];
      }

      vec4 previous_output = imageLoad(output_img, texel.yx);
      imageStore(output_img, texel.yx, outputs[0] + previous_output);

      for (int i = FILTER_ORDER; i >= 1; i--) {
        inputs[i] = inputs[i - 1];
      }

      for (int i = FILTER_ORDER; i >= 1; i--) {
        outputs[i] = outputs[i - 1];
      }
    }
  }

  /* **** Second Causal Filter **** */

  {
    vec4 input_boundary = texture_load(input_tx, ivec2(0, y));
    vec4 inputs[FILTER_ORDER] = vec4[](input_boundary, input_boundary);

    vec4 output_boundary = input_boundary * boundary_coefficient;
    vec4 outputs[FILTER_ORDER + 1] = vec4[](output_boundary, output_boundary, output_boundary);

    for (int x = 0; x < width; x++) {
      ivec2 texel = ivec2(x, y);
      inputs[0] = texture_load(input_tx, texel);

      outputs[0] = vec4(0.0);
      for (int i = 0; i < FILTER_ORDER; i++) {
        outputs[0] += second_causal_feedforward_coefficients[i] * inputs[i];
        outputs[0] -= second_feedback_coefficients[i] * outputs[i + 1];
      }

      vec4 previous_output = imageLoad(output_img, texel.yx);
      imageStore(output_img, texel.yx, outputs[0] + previous_output);

      for (int i = FILTER_ORDER - 1; i >= 1; i--) {
        inputs[i] = inputs[i - 1];
      }

      for (int i = FILTER_ORDER; i >= 1; i--) {
        outputs[i] = outputs[i - 1];
      }
    }
  }

  /* **** Second Non Causal Filter **** */

  {
    vec4 input_boundary = texture_load(input_tx, ivec2(width - 1, y));
    vec4 inputs[FILTER_ORDER + 1] = vec4[](input_boundary, input_boundary, input_boundary);

    vec4 output_boundary = input_boundary * boundary_coefficient;
    vec4 outputs[FILTER_ORDER + 1] = vec4[](output_boundary, output_boundary, output_boundary);

    for (int x = width - 1; x >= 0; x--) {
      ivec2 texel = ivec2(x, y);
      inputs[0] = texture_load(input_tx, texel);

      outputs[0] = vec4(0.0);
      for (int i = 0; i < FILTER_ORDER; i++) {
        outputs[0] += second_non_causal_feedforward_coefficients[i] * inputs[i + 1];
        outputs[0] -= second_feedback_coefficients[i] * outputs[i + 1];
      }

      vec4 previous_output = imageLoad(output_img, texel.yx);
      imageStore(output_img, texel.yx, outputs[0] + previous_output);

      for (int i = FILTER_ORDER; i >= 1; i--) {
        inputs[i] = inputs[i - 1];
      }

      for (int i = FILTER_ORDER; i >= 1; i--) {
        outputs[i] = outputs[i - 1];
      }
    }
  }
}
