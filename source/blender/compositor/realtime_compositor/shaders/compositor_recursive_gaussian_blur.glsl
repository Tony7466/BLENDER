/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_compositor_texture_utilities.glsl)

void main()
{
  int y = int(gl_GlobalInvocationID.x);
  int width = texture_size(input_tx).x;

  /* **** Causal Filter **** */

  vec4 boundary = texture_load(input_tx, ivec2(0, y));
  vec4 causal_inputs[4] = vec4[](boundary, boundary, boundary, boundary);
  vec4 causal_outputs[5] = vec4[](boundary, boundary, boundary, boundary, boundary);

  for (int x = 0; x < width; x++) {
    ivec2 texel = ivec2(x, y);
    causal_inputs[0] = texture_load(input_tx, texel);

    causal_outputs[0] = vec4(0.0);
    for (int i = 0; i < causal_inputs.length(); i++) {
      causal_outputs[0] += causal_feedforward_coefficients[i] * causal_inputs[i];
      causal_outputs[0] -= feedback_coefficients[i] * causal_outputs[i + 1];
    }

    imageStore(output_img, texel.yx, vec4(causal_outputs[0]));

    for (int i = causal_inputs.length() - 1; i >= 1; i--) {
      causal_inputs[i] = causal_inputs[i - 1];
    }

    for (int i = causal_outputs.length() - 1; i >= 1; i--) {
      causal_outputs[i] = causal_outputs[i - 1];
    }
  }

  /* **** Non Causal Filter **** */

  boundary = texture_load(input_tx, ivec2(width - 1, y));
  vec4 non_causal_inputs[5] = vec4[](boundary, boundary, boundary, boundary, boundary);
  vec4 non_causal_outputs[5] = vec4[](boundary, boundary, boundary, boundary, boundary);

  for (int x = width - 1; x >= 0; x--) {
    ivec2 texel = ivec2(x, y);
    non_causal_inputs[0] = texture_load(input_tx, texel);

    non_causal_outputs[0] = dvec4(0.0);
    for (int i = 0; i < non_causal_inputs.length() - 1; i++) {
      non_causal_outputs[0] += non_causal_feedforward_coefficients[i] * non_causal_inputs[i + 1];
      non_causal_outputs[0] -= feedback_coefficients[i] * non_causal_outputs[i + 1];
    }

    vec4 causal_output = imageLoad(output_img, texel.yx);
    imageStore(output_img, texel.yx, vec4(non_causal_outputs[0] + causal_output));

    for (int i = non_causal_inputs.length() - 1; i >= 1; i--) {
      non_causal_inputs[i] = non_causal_inputs[i - 1];
    }

    for (int i = non_causal_outputs.length() - 1; i >= 1; i--) {
      non_causal_outputs[i] = non_causal_outputs[i - 1];
    }
  }
}
