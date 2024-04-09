/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"

#include "GPU_shader.hh"

#include "COM_context.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "COM_algorithm_recursive_gaussian_blur.hh"
#include "COM_deriche_gaussian_coefficients.hh"

namespace blender::realtime_compositor {

static Result horizontal_pass(Context &context, Result &input, float radius)
{
  GPUShader *shader = context.get_shader("compositor_recursive_gaussian_blur");
  GPU_shader_bind(shader);

  const float sigma = radius / 2.0f;
  const DericheGaussianCoefficients &coefficients =
      context.cache_manager().deriche_gaussian_coefficients.get(context, sigma);

  GPU_shader_uniform_4fv(
      shader, "causal_feedforward_coefficients", coefficients.causal_feedforward_coefficients());
  GPU_shader_uniform_4fv(shader,
                         "non_causal_feedforward_coefficients",
                         coefficients.non_causal_feedforward_coefficients());
  GPU_shader_uniform_4fv(shader, "feedback_coefficients", coefficients.feedback_coefficients());

  input.bind_as_texture(shader, "input_tx");

  const Domain domain = input.domain();

  /* We allocate an output image of a transposed size, that is, with a height equivalent to the
   * width of the input and vice versa. This is done as a performance optimization. The shader
   * will blur the image horizontally and write it to the intermediate output transposed. Then
   * the vertical pass will execute the same horizontal blur shader, but since its input is
   * transposed, it will effectively do a vertical blur and write to the output transposed,
   * effectively undoing the transposition in the horizontal pass. This is done to improve
   * spatial cache locality in the shader and to avoid having two separate shaders for each blur
   * pass. */
  const int2 transposed_domain = int2(domain.size.y, domain.size.x);

  Result horizontal_pass_result = context.create_temporary_result(ResultType::Color,
                                                                  ResultPrecision::Full);
  horizontal_pass_result.allocate_texture(transposed_domain);
  horizontal_pass_result.bind_as_image(shader, "output_img");

  compute_dispatch_threads_at_least(shader, int2(domain.size.y, 1), int2(256, 1));

  GPU_shader_unbind();
  input.unbind_as_texture();
  horizontal_pass_result.unbind_as_image();

  return horizontal_pass_result;
}

static void vertical_pass(Context &context,
                          Result &original_input,
                          Result &horizontal_pass_result,
                          Result &output,
                          float radius)
{
  GPUShader *shader = context.get_shader("compositor_recursive_gaussian_blur");
  GPU_shader_bind(shader);

  const float sigma = radius / 2.0f;
  const DericheGaussianCoefficients &coefficients =
      context.cache_manager().deriche_gaussian_coefficients.get(context, sigma);

  GPU_shader_uniform_4fv(
      shader, "causal_feedforward_coefficients", coefficients.causal_feedforward_coefficients());
  GPU_shader_uniform_4fv(shader,
                         "non_causal_feedforward_coefficients",
                         coefficients.non_causal_feedforward_coefficients());
  GPU_shader_uniform_4fv(shader, "feedback_coefficients", coefficients.feedback_coefficients());

  horizontal_pass_result.bind_as_texture(shader, "input_tx");

  const Domain domain = original_input.domain();
  output.allocate_texture(domain);
  output.bind_as_image(shader, "output_img");

  /* Notice that the domain is transposed, see the note on the horizontal pass method for more
   * information on the reasoning behind this. */
  compute_dispatch_threads_at_least(shader, int2(domain.size.x, 1), int2(256, 1));

  GPU_shader_unbind();
  output.unbind_as_image();
  horizontal_pass_result.unbind_as_texture();
}

void recursive_gaussian_blur(Context &context, Result &input, Result &output, float2 radius)
{
  Result horizontal_pass_result = horizontal_pass(context, input, radius.x);
  vertical_pass(context, input, horizontal_pass_result, output, radius.y);
  horizontal_pass_result.release();
}

}  // namespace blender::realtime_compositor
