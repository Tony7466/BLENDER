/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"

#include "GPU_compute.h"
#include "GPU_shader.h"
#include "GPU_texture.h"

#include "COM_context.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "COM_algorithm_summed_area_table.hh"

namespace blender::realtime_compositor {

/* Computes the horizontal and vertical incomplete prologues from the given input using equations
 * (42) and (43) to implement the first pass of Algorithm SAT. Those equations simply sum each row
 * and column in each block and write the sum to the incomplete prologues. We compute both vertical
 * and horizontal sums at the same time in the same shader dispatch, so we dispatch a range that
 * covers the larger of both axis, out of bound computations for the smaller axis is then simply
 * ignored. Summing rows and columns within each block is done serially using an accumulation loop
 * in the shader, but we run that sum in parallel for all blocks and for all rows and columns. We
 * call the axis that will be summed the serial axis, and we call the other axis the parallel axis.
 * Simply swapping the axis switches between vertical and horizontal accumulation. We store both
 * prologues with the parallel axis aligned with the texture horizontal axis for better cache
 * locality. */
static void compute_incomplete_prologues(Context &context,
                                         Result &input,
                                         Result &incomplete_x_prologues,
                                         Result &incomplete_y_prologues)
{
  GPUShader *shader = context.shader_manager().get(
      "compositor_summed_area_table_compute_incomplete_prologues");
  GPU_shader_bind(shader);

  input.bind_as_texture(shader, "input_tx");

  const int2 group_size = int2(16);
  const int2 input_size = input.domain().size;
  const int2 number_of_groups = math::divide_ceil(input_size, group_size);

  incomplete_x_prologues.allocate_texture(Domain(int2(input_size.y, number_of_groups.x)));
  incomplete_x_prologues.bind_as_image(shader, "incomplete_x_prologues_img");

  incomplete_y_prologues.allocate_texture(Domain(int2(input_size.x, number_of_groups.y)));
  incomplete_y_prologues.bind_as_image(shader, "incomplete_y_prologues_img");

  /* We take the maximum of both axis to compute both axis at the same time in the shader, see the
   * function description for more information. */
  const int parallel_axis_groups = math::max(number_of_groups.x, number_of_groups.y);
  const int2 number_of_sub_groups = math::divide_ceil(number_of_groups, group_size);
  const int serial_axis_groups = math::max(number_of_sub_groups.x, number_of_sub_groups.y);
  GPU_compute_dispatch(shader, parallel_axis_groups, serial_axis_groups, 1);

  GPU_shader_unbind();
  input.unbind_as_texture();
  incomplete_x_prologues.unbind_as_image();
  incomplete_y_prologues.unbind_as_image();
}

/* Computes the complete X prologues and their sum from the incomplete X prologues using equation
 * (44) to implement the second pass of Algorithm SAT. That equation simply sum the incomplete
 * prologue and all incomplete prologues before it, writing the sum to the complete prologue. Then,
 * each of the complete prologues is summed using parallel reduction writing the sum to the output
 * sum for each block. The shader runs in parallel vertically, but serially horizontally. Note that
 * the input incomplete X prologues and output complete X prologues are stored transposed for
 * better cache locality, but the output sum is stored straight, not transposed. */
static void compute_complete_x_prologues(Context &context,
                                         Result &input,
                                         Result &incomplete_x_prologues,
                                         Result &complete_x_prologues,
                                         Result &complete_x_prologues_sum)
{
  GPUShader *shader = context.shader_manager().get(
      "compositor_summed_area_table_compute_complete_x_prologues");
  GPU_shader_bind(shader);

  incomplete_x_prologues.bind_as_texture(shader, "incomplete_x_prologues_tx");

  const int2 group_size = int2(16);
  const int2 input_size = input.domain().size;
  const int2 number_of_groups = math::divide_ceil(input_size, group_size);

  complete_x_prologues.allocate_texture(incomplete_x_prologues.domain());
  complete_x_prologues.bind_as_image(shader, "complete_x_prologues_img");

  complete_x_prologues_sum.allocate_texture(Domain(number_of_groups));
  complete_x_prologues_sum.bind_as_image(shader, "complete_x_prologues_sum_img");

  GPU_compute_dispatch(shader, number_of_groups.y, 1, 1);

  GPU_shader_unbind();
  incomplete_x_prologues.unbind_as_texture();
  complete_x_prologues.unbind_as_image();
  complete_x_prologues_sum.unbind_as_image();
}

/* Computes the complete Y prologues from the incomplete Y prologues using equation (45) to
 * implement the third pass of Algorithm SAT. That equation simply sum the incomplete prologue and
 * all incomplete prologues before it, then adds the sum of the complete X prologue for the same
 * block, writing the sum to the complete prologue. The shader runs in parallel horizontally, but
 * serially vertically. */
static void compute_complete_y_prologues(Context &context,
                                         Result &input,
                                         Result &incomplete_y_prologues,
                                         Result &complete_x_prologues_sum,
                                         Result &complete_y_prologues)
{
  GPUShader *shader = context.shader_manager().get(
      "compositor_summed_area_table_compute_complete_y_prologues");
  GPU_shader_bind(shader);

  incomplete_y_prologues.bind_as_texture(shader, "incomplete_y_prologues_tx");
  complete_x_prologues_sum.bind_as_texture(shader, "complete_x_prologues_sum_tx");

  const int2 group_size = int2(16);
  const int2 input_size = input.domain().size;
  const int2 number_of_groups = math::divide_ceil(input_size, group_size);

  complete_y_prologues.allocate_texture(incomplete_y_prologues.domain());
  complete_y_prologues.bind_as_image(shader, "complete_y_prologues_img");

  GPU_compute_dispatch(shader, number_of_groups.x, 1, 1);

  GPU_shader_unbind();
  incomplete_y_prologues.unbind_as_texture();
  complete_x_prologues_sum.unbind_as_texture();
  complete_y_prologues.unbind_as_image();
}

/* Computes the final summed area table blocks from the complete X and Y prologues using equation
 * (41) to implement the fourth pass of Algorithm SAT. That equation simply uses an intermediate
 * shared memory to cascade the accumulation of rows and then column in each block using the
 * prologues as initial values and writes each step of the latter accumulation to the output. Since
 * we cascade the X and Y accumulations, we dispatch a range that covers the larger of both axis,
 * out of bound computations for the smaller axis is then simply ignored. Summing rows and columns
 * within each block is done serially using an accumulation loop in the shader, but we run that sum
 * in parallel for all blocks and for all rows and columns. We call the axis that will be summed
 * the serial axis, and we call the other axis the parallel axis. Simply swapping the axis switches
 * between vertical and horizontal accumulation. */
static void compute_complete_blocks(Context &context,
                                    Result &input,
                                    Result &complete_x_prologues,
                                    Result &complete_y_prologues,
                                    Result &output)
{
  GPUShader *shader = context.shader_manager().get(
      "compositor_summed_area_table_compute_complete_blocks");
  GPU_shader_bind(shader);

  input.bind_as_texture(shader, "input_tx");
  complete_x_prologues.bind_as_texture(shader, "complete_x_prologues_tx");
  complete_y_prologues.bind_as_texture(shader, "complete_y_prologues_tx");

  output.allocate_texture(input.domain());
  output.bind_as_image(shader, "output_img", true);

  const int2 group_size = int2(16);
  const int2 input_size = input.domain().size;
  const int2 number_of_groups = math::divide_ceil(input_size, group_size);

  GPU_compute_dispatch(shader, number_of_groups.x, number_of_groups.y, 1);

  GPU_shader_unbind();
  input.unbind_as_texture();
  complete_x_prologues.unbind_as_texture();
  complete_y_prologues.unbind_as_texture();
  output.unbind_as_image();
}

/* An implementation of the summed area table algorithm from the paper:
 *
 *   Nehab, Diego, et al. "GPU-efficient recursive filtering and summed-area tables."
 *
 * This function is a straightforward implementation of each of the four passes described in
 * Algorithm SAT in section 6 of the paper. Note that we use Blender's convention of first
 * quadrant images, so we call prologues horizontal or X prologues, and we call transposed
 * prologues vertical or Y prologues. See each of the functions for more details. */
void summed_area_table(Context &context, Result &input, Result &output)
{
  Result incomplete_x_prologues = Result::Temporary(ResultType::Color, context.texture_pool());
  Result incomplete_y_prologues = Result::Temporary(ResultType::Color, context.texture_pool());
  compute_incomplete_prologues(context, input, incomplete_x_prologues, incomplete_y_prologues);

  Result complete_x_prologues = Result::Temporary(ResultType::Color, context.texture_pool());
  Result complete_x_prologues_sum = Result::Temporary(ResultType::Color, context.texture_pool());
  compute_complete_x_prologues(
      context, input, incomplete_x_prologues, complete_x_prologues, complete_x_prologues_sum);
  incomplete_x_prologues.release();

  Result complete_y_prologues = Result::Temporary(ResultType::Color, context.texture_pool());
  compute_complete_y_prologues(
      context, input, incomplete_y_prologues, complete_x_prologues_sum, complete_y_prologues);
  incomplete_y_prologues.release();
  complete_x_prologues_sum.release();

  compute_complete_blocks(context, input, complete_x_prologues, complete_y_prologues, output);
  complete_x_prologues.release();
  complete_y_prologues.release();
}

}  // namespace blender::realtime_compositor
