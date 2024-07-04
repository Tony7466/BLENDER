/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(integrator_shade_background,
                             ccl_global const int *path_index_array,
                             ccl_global float *render_buffer,
                             const int work_size)
{
  const int global_index = ccl_gpu_global_id_x();

  if (ccl_gpu_kernel_within_bounds(global_index, work_size)) {
    const int state = (path_index_array) ? path_index_array[global_index] : global_index;
    ccl_gpu_kernel_call(integrator_shade_background(NULL, state, render_buffer));
  }
}
ccl_gpu_kernel_postfix
