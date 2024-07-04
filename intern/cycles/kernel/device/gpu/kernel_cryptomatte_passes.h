/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(cryptomatte_postprocess,
                             ccl_global float *render_buffer,
                             int num_pixels)
{
  const int pixel_index = ccl_gpu_global_id_x();

  if (pixel_index < num_pixels) {
    ccl_gpu_kernel_call(film_cryptomatte_post(nullptr, render_buffer, pixel_index));
  }
}
ccl_gpu_kernel_postfix
