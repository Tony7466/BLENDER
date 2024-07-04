/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(adaptive_sampling_convergence_check,
                             ccl_global float *render_buffer,
                             int sx,
                             int sy,
                             int sw,
                             int sh,
                             float threshold,
                             int reset,
                             int offset,
                             int stride,
                             ccl_global uint *num_active_pixels)
{
  const int work_index = ccl_gpu_global_id_x();
  const int y = work_index / sw;
  const int x = work_index - y * sw;

  bool converged = true;

  if (x < sw && y < sh) {
    converged = ccl_gpu_kernel_call(film_adaptive_sampling_convergence_check(
        nullptr, render_buffer, sx + x, sy + y, threshold, reset, offset, stride));
  }

  /* NOTE: All threads specified in the mask must execute the intrinsic. */
  const auto num_active_pixels_mask = ccl_gpu_ballot(!converged);
  const int lane_id = ccl_gpu_thread_idx_x % ccl_gpu_warp_size;
  if (lane_id == 0) {
    atomic_fetch_and_add_uint32(num_active_pixels, popcount(num_active_pixels_mask));
  }
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(adaptive_sampling_filter_x,
                             ccl_global float *render_buffer,
                             int sx,
                             int sy,
                             int sw,
                             int sh,
                             int offset,
                             int stride)
{
  const int y = ccl_gpu_global_id_x();

  if (y < sh) {
    ccl_gpu_kernel_call(
        film_adaptive_sampling_filter_x(NULL, render_buffer, sy + y, sx, sw, offset, stride));
  }
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(adaptive_sampling_filter_y,
                             ccl_global float *render_buffer,
                             int sx,
                             int sy,
                             int sw,
                             int sh,
                             int offset,
                             int stride)
{
  const int x = ccl_gpu_global_id_x();

  if (x < sw) {
    ccl_gpu_kernel_call(
        film_adaptive_sampling_filter_y(NULL, render_buffer, sx + x, sy, sh, offset, stride));
  }
}
ccl_gpu_kernel_postfix
