/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#if !defined(__HIPRT__)

/* Intersection kernels need access to the kernel handler for specialization constants to work
 * properly. */
#  ifdef __KERNEL_ONEAPI__
#    include "kernel/device/oneapi/context_intersect_begin.h"
#  endif

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(integrator_intersect_volume_stack,
                             ccl_global const int *path_index_array,
                             const int work_size)
{
  const int global_index = ccl_gpu_global_id_x();

  if (ccl_gpu_kernel_within_bounds(global_index, work_size)) {
    const int state = (path_index_array) ? path_index_array[global_index] : global_index;
    ccl_gpu_kernel_call(integrator_intersect_volume_stack(NULL, state));
  }
}
ccl_gpu_kernel_postfix

#  ifdef __KERNEL_ONEAPI__
#    include "kernel/device/oneapi/context_intersect_end.h"
#  endif

#else

ccl_gpu_kernel_threads(GPU_HIPRT_KERNEL_BLOCK_NUM_THREADS)
    ccl_gpu_kernel_signature(integrator_intersect_volume_stack,
                             ccl_global const int *path_index_array,
                             const int work_size,
                             ccl_global int *stack_buffer)
{
  const int global_index = ccl_gpu_global_id_x();

  if (global_index < work_size) {
    HIPRT_INIT_KERNEL_GLOBAL()
    const int state = (path_index_array) ? path_index_array[global_index] : global_index;
    ccl_gpu_kernel_call(integrator_intersect_volume_stack(kg, state));
  }
}

#endif
