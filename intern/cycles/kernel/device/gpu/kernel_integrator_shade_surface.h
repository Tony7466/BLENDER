/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(integrator_shade_surface,
                             ccl_global const int *path_index_array,
                             ccl_global float *render_buffer,
                             const int work_size)
{
  const int global_index = ccl_gpu_global_id_x();

  if (ccl_gpu_kernel_within_bounds(global_index, work_size)) {
    const int state = (path_index_array) ? path_index_array[global_index] : global_index;
    ccl_gpu_kernel_call(integrator_shade_surface(NULL, state, render_buffer));
  }
}
ccl_gpu_kernel_postfix

#if defined(__KERNEL_METAL_APPLE__) && defined(__METALRT__)
constant int __dummy_constant [[function_constant(Kernel_DummyConstant)]];
#endif

#if !defined(__HIPRT__)

/* Kernels using intersections need access to the kernel handler for specialization constants to
 * work properly. */
#  ifdef __KERNEL_ONEAPI__
#    include "kernel/device/oneapi/context_intersect_begin.h"
#  endif

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(integrator_shade_surface_raytrace,
                             ccl_global const int *path_index_array,
                             ccl_global float *render_buffer,
                             const int work_size)
{
  const int global_index = ccl_gpu_global_id_x();

  if (ccl_gpu_kernel_within_bounds(global_index, work_size)) {
    const int state = (path_index_array) ? path_index_array[global_index] : global_index;

#  if defined(__KERNEL_METAL_APPLE__) && defined(__METALRT__)
    KernelGlobals kg = NULL;
    /* Workaround Ambient Occlusion and Bevel nodes not working with Metal.
     * Dummy offset should not affect result, but somehow fixes bug! */
    kg += __dummy_constant;
    ccl_gpu_kernel_call(integrator_shade_surface_raytrace(kg, state, render_buffer));
#  else
    ccl_gpu_kernel_call(integrator_shade_surface_raytrace(NULL, state, render_buffer));
#  endif
  }
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(integrator_shade_surface_mnee,
                             ccl_global const int *path_index_array,
                             ccl_global float *render_buffer,
                             const int work_size)
{
  const int global_index = ccl_gpu_global_id_x();

  if (ccl_gpu_kernel_within_bounds(global_index, work_size)) {
    const int state = (path_index_array) ? path_index_array[global_index] : global_index;
    ccl_gpu_kernel_call(integrator_shade_surface_mnee(NULL, state, render_buffer));
  }
}
ccl_gpu_kernel_postfix

#  ifdef __KERNEL_ONEAPI__
#    include "kernel/device/oneapi/context_intersect_end.h"
#  endif

#else

ccl_gpu_kernel_postfix
ccl_gpu_kernel_threads(GPU_HIPRT_KERNEL_BLOCK_NUM_THREADS)
    ccl_gpu_kernel_signature(integrator_shade_surface_raytrace,
                             ccl_global const int *path_index_array,
                             ccl_global float *render_buffer,
                             const int work_size,
                             ccl_global int *stack_buffer)
{
  const int global_index = ccl_gpu_global_id_x();
  if (global_index < work_size) {
    HIPRT_INIT_KERNEL_GLOBAL()
    const int state = (path_index_array) ? path_index_array[global_index] : global_index;
    ccl_gpu_kernel_call(integrator_shade_surface_raytrace(kg, state, render_buffer));
  }
}
ccl_gpu_kernel_postfix
ccl_gpu_kernel_threads(GPU_HIPRT_KERNEL_BLOCK_NUM_THREADS)
    ccl_gpu_kernel_signature(integrator_shade_surface_mnee,
                             ccl_global const int *path_index_array,
                             ccl_global float *render_buffer,
                             const int work_size,
                             ccl_global int *stack_buffer)
{
  const int global_index = ccl_gpu_global_id_x();
  if (global_index < work_size) {
    HIPRT_INIT_KERNEL_GLOBAL()
    const int state = (path_index_array) ? path_index_array[global_index] : global_index;
    ccl_gpu_kernel_call(integrator_shade_surface_mnee(kg, state, render_buffer));
  }
}
ccl_gpu_kernel_postfix

#endif
