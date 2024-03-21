/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* Displacement */

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(shader_eval_displace,
                             ccl_global KernelShaderEvalInput *input,
                             ccl_global float *output,
                             const int offset,
                             const int work_size)
{
  int i = ccl_gpu_global_id_x();
  if (i < work_size) {
    ccl_gpu_kernel_call(kernel_displace_evaluate(NULL, input, output, offset + i));
  }
}
ccl_gpu_kernel_postfix

/* Background */

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(shader_eval_background,
                             ccl_global KernelShaderEvalInput *input,
                             ccl_global float *output,
                             const int offset,
                             const int work_size)
{
  int i = ccl_gpu_global_id_x();
  if (i < work_size) {
    ccl_gpu_kernel_call(kernel_background_evaluate(NULL, input, output, offset + i));
  }
}
ccl_gpu_kernel_postfix

/* Curve Shadow Transparency */

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(shader_eval_curve_shadow_transparency,
                             ccl_global KernelShaderEvalInput *input,
                             ccl_global float *output,
                             const int offset,
                             const int work_size)
{
  int i = ccl_gpu_global_id_x();
  if (i < work_size) {
    ccl_gpu_kernel_call(
        kernel_curve_shadow_transparency_evaluate(NULL, input, output, offset + i));
  }
}
ccl_gpu_kernel_postfix
