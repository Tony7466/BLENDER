/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#define __OSL__

#include "kernel/device/optix/compat.h"
#include "kernel/device/optix/globals.h"

#include "kernel/device/gpu/image.h"
#include "kernel/tables.h"

#include "kernel/integrator/state.h"
#include "kernel/integrator/state_flow.h"
#include "kernel/integrator/state_util.h"

#include "kernel/bake/bake.h"

extern "C" __global__ void __raygen__kernel_optix_shader_eval_displace()
{
  KernelShaderEvalInput *const input = (KernelShaderEvalInput *)kernel_params.path_index_array;
  float *const output = kernel_params.render_buffer;
  const int global_index = kernel_params.offset + optixGetLaunchIndex().x;
  kernel_displace_evaluate(nullptr, input, output, global_index);
}

extern "C" __global__ void __raygen__kernel_optix_shader_eval_background()
{
  KernelShaderEvalInput *const input = (KernelShaderEvalInput *)kernel_params.path_index_array;
  float *const output = kernel_params.render_buffer;
  const int global_index = kernel_params.offset + optixGetLaunchIndex().x;
  kernel_background_evaluate(nullptr, input, output, global_index);
}

extern "C" __global__ void __raygen__kernel_optix_shader_eval_curve_shadow_transparency()
{
  KernelShaderEvalInput *const input = (KernelShaderEvalInput *)kernel_params.path_index_array;
  float *const output = kernel_params.render_buffer;
  const int global_index = kernel_params.offset + optixGetLaunchIndex().x;
  kernel_curve_shadow_transparency_evaluate(nullptr, input, output, global_index);
}
