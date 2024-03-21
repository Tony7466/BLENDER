/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#define CCL_NOINLINE

#include "kernel/device/optix/compat.h"
#include "kernel/device/optix/globals.h"

#include "kernel/device/gpu/image.h"
#include "kernel/tables_extern.h"

#include "kernel/integrator/state_util.h"

#include "kernel/integrator/shade_volume.h"

extern "C" __device__ void __raygen__kernel_optix_integrator_shade_volume()
{
  const int global_index = optixGetLaunchIndex().x;
  const int path_index = (kernel_params.path_index_array) ?
                             kernel_params.path_index_array[global_index] :
                             global_index;
  integrator_shade_volume(nullptr, path_index, kernel_params.render_buffer);
}
