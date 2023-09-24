/* SPDX-FileCopyrightText: 2021-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

/* Parallel prefix sum.
 *
 * TODO: actually make this work in parallel.
 *
 * This is used for an array the size of the number of shaders in the scene
 * which is not usually huge, so might not be a significant bottleneck. */

#include "util/atomic.h"

#ifdef __HIP__
#  define GPU_PARALLEL_PREFIX_SUM_DEFAULT_BLOCK_SIZE 1024
#else
#  define GPU_PARALLEL_PREFIX_SUM_DEFAULT_BLOCK_SIZE 512
#endif


__device__ void gpu_parallel_prefix_sum(const int global_id,
                                        ccl_global int *counter,
                                        ccl_global int *prefix_sum,
                                        const int num_values,
                                        const int num_path_limit,
                                        const int merge_dispatch_threshold,
                                        const ushort threadgroup_size,
                                        ccl_global int* offset_per_threadgroup,
                                        ccl_global uint *threadgroups_per_shader,
                                        ccl_global uint *offset_per_shader,
                                        const bool material_specialization_enabled)
{
  if (global_id != 0) {
    return;
  }
    
  int offset = 0;
  for (int i = 0; i < num_values; i++) {
    const int new_offset = offset + counter[i];
    prefix_sum[i] = offset;
    counter[i] = 0;
          
    if(material_specialization_enabled)
    {
        threadgroups_per_shader[3 * i + 0] = (max( min(new_offset, num_path_limit) - offset, 0) + threadgroup_size - 1) / threadgroup_size;
        threadgroups_per_shader[3 * i + 1] = 1;
        threadgroups_per_shader[3 * i + 2] = 1;
        
        offset_per_threadgroup[i] = min(offset, num_path_limit);
    }
      
    offset = new_offset;
  }

  offset_per_shader[num_values] = min(offset, num_path_limit);
  if(material_specialization_enabled)
  {
      offset_per_threadgroup[num_values] = min(offset, num_path_limit);
  }
}

CCL_NAMESPACE_END
