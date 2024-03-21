/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(integrator_reset, int num_states)
{
  const int state = ccl_gpu_global_id_x();

  if (state < num_states) {
    INTEGRATOR_STATE_WRITE(state, path, queued_kernel) = 0;
    INTEGRATOR_STATE_WRITE(state, shadow_path, queued_kernel) = 0;
  }
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel_threads(GPU_PARALLEL_ACTIVE_INDEX_DEFAULT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_queued_paths_array,
                             int num_states,
                             ccl_global int *indices,
                             ccl_global int *num_indices,
                             int kernel_index)
{
  ccl_gpu_kernel_lambda(INTEGRATOR_STATE(state, path, queued_kernel) == kernel_index,
                        int kernel_index);
  ccl_gpu_kernel_lambda_pass.kernel_index = kernel_index;

  gpu_parallel_active_index_array(num_states, indices, num_indices, ccl_gpu_kernel_lambda_pass);
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel_threads(GPU_PARALLEL_ACTIVE_INDEX_DEFAULT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_queued_shadow_paths_array,
                             int num_states,
                             ccl_global int *indices,
                             ccl_global int *num_indices,
                             int kernel_index)
{
  ccl_gpu_kernel_lambda(INTEGRATOR_STATE(state, shadow_path, queued_kernel) == kernel_index,
                        int kernel_index);
  ccl_gpu_kernel_lambda_pass.kernel_index = kernel_index;

  gpu_parallel_active_index_array(num_states, indices, num_indices, ccl_gpu_kernel_lambda_pass);
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel_threads(GPU_PARALLEL_ACTIVE_INDEX_DEFAULT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_active_paths_array,
                             int num_states,
                             ccl_global int *indices,
                             ccl_global int *num_indices)
{
  ccl_gpu_kernel_lambda(INTEGRATOR_STATE(state, path, queued_kernel) != 0);

  gpu_parallel_active_index_array(num_states, indices, num_indices, ccl_gpu_kernel_lambda_pass);
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel_threads(GPU_PARALLEL_ACTIVE_INDEX_DEFAULT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_terminated_paths_array,
                             int num_states,
                             ccl_global int *indices,
                             ccl_global int *num_indices,
                             int indices_offset)
{
  ccl_gpu_kernel_lambda(INTEGRATOR_STATE(state, path, queued_kernel) == 0);

  gpu_parallel_active_index_array(
      num_states, indices + indices_offset, num_indices, ccl_gpu_kernel_lambda_pass);
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel_threads(GPU_PARALLEL_ACTIVE_INDEX_DEFAULT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_terminated_shadow_paths_array,
                             int num_states,
                             ccl_global int *indices,
                             ccl_global int *num_indices,
                             int indices_offset)
{
  ccl_gpu_kernel_lambda(INTEGRATOR_STATE(state, shadow_path, queued_kernel) == 0);

  gpu_parallel_active_index_array(
      num_states, indices + indices_offset, num_indices, ccl_gpu_kernel_lambda_pass);
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel_threads(GPU_PARALLEL_SORTED_INDEX_DEFAULT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_sorted_paths_array,
                             int num_states,
                             int num_states_limit,
                             ccl_global int *indices,
                             ccl_global int *num_indices,
                             ccl_global int *key_counter,
                             ccl_global int *key_prefix_sum,
                             int kernel_index)
{
  ccl_gpu_kernel_lambda((INTEGRATOR_STATE(state, path, queued_kernel) == kernel_index) ?
                            INTEGRATOR_STATE(state, path, shader_sort_key) :
                            GPU_PARALLEL_SORTED_INDEX_INACTIVE_KEY,
                        int kernel_index);
  ccl_gpu_kernel_lambda_pass.kernel_index = kernel_index;

  const uint state_index = ccl_gpu_global_id_x();
  gpu_parallel_sorted_index_array(state_index,
                                  num_states,
                                  num_states_limit,
                                  indices,
                                  num_indices,
                                  key_counter,
                                  key_prefix_sum,
                                  ccl_gpu_kernel_lambda_pass);
}
ccl_gpu_kernel_postfix

/* oneAPI Verizon needs the local_mem accessor in the arguments. */
#ifdef __KERNEL_ONEAPI__
ccl_gpu_kernel_threads(GPU_PARALLEL_SORT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_sort_bucket_pass,
                             int num_states,
                             int partition_size,
                             int num_states_limit,
                             ccl_global int *indices,
                             int kernel_index,
                             sycl::local_accessor<int> &local_mem)
#else
ccl_gpu_kernel_threads(GPU_PARALLEL_SORT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_sort_bucket_pass,
                             int num_states,
                             int partition_size,
                             int num_states_limit,
                             ccl_global int *indices,
                             int kernel_index)
#endif
{
#if defined(__KERNEL_LOCAL_ATOMIC_SORT__)
  ccl_global ushort *d_queued_kernel = (ccl_global ushort *)
                                           kernel_integrator_state.path.queued_kernel;
  ccl_global uint *d_shader_sort_key = (ccl_global uint *)
                                           kernel_integrator_state.path.shader_sort_key;
  ccl_global int *key_offsets = (ccl_global int *)
                                    kernel_integrator_state.sort_partition_key_offsets;

#  ifdef __KERNEL_METAL__
  int max_shaders = context.launch_params_metal.data.max_shaders;
#  endif

#  ifdef __KERNEL_ONEAPI__
  /* Metal backend doesn't have these particular ccl_gpu_* defines and current kernel code
   * uses metal_*, we need the below to be compatible with these kernels. */
  int max_shaders = ((ONEAPIKernelContext *)kg)->__data->max_shaders;
  int metal_local_id = ccl_gpu_thread_idx_x;
  int metal_local_size = ccl_gpu_block_dim_x;
  int metal_grid_id = ccl_gpu_block_idx_x;
  ccl_gpu_shared int *threadgroup_array = local_mem.get_pointer();
#  endif

  gpu_parallel_sort_bucket_pass(num_states,
                                partition_size,
                                max_shaders,
                                kernel_index,
                                d_queued_kernel,
                                d_shader_sort_key,
                                key_offsets,
                                (ccl_gpu_shared int *)threadgroup_array,
                                metal_local_id,
                                metal_local_size,
                                metal_grid_id);
#endif
}
ccl_gpu_kernel_postfix

/* oneAPI version needs the local_mem accessor in the arguments. */
#ifdef __KERNEL_ONEAPI__
ccl_gpu_kernel_threads(GPU_PARALLEL_SORT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_sort_write_pass,
                             int num_states,
                             int partition_size,
                             int num_states_limit,
                             ccl_global int *indices,
                             int kernel_index,
                             sycl::local_accessor<int> &local_mem)
#else
ccl_gpu_kernel_threads(GPU_PARALLEL_SORT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_sort_write_pass,
                             int num_states,
                             int partition_size,
                             int num_states_limit,
                             ccl_global int *indices,
                             int kernel_index)
#endif

{
#if defined(__KERNEL_LOCAL_ATOMIC_SORT__)
  ccl_global ushort *d_queued_kernel = (ccl_global ushort *)
                                           kernel_integrator_state.path.queued_kernel;
  ccl_global uint *d_shader_sort_key = (ccl_global uint *)
                                           kernel_integrator_state.path.shader_sort_key;
  ccl_global int *key_offsets = (ccl_global int *)
                                    kernel_integrator_state.sort_partition_key_offsets;

#  ifdef __KERNEL_METAL__
  int max_shaders = context.launch_params_metal.data.max_shaders;
#  endif

#  ifdef __KERNEL_ONEAPI__
  /* Metal backend doesn't have these particular ccl_gpu_* defines and current kernel code
   * uses metal_*, we need the below to be compatible with these kernels. */
  int max_shaders = ((ONEAPIKernelContext *)kg)->__data->max_shaders;
  int metal_local_id = ccl_gpu_thread_idx_x;
  int metal_local_size = ccl_gpu_block_dim_x;
  int metal_grid_id = ccl_gpu_block_idx_x;
  ccl_gpu_shared int *threadgroup_array = local_mem.get_pointer();
#  endif

  gpu_parallel_sort_write_pass(num_states,
                               partition_size,
                               max_shaders,
                               kernel_index,
                               num_states_limit,
                               indices,
                               d_queued_kernel,
                               d_shader_sort_key,
                               key_offsets,
                               (ccl_gpu_shared int *)threadgroup_array,
                               metal_local_id,
                               metal_local_size,
                               metal_grid_id);
#endif
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel_threads(GPU_PARALLEL_ACTIVE_INDEX_DEFAULT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_compact_paths_array,
                             int num_states,
                             ccl_global int *indices,
                             ccl_global int *num_indices,
                             int num_active_paths)
{
  ccl_gpu_kernel_lambda((state >= num_active_paths) &&
                            (INTEGRATOR_STATE(state, path, queued_kernel) != 0),
                        int num_active_paths);
  ccl_gpu_kernel_lambda_pass.num_active_paths = num_active_paths;

  gpu_parallel_active_index_array(num_states, indices, num_indices, ccl_gpu_kernel_lambda_pass);
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel_threads(GPU_PARALLEL_SORTED_INDEX_DEFAULT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_compact_states,
                             ccl_global const int *active_terminated_states,
                             const int active_states_offset,
                             const int terminated_states_offset,
                             const int work_size)
{
  const int global_index = ccl_gpu_global_id_x();

  if (ccl_gpu_kernel_within_bounds(global_index, work_size)) {
    const int from_state = active_terminated_states[active_states_offset + global_index];
    const int to_state = active_terminated_states[terminated_states_offset + global_index];

    ccl_gpu_kernel_call(integrator_state_move(NULL, to_state, from_state));
  }
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel_threads(GPU_PARALLEL_ACTIVE_INDEX_DEFAULT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_compact_shadow_paths_array,
                             int num_states,
                             ccl_global int *indices,
                             ccl_global int *num_indices,
                             int num_active_paths)
{
  ccl_gpu_kernel_lambda((state >= num_active_paths) &&
                            (INTEGRATOR_STATE(state, shadow_path, queued_kernel) != 0),
                        int num_active_paths);
  ccl_gpu_kernel_lambda_pass.num_active_paths = num_active_paths;

  gpu_parallel_active_index_array(num_states, indices, num_indices, ccl_gpu_kernel_lambda_pass);
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel_threads(GPU_PARALLEL_SORTED_INDEX_DEFAULT_BLOCK_SIZE)
    ccl_gpu_kernel_signature(integrator_compact_shadow_states,
                             ccl_global const int *active_terminated_states,
                             const int active_states_offset,
                             const int terminated_states_offset,
                             const int work_size)
{
  const int global_index = ccl_gpu_global_id_x();

  if (ccl_gpu_kernel_within_bounds(global_index, work_size)) {
    const int from_state = active_terminated_states[active_states_offset + global_index];
    const int to_state = active_terminated_states[terminated_states_offset + global_index];

    ccl_gpu_kernel_call(integrator_shadow_state_move(NULL, to_state, from_state));
  }
}
ccl_gpu_kernel_postfix

ccl_gpu_kernel_threads(GPU_PARALLEL_PREFIX_SUM_DEFAULT_BLOCK_SIZE) ccl_gpu_kernel_signature(
    prefix_sum, ccl_global int *counter, ccl_global int *prefix_sum, int num_values)
{
  gpu_parallel_prefix_sum(ccl_gpu_global_id_x(), counter, prefix_sum, num_values);
}
ccl_gpu_kernel_postfix
