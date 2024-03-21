/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(integrator_shadow_catcher_count_possible_splits,
                             int num_states,
                             ccl_global uint *num_possible_splits)
{
  const int state = ccl_gpu_global_id_x();

  bool can_split = false;

  if (state < num_states) {
    can_split = ccl_gpu_kernel_call(kernel_shadow_catcher_path_can_split(nullptr, state));
  }

  /* NOTE: All threads specified in the mask must execute the intrinsic. */
  const auto can_split_mask = ccl_gpu_ballot(can_split);
  const int lane_id = ccl_gpu_thread_idx_x % ccl_gpu_warp_size;
  if (lane_id == 0) {
    atomic_fetch_and_add_uint32(num_possible_splits, popcount(can_split_mask));
  }
}
ccl_gpu_kernel_postfix
