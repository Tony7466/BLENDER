/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

ccl_gpu_kernel(GPU_KERNEL_BLOCK_NUM_THREADS, GPU_KERNEL_MAX_REGISTERS)
    ccl_gpu_kernel_signature(integrator_init_from_bake,
                             ccl_global KernelWorkTile *tiles,
                             const int num_tiles,
                             ccl_global float *render_buffer,
                             const int max_tile_work_size)
{
  const int work_index = ccl_gpu_global_id_x();

  if (work_index >= max_tile_work_size * num_tiles) {
    return;
  }

  const int tile_index = work_index / max_tile_work_size;
  const int tile_work_index = work_index - tile_index * max_tile_work_size;

  ccl_global const KernelWorkTile *tile = &tiles[tile_index];

  if (tile_work_index >= tile->work_size) {
    return;
  }

  const int state = tile->path_index_offset + tile_work_index;

  uint x, y, sample;
  ccl_gpu_kernel_call(get_work_pixel(tile, tile_work_index, &x, &y, &sample));

  ccl_gpu_kernel_call(
      integrator_init_from_bake(nullptr, state, tile, render_buffer, x, y, sample));
}
ccl_gpu_kernel_postfix
