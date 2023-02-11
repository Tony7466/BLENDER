/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_vector.hh"

namespace blender::index_mask {

/* Chunks contain up to 2^14 = 16384 indices. */
static constexpr int64_t index_mask_chunk_shift = 14;
static constexpr int64_t index_mask_chunk_mask_low = (1 << index_mask_chunk_shift) - 1;
static constexpr int64_t index_mask_chunk_mask_high = ~index_mask_chunk_mask_low;
static constexpr int64_t max_index_mask_chunk_size = (1 << index_mask_chunk_shift);

inline const std::array<int16_t, max_index_mask_chunk_size> &get_static_offsets_array()
{
  static const std::array<int16_t, max_index_mask_chunk_size> data = []() {
    static std::array<int16_t, max_index_mask_chunk_size> data;
    for (int16_t i = 0; i < max_index_mask_chunk_size; i++) {
      data[i] = i;
    }
    return data;
  }();
  return data;
}

}  // namespace blender::index_mask
