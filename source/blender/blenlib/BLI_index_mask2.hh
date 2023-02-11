/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_vector.hh"

namespace blender::index_mask {

/* Chunks contain up to 2^14 = 16384 indices. */
static constexpr int64_t chunk_size_shift = 14;
static constexpr int64_t chunk_mask_low = (1 << chunk_size_shift) - 1;
static constexpr int64_t chunk_mask_high = ~chunk_mask_low;
static constexpr int64_t max_chunk_size = (1 << chunk_size_shift);

std::array<int16_t, max_chunk_size> build_static_offsets_array();

inline const std::array<int16_t, max_chunk_size> &get_static_offsets_array()
{
  alignas(64) static const std::array<int16_t, max_chunk_size> data = build_static_offsets_array();
  return data;
}

}  // namespace blender::index_mask
