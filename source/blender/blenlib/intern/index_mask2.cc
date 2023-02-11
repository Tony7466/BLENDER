/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_index_mask2.hh"

namespace blender::index_mask {

std::array<int16_t, max_chunk_size> build_static_offsets_array()
{
  std::array<int16_t, max_chunk_size> data;
  for (int16_t i = 0; i < max_chunk_size; i++) {
    data[i] = i;
  }
  return data;
}

}  // namespace blender::index_mask
