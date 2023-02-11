/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_index_mask2.hh"
#include "BLI_strict_flags.h"
#include "BLI_task.hh"

namespace blender::index_mask {

std::array<int16_t, max_chunk_size> build_static_indices_array()
{
  std::array<int16_t, max_chunk_size> data;
  for (int16_t i = 0; i < max_chunk_size; i++) {
    data[size_t(i)] = i;
  }
  return data;
}

const IndexMask &get_static_index_mask_for_min_size(const int64_t min_size)
{
  static constexpr int64_t size_shift = 30;
  static constexpr int64_t max_size = (1 << size_shift);
  static constexpr int64_t chunks_num = max_size / max_chunk_size;

  BLI_assert(min_size <= max_size);
  UNUSED_VARS_NDEBUG(min_size);

  static IndexMask static_mask = []() {
    static Array<Chunk> chunks_array(chunks_num);
    static Array<int64_t> chunk_offsets_array(chunks_num);
    static Array<int64_t> chunk_sizes_cumulative(chunks_num + 1);

    static const int16_t *static_offsets = get_static_indices_array().data();
    static const int16_t static_segment_sizes[1] = {max_chunk_size};
    static const int16_t static_segment_sizes_cumulative[2] = {0, max_chunk_size};

    threading::parallel_for(IndexRange(chunks_num), 1024, [&](const IndexRange range) {
      for (const int64_t i : range) {
        Chunk &chunk = chunks_array[i];
        chunk.segments_num = 1;
        chunk.indices_num = max_chunk_size;
        chunk.segment_indices = &static_offsets;
        chunk.segment_sizes = static_segment_sizes;
        chunk.segment_sizes_cumulative = static_segment_sizes_cumulative;

        chunk_offsets_array[i] = i * max_chunk_size;
        chunk_sizes_cumulative[i] = i * max_chunk_size;
      }
    });
    chunk_sizes_cumulative.last() = max_size;

    IndexMask mask;
    IndexMaskData &mask_data = mask.data_for_inplace_construction();
    mask_data.chunks_num = chunks_num;
    mask_data.indices_num = max_size;
    mask_data.chunks = chunks_array.data();
    mask_data.chunk_offsets = chunk_offsets_array.data();
    mask_data.chunk_sizes_cumulative = chunk_sizes_cumulative.data();
    mask_data.begin_it.segment_index = 0;
    mask_data.begin_it.index_in_segment = 0;
    mask_data.end_it.segment_index = 0;
    mask_data.end_it.index_in_segment = max_chunk_size;
    return mask;
  }();
  return static_mask;
}

}  // namespace blender::index_mask
