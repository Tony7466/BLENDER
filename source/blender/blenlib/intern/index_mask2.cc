/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_index_mask2.hh"
#include "BLI_strict_flags.h"
#include "BLI_task.hh"
#include "BLI_timeit.hh"

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

template<typename T>
static void split_sorted_indices_by_chunk_recursive(const Span<T> indices,
                                                    const int64_t offset,
                                                    Vector<IndexRange> &r_chunks)
{
  if (indices.is_empty()) {
    return;
  }
  const T first_index = indices.first();
  const T last_index = indices.last();
  const int64_t first_chunk_index = index_to_chunk_index(first_index);
  const int64_t last_chunk_index = index_to_chunk_index(last_index);
  if (first_chunk_index == last_chunk_index) {
    r_chunks.append_as(offset, indices.size());
    return;
  }
  const int64_t middle_chunk_index = (first_chunk_index + last_chunk_index + 1) / 2;
  const int64_t split_value = middle_chunk_index * max_chunk_size - 1;
  const int64_t left_split_size = std::upper_bound(indices.begin(), indices.end(), split_value) -
                                  indices.begin();
  split_sorted_indices_by_chunk_recursive(indices.take_front(left_split_size), offset, r_chunks);
  split_sorted_indices_by_chunk_recursive(
      indices.drop_front(left_split_size), offset + left_split_size, r_chunks);
}

template<typename T> Vector<IndexRange> split_sorted_indices_by_chunk(const Span<T> indices)
{
  BLI_assert(std::is_sorted(indices.begin(), indices.end()));
  SCOPED_TIMER_AVERAGED(__func__);
  Vector<IndexRange> chunks;
  split_sorted_indices_by_chunk_recursive(indices, 0, chunks);
  return chunks;
}

template<typename T>
IndexMask from_unique_sorted_indices(const Span<T> indices, LinearAllocator<> & /*allocator*/)
{
  Vector<IndexRange> split_ranges = split_sorted_indices_by_chunk(indices);
  return {};
}

template IndexMask from_unique_sorted_indices(const Span<int>, LinearAllocator<> &);

}  // namespace blender::index_mask
