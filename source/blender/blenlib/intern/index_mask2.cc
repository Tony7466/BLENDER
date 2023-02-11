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

namespace unique_sorted_indices {

template<typename T>
static void split_by_chunk_recursive(const Span<T> indices,
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
  split_by_chunk_recursive(indices.take_front(left_split_size), offset, r_chunks);
  split_by_chunk_recursive(
      indices.drop_front(left_split_size), offset + left_split_size, r_chunks);
}

template<typename T> Vector<IndexRange> split_by_chunk(const Span<T> indices)
{
  BLI_assert(std::is_sorted(indices.begin(), indices.end()));
  SCOPED_TIMER_AVERAGED(__func__);
  Vector<IndexRange> chunks;
  /* This can be too low in some cases, but it's never too large. */
  chunks.reserve(size_to_chunk_num(indices.size()));
  split_by_chunk_recursive(indices, 0, chunks);
  return chunks;
}

template<typename T>
void split_to_ranges_and_spans(const Span<T> indices,
                               const T range_threshold,
                               Vector<std::variant<IndexRange, Span<T>>> &r_parts)
{
  BLI_assert(range_threshold >= 1);
  Span<T> remaining_indices = indices;
  while (!remaining_indices.is_empty()) {
    if (non_empty_is_range(remaining_indices)) {
      /* All remaining indices are range. */
      r_parts.append(non_empty_as_range(remaining_indices));
      break;
    }
    if (non_empty_is_range(remaining_indices.take_front(range_threshold))) {
      /* Next segment is a range. Now find the place where the range ends. */
      const int64_t segment_size = find_size_of_next_range(remaining_indices);
      r_parts.append(IndexRange(remaining_indices[0], segment_size));
      remaining_indices = remaining_indices.drop_front(segment_size);
      continue;
    }
    /* Next segment is just indices. Now find the place where the next range starts. */
    const int64_t segment_size = find_size_until_next_range(remaining_indices, range_threshold);
    r_parts.append(remaining_indices.take_front(segment_size));
    remaining_indices = remaining_indices.drop_back(segment_size);
  }
}

template<typename T>
IndexMask to_index_mask(const Span<T> indices, LinearAllocator<> & /*allocator*/)
{
  if (indices.is_empty()) {
    return {};
  }
  // if (auto full_range = try_convert_unique_sorted_indices_as_range(indices)) {
  //   return IndexMask(*full_range);
  // }
  const Vector<IndexRange> split_ranges = split_by_chunk(indices);
  // const int64_t chunks_num = split_ranges.size();
  // MutableSpan<Chunk> chunks = allocator.allocate_array<Chunk>(chunks_num);
  // MutableSpan<int64_t> chunk_offsets = allocator.allocate_array<int64_t>(chunks_num);
  // MutableSpan<int64_t> chunk_sizes_cumulative = allocator.allocate_array<int64_t>(chunks_num +
  // 1);

  // static const int16_t *static_offsets = get_static_indices_array().data();

  // threading::parallel_for(split_ranges.index_range(), 128, [&](const IndexRange slice) {
  //   for (const int64_t i : slice) {
  //     const IndexRange range_for_chunk = split_ranges[i];
  //     const Span<T> indices_in_chunk = indices.slice(range_for_chunk);
  //     BLI_assert(!indices_in_chunk.is_empty());
  //     const int64_t chunk_i = index_to_chunk_index(int64_t(indices_in_chunk[0]));
  //     BLI_assert(chunk_i == index_to_chunk_index(int64_t(indices_in_chunk.last())));
  //     Chunk &chunk = chunks[chunk_i];
  //   }
  // });

  return {};
}

template IndexMask to_index_mask(const Span<int>, LinearAllocator<> &);

}  // namespace unique_sorted_indices

}  // namespace blender::index_mask
