/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_enumerable_thread_specific.hh"
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
    static const int16_t static_segment_sizes_cumulative[2] = {0, max_chunk_size};

    threading::parallel_for(IndexRange(chunks_num), 1024, [&](const IndexRange range) {
      for (const int64_t i : range) {
        Chunk &chunk = chunks_array[i];
        chunk.segments_num = 1;
        chunk.segment_indices = &static_offsets;
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
  Vector<IndexRange> chunks;
  /* This can be too low in some cases, but it's never too large. */
  chunks.reserve(size_to_chunk_num(indices.size()));
  split_by_chunk_recursive(indices, 0, chunks);
  return chunks;
}

template<typename T>
void split_to_ranges_and_spans(const Span<T> indices,
                               const int64_t range_threshold,
                               Vector<RangeOrSpanVariant<T>> &r_parts)
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
    remaining_indices = remaining_indices.drop_front(segment_size);
  }
}

template<typename T> IndexMask to_index_mask(const Span<T> indices, ResourceScope &scope)
{
  if (indices.is_empty()) {
    return {};
  }
  if (non_empty_is_range(indices)) {
    return non_empty_as_range(indices);
  }
  const Vector<IndexRange> split_ranges = split_by_chunk(indices);
  const int64_t chunks_num = split_ranges.size();

  LinearAllocator<> &main_allocator = scope.linear_allocator();
  auto &allocator_by_thread =
      scope.construct<threading::EnumerableThreadSpecific<LinearAllocator<>>>();

  MutableSpan<Chunk> chunks = main_allocator.allocate_array<Chunk>(chunks_num);
  MutableSpan<int64_t> chunk_offsets = main_allocator.allocate_array<int64_t>(chunks_num);

  static const int16_t *static_offsets = get_static_indices_array().data();

  const Chunk full_chunk_template = IndexMask(max_chunk_size).data().chunks[0];

  threading::parallel_for(split_ranges.index_range(), 32, [&](const IndexRange slice) {
    LinearAllocator<> &allocator = allocator_by_thread.local();
    Vector<RangeOrSpanVariant<T>> segments;

    for (const int64_t i : slice) {
      const IndexRange range_for_chunk = split_ranges[i];
      const Span<T> indices_in_chunk = indices.slice(range_for_chunk);
      BLI_assert(!indices_in_chunk.is_empty());

      const int64_t chunk_i = index_to_chunk_index(int64_t(indices_in_chunk[0]));
      BLI_assert(chunk_i == index_to_chunk_index(int64_t(indices_in_chunk.last())));
      Chunk &chunk = chunks[i];
      const int64_t chunk_offset = max_chunk_size * chunk_i;
      chunk_offsets[i] = chunk_offset;

      if (indices_in_chunk.size() == max_chunk_size) {
        chunk = full_chunk_template;
        continue;
      }

      segments.clear();
      split_to_ranges_and_spans(indices_in_chunk, 16, segments);
      BLI_assert(!segments.is_empty());

      const int16_t segments_num = int16_t(segments.size());
      MutableSpan<const int16_t *> segment_indices_pointers =
          allocator.allocate_array<const int16_t *>(segments_num);
      MutableSpan<int16_t> segment_sizes_cumulative = allocator.allocate_array<int16_t>(
          segments_num + 1);

      int64_t cumulative_size = 0;
      for (const int64_t segment_i : segments.index_range()) {
        const RangeOrSpanVariant<T> &segment = segments[segment_i];
        segment_sizes_cumulative[segment_i] = int16_t(cumulative_size);
        if (std::holds_alternative<IndexRange>(segment)) {
          const IndexRange range_in_segment = std::get<IndexRange>(segment);
          segment_indices_pointers[segment_i] = static_offsets +
                                                (range_in_segment.first() - chunk_offset);
          cumulative_size += range_in_segment.size();
        }
        else {
          const Span<T> indices_in_segment = std::get<Span<T>>(segment);
          MutableSpan<int16_t> new_indices = allocator.allocate_array<int16_t>(
              indices_in_segment.size());
          for (const int64_t index_in_segment : new_indices.index_range()) {
            new_indices[index_in_segment] = int16_t(indices_in_segment[index_in_segment] -
                                                    chunk_offset);
          }
          segment_indices_pointers[segment_i] = new_indices.data();
          cumulative_size += indices_in_segment.size();
        }
      }

      segment_sizes_cumulative[segments_num] = int16_t(cumulative_size);

      chunk.segments_num = segments_num;
      chunk.segment_indices = segment_indices_pointers.data();
      chunk.segment_sizes_cumulative = segment_sizes_cumulative.data();
    }
  });

  MutableSpan<int64_t> chunk_sizes_cumulative = main_allocator.allocate_array<int64_t>(chunks_num +
                                                                                       1);
  int64_t cumulative_size = 0;
  for (const int64_t i : chunks.index_range()) {
    chunk_sizes_cumulative[i] = cumulative_size;
    cumulative_size += chunks[i].size();
  }
  chunk_sizes_cumulative.last() = cumulative_size;

  IndexMask mask;
  IndexMaskData &mask_data = mask.data_for_inplace_construction();
  mask_data.chunks_num = chunks_num;
  mask_data.indices_num = indices.size();
  mask_data.chunks = chunks.data();
  mask_data.chunk_offsets = chunk_offsets.data();
  mask_data.chunk_sizes_cumulative = chunk_sizes_cumulative.data();
  mask_data.begin_it = {0, 0};
  mask_data.end_it = chunks.last().end_data();
  return mask;

  return {};
}

template Vector<IndexRange> split_by_chunk(const Span<int> indices);
template IndexMask to_index_mask(const Span<int>, ResourceScope &);
template void split_to_ranges_and_spans(const Span<int> indices,
                                        const int64_t range_threshold,
                                        Vector<std::variant<IndexRange, Span<int>>> &r_parts);

}  // namespace unique_sorted_indices

}  // namespace blender::index_mask
