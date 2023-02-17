/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask2.hh"
#include "BLI_strict_flags.h"
#include "BLI_task.hh"
#include "BLI_timeit.hh"

namespace blender::index_mask {

std::array<int16_t, chunk_capacity> build_static_indices_array()
{
  std::array<int16_t, chunk_capacity> data;
  for (int16_t i = 0; i < chunk_capacity; i++) {
    data[size_t(i)] = i;
  }
  return data;
}

const IndexMask &get_static_index_mask_for_min_size(const int64_t min_size)
{
  static constexpr int64_t size_shift = 30;
  static constexpr int64_t max_size = (1 << size_shift);
  static constexpr int64_t chunks_num = max_size / chunk_capacity;

  BLI_assert(min_size <= max_size);
  UNUSED_VARS_NDEBUG(min_size);

  static IndexMask static_mask = []() {
    static Array<Chunk> chunks_array(chunks_num);
    static Array<int64_t> chunk_ids_array(chunks_num);
    static Array<int64_t> chunk_sizes_cumulative(chunks_num + 1);

    static const int16_t *static_offsets = get_static_indices_array().data();
    static const int16_t static_cumulative_segment_sizes[2] = {0, chunk_capacity};

    threading::parallel_for(IndexRange(chunks_num), 1024, [&](const IndexRange range) {
      for (const int64_t i : range) {
        Chunk &chunk = chunks_array[i];
        chunk.segments_num = 1;
        chunk.indices_by_segment = &static_offsets;
        chunk.cumulative_segment_sizes = static_cumulative_segment_sizes;

        chunk_ids_array[i] = i;
        chunk_sizes_cumulative[i] = i * chunk_capacity;
      }
    });
    chunk_sizes_cumulative.last() = max_size;

    IndexMask mask;
    IndexMaskData &mask_data = mask.data_for_inplace_construction();
    mask_data.chunks_num = chunks_num;
    mask_data.indices_num = max_size;
    mask_data.chunks = chunks_array.data();
    mask_data.chunk_ids = chunk_ids_array.data();
    mask_data.chunk_sizes_cumulative = chunk_sizes_cumulative.data();
    mask_data.begin_it.segment_i = 0;
    mask_data.begin_it.index_in_segment = 0;
    mask_data.end_it.segment_i = 0;
    mask_data.end_it.index_in_segment = chunk_capacity;
    return mask;
  }();
  return static_mask;
}

std::ostream &operator<<(std::ostream &stream, const IndexMask &mask)
{
  Array<int64_t> indices(mask.size());
  unique_sorted_indices::from_index_mask<int64_t>(mask, indices);
  Vector<unique_sorted_indices::RangeOrSpanVariant<int64_t>> segments;
  unique_sorted_indices::split_to_ranges_and_spans<int64_t>(indices, 16, segments);
  std::cout << "(Size: " << mask.size() << " | ";
  for (const unique_sorted_indices::RangeOrSpanVariant<int64_t> &segment : segments) {
    if (std::holds_alternative<IndexRange>(segment)) {
      const IndexRange range = std::get<IndexRange>(segment);
      std::cout << range;
    }
    else {
      const Span<int64_t> segment_indices = std::get<Span<int64_t>>(segment);
      std::cout << "[";
      for (const int64_t index : segment_indices) {
        std::cout << index << ",";
      }
      std::cout << "]";
    }
    std::cout << ", ";
  }
  std::cout << ")";
  return stream;
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
  const int64_t first_chunk_id = index_to_chunk_id(first_index);
  const int64_t last_chunk_id = index_to_chunk_id(last_index);
  if (first_chunk_id == last_chunk_id) {
    r_chunks.append_as(offset, indices.size());
    return;
  }
  const int64_t middle_chunk_index = (first_chunk_id + last_chunk_id + 1) / 2;
  const int64_t split_value = middle_chunk_index * chunk_capacity - 1;
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
int64_t split_to_ranges_and_spans(const Span<T> indices,
                                  const int64_t range_threshold,
                                  Vector<RangeOrSpanVariant<T>> &r_parts)
{
  BLI_assert(range_threshold >= 1);
  const int64_t old_parts_num = r_parts.size();
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
  return r_parts.size() - old_parts_num;
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
  MutableSpan<int64_t> chunk_ids = main_allocator.allocate_array<int64_t>(chunks_num);

  static const int16_t *static_offsets = get_static_indices_array().data();

  /* TODO: Use that again to avoid some allocations. */
  [[maybe_unused]] const Chunk full_chunk_template = IndexMask(chunk_capacity).data().chunks[0];

  threading::parallel_for(split_ranges.index_range(), 32, [&](const IndexRange slice) {
    Vector<RangeOrSpanVariant<T>> segments_in_chunks;
    Vector<int64_t> segments_per_chunk_cumulative;
    segments_per_chunk_cumulative.reserve(slice.size() + 1);
    segments_per_chunk_cumulative.append(0);
    int64_t index_allocations_num = 0;

    for (const int64_t i : slice) {
      const IndexRange range_for_chunk = split_ranges[i];
      const Span<T> indices_in_chunk = indices.slice(range_for_chunk);
      BLI_assert(!indices_in_chunk.is_empty());

      const int64_t chunk_id = index_to_chunk_id(int64_t(indices_in_chunk[0]));
      BLI_assert(chunk_id == index_to_chunk_id(int64_t(indices_in_chunk.last())));
      Chunk &chunk = chunks[i];
      chunk_ids[i] = chunk_id;

      const int16_t segments_in_chunk_num = int16_t(
          split_to_ranges_and_spans(indices_in_chunk, 16, segments_in_chunks));
      BLI_assert(segments_in_chunk_num > 0);
      segments_per_chunk_cumulative.append(segments_per_chunk_cumulative.last() +
                                           segments_in_chunk_num);

      for (const int64_t segment_i :
           segments_in_chunks.index_range().take_back(segments_in_chunk_num)) {
        const RangeOrSpanVariant<T> &segment = segments_in_chunks[segment_i];
        if (std::holds_alternative<IndexRange>(segment)) {
          /* No extra allocations necessary because static memory is used. */
        }
        else {
          const Span<T> indices_in_segment = std::get<Span<T>>(segment);
          index_allocations_num += indices_in_segment.size();
        }
      }

      chunk.segments_num = segments_in_chunk_num;
    }

    auto take_front_and_drop = [](auto &span, const int64_t n) {
      auto front = span.take_front(n);
      BLI_assert(front.size() == n);
      span = span.drop_front(n);
      return front;
    };

    LinearAllocator<> &allocator = allocator_by_thread.local();
    MutableSpan<int16_t> remaining_indices = allocator.allocate_array<int16_t>(
        index_allocations_num);
    MutableSpan<const int16_t *> remaining_indices_by_segment =
        allocator.allocate_array<const int16_t *>(segments_in_chunks.size());
    MutableSpan<int16_t> remaining_cumulative_segment_sizes = allocator.allocate_array<int16_t>(
        segments_in_chunks.size() + slice.size());

    const OffsetIndices<int64_t> segments_by_chunk = segments_per_chunk_cumulative.as_span();

    for (const int64_t i : IndexRange(slice.size())) {
      const IndexRange segments_in_chunk = segments_by_chunk[i];
      const int16_t segments_num = int16_t(segments_in_chunk.size());
      Chunk &chunk = chunks[slice[i]];
      const int64_t chunk_offset = chunk_ids[slice[i]] * chunk_capacity;

      MutableSpan<const int16_t *> indices_by_segment = take_front_and_drop(
          remaining_indices_by_segment, segments_num);
      MutableSpan<int16_t> cumulative_segment_sizes = take_front_and_drop(
          remaining_cumulative_segment_sizes, segments_num + 1);

      int64_t cumulative_size = 0;
      for (const int64_t segment_i : IndexRange(segments_num)) {
        const RangeOrSpanVariant<T> &segment = segments_in_chunks[segments_in_chunk[segment_i]];
        cumulative_segment_sizes[segment_i] = int16_t(cumulative_size);

        if (std::holds_alternative<IndexRange>(segment)) {
          const IndexRange range_in_segment = std::get<IndexRange>(segment);
          indices_by_segment[segment_i] = static_offsets +
                                          (range_in_segment.first() - chunk_offset);
          cumulative_size += range_in_segment.size();
        }
        else {
          const Span<T> indices_in_segment = std::get<Span<T>>(segment);
          MutableSpan<int16_t> new_indices = take_front_and_drop(remaining_indices,
                                                                 indices_in_segment.size());
          for (const int64_t index_in_segment : new_indices.index_range()) {
            new_indices[index_in_segment] = int16_t(indices_in_segment[index_in_segment] -
                                                    chunk_offset);
          }
          indices_by_segment[segment_i] = new_indices.data();
          cumulative_size += indices_in_segment.size();
        }
      }
      cumulative_segment_sizes[segments_num] = int16_t(cumulative_size);

      chunk.indices_by_segment = indices_by_segment.data();
      chunk.cumulative_segment_sizes = cumulative_segment_sizes.data();
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
  mask_data.chunk_ids = chunk_ids.data();
  mask_data.chunk_sizes_cumulative = chunk_sizes_cumulative.data();
  mask_data.begin_it = {0, 0};
  mask_data.end_it = chunks.last().end_iterator();
  return mask;

  return {};
}

template<typename T> void from_index_mask(const IndexMask &mask, MutableSpan<T> r_indices)
{
  BLI_assert(mask.size() == r_indices.size());
  int64_t current_i = 0;
  mask.foreach_index([&](const int64_t index) mutable {
    r_indices[current_i] = T(index);
    current_i++;
  });
}

template Vector<IndexRange> split_by_chunk(const Span<int> indices);
template IndexMask to_index_mask(const Span<int>, ResourceScope &);
template void from_index_mask(const IndexMask &mask, MutableSpan<int> r_indices);
template int64_t split_to_ranges_and_spans(const Span<int> indices,
                                           const int64_t range_threshold,
                                           Vector<std::variant<IndexRange, Span<int>>> &r_parts);

}  // namespace unique_sorted_indices

void do_benchmark(const int64_t total);
void do_benchmark(const int64_t total)
{
  const IndexRange range(total);
  IndexMask mask(total);

  Array<int8_t> data(total, 0);

  for ([[maybe_unused]] const int64_t iteration : IndexRange(5)) {
    {
      SCOPED_TIMER_AVERAGED("mask ");
      threading::parallel_for(range, 1e4, [&](const IndexRange subrange) {
        const IndexMask submask = mask.slice(subrange);
        submask.foreach_span_or_range([&](const auto indices) {
          int8_t *data_ = data.data();
          for (const int64_t i : indices) {
            data_[i]++;
          }
        });
      });
    }
    {
      SCOPED_TIMER_AVERAGED("range");
      threading::parallel_for(range, 1e4, [&](const IndexRange subrange) {
        int8_t *data_ = data.data();
        for (const int64_t i : subrange) {
          data_[i]++;
        }
      });
    }
    // {
    //   SCOPED_TIMER_AVERAGED("mask span");
    //   mask.foreach_span([&](const OffsetSpan<int64_t, int16_t> span) {
    //     int *data_ = data.data();
    //     for (const int64_t i : span) {
    //       data_[i]++;
    //     }
    //   });
    // }
  }
}

}  // namespace blender::index_mask
