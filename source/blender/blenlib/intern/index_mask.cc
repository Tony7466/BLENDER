/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <mutex>

#include "BLI_array.hh"
#include "BLI_bit_vector.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask.hh"
#include "BLI_set.hh"
#include "BLI_strict_flags.h"
#include "BLI_task.hh"
#include "BLI_timeit.hh"
#include "BLI_virtual_array.hh"

namespace blender::index_mask {

std::array<int16_t, max_segment_size> build_static_indices_array()
{
  std::array<int16_t, max_segment_size> data;
  for (int16_t i = 0; i < max_segment_size; i++) {
    data[size_t(i)] = i;
  }
  return data;
}

const IndexMask &get_static_index_mask_for_min_size(const int64_t min_size)
{
  static constexpr int64_t size_shift = 30;
  static constexpr int64_t max_size = (1 << size_shift);
  static constexpr int64_t segments_num = max_size / max_segment_size;

  BLI_assert(min_size <= max_size);
  UNUSED_VARS_NDEBUG(min_size);

  static IndexMask static_mask = []() {
    static Array<const int16_t *> indices_by_segment(segments_num);
    static Array<int64_t> segment_offsets(segments_num + 1);

    static const int16_t *static_offsets = get_static_indices_array().data();

    threading::isolate_task([&]() {
      threading::parallel_for(IndexRange(segments_num), 1024, [&](const IndexRange range) {
        for (const int64_t segment_i : range) {
          indices_by_segment[segment_i] = static_offsets;
          segment_offsets[segment_i] = segment_i * max_segment_size;
        }
      });
    });
    segment_offsets.last() = max_size;

    IndexMask mask;
    IndexMaskData &data = mask.data_for_inplace_construction();
    data.indices_num = max_size;
    data.segments_num = segments_num;
    data.indices_by_segment = indices_by_segment.data();
    data.segment_offsets = segment_offsets.data();
    data.cumulative_segment_sizes = segment_offsets.data();
    data.begin_index_in_segment = 0;
    data.end_index_in_segment = max_segment_size;

    return mask;
  }();
  return static_mask;
}

template<typename T>
static void split_by_aligned_segment_recursive(const Span<T> indices,
                                               const int64_t offset,
                                               Vector<IndexRange> &r_result)
{
  if (indices.is_empty()) {
    return;
  }
  const T first_index = indices.first();
  const T last_index = indices.last();
  const int64_t first_segment_id = first_index >> max_segment_size_shift;
  const int64_t last_segment_id = last_index >> max_segment_size_shift;
  if (first_segment_id == last_segment_id) {
    r_result.append_as(offset, indices.size());
    return;
  }
  const int64_t middle_chunk_index = (first_segment_id + last_segment_id + 1) / 2;
  const int64_t split_value = middle_chunk_index * max_segment_size - 1;
  const int64_t left_split_size = std::upper_bound(indices.begin(), indices.end(), split_value) -
                                  indices.begin();
  split_by_aligned_segment_recursive(indices.take_front(left_split_size), offset, r_result);
  split_by_aligned_segment_recursive(
      indices.drop_front(left_split_size), offset + left_split_size, r_result);
}

template<typename T> Vector<IndexRange> split_indices_by_aligned_segment(const Span<T> indices)
{
  BLI_assert(std::is_sorted(indices.begin(), indices.end()));
  Vector<IndexRange> result;
  /* This can be too low in some cases. */
  result.reserve((indices.size() >> max_segment_size_shift) + 1);
  split_by_aligned_segment_recursive(indices, 0, result);
  return result;
}

template Vector<IndexRange> split_indices_by_aligned_segment(Span<int>);

std::ostream &operator<<(std::ostream &stream, const IndexMask &mask)
{
  Array<int64_t> indices(mask.size());
  mask.to_indices<int64_t>(indices);
  Vector<std::variant<IndexRange, Span<int64_t>>> segments;
  unique_sorted_indices::split_to_ranges_and_spans<int64_t>(indices, 16, segments);
  std::cout << "(Size: " << mask.size() << " | ";
  for (const std::variant<IndexRange, Span<int64_t>> &segment : segments) {
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

void IndexMask::foreach_span_impl(const FunctionRef<void(OffsetSpan<int64_t, int16_t>)> fn) const
{
  for (const int64_t segment_i : IndexRange(data_.segments_num)) {
    const OffsetSpan<int64_t, int16_t> segment = this->segment(segment_i);
    fn(segment);
  }
}

IndexMask IndexMask::slice(const int64_t start, const int64_t size) const
{
  if (size == 0) {
    return {};
  }
  const RawMaskIterator first_it = this->index_to_iterator(start);
  const RawMaskIterator last_it = this->index_to_iterator(start + size - 1);

  IndexMask sliced = *this;
  sliced.data_.indices_num = size;
  sliced.data_.segments_num = last_it.segment_i - first_it.segment_i + 1;
  sliced.data_.indices_by_segment += first_it.segment_i;
  sliced.data_.segment_offsets += first_it.segment_i;
  sliced.data_.cumulative_segment_sizes += first_it.segment_i;
  sliced.data_.begin_index_in_segment = first_it.index_in_segment;
  sliced.data_.end_index_in_segment = last_it.index_in_segment + 1;
  return sliced;
}

IndexMask IndexMask::slice_and_offset(const IndexRange range,
                                      const int64_t offset,
                                      IndexMaskMemory &memory) const
{
  return this->slice_and_offset(range.start(), range.size(), offset, memory);
}

IndexMask IndexMask::slice_and_offset(const int64_t start,
                                      const int64_t size,
                                      const int64_t offset,
                                      IndexMaskMemory &memory) const
{
  if (size == 0) {
    return {};
  }
  if (std::optional<IndexRange> range = this->to_range()) {
    return range->slice(start, size).shift(offset);
  }
  const IndexMask sliced_mask = this->slice(start, size);
  if (offset == 0) {
    return sliced_mask;
  }
  if (std::optional<IndexRange> range = sliced_mask.to_range()) {
    return range->shift(offset);
  }
  MutableSpan<int64_t> new_segment_offsets = memory.allocate_array<int64_t>(data_.segments_num);
  for (const int64_t i : IndexRange(data_.segments_num)) {
    new_segment_offsets[i] = data_.segment_offsets[i] + offset;
  }
  IndexMask offset_mask = *this;
  offset_mask.data_.segment_offsets = new_segment_offsets.data();
  return offset_mask;
}

IndexMask IndexMask::complement(const IndexRange universe, IndexMaskMemory &memory) const
{
  return IndexMask::from_predicate(universe, GrainSize(512), memory, [&](const int64_t index) {
    return !this->contains(index);
  });
}

template<typename T>
IndexMask IndexMask::from_indices(const Span<T> indices, IndexMaskMemory &memory)
{
  if (indices.is_empty()) {
    return {};
  }
  return IndexMask::from_predicate(
      IndexRange(indices.first(), indices.last() - indices.first() + 1),
      GrainSize(512),
      memory,
      [&](const int64_t index) {
        return std::binary_search(indices.begin(), indices.end(), T(index));
      });
}

IndexMask IndexMask::from_bits(const BitSpan bits, IndexMaskMemory &memory)
{
  return IndexMask::from_bits(bits.index_range(), bits, memory);
}

IndexMask IndexMask::from_bits(const IndexMask &universe,
                               const BitSpan bits,
                               IndexMaskMemory &memory)
{
  return IndexMask::from_predicate(universe, GrainSize(1024), memory, [bits](const int64_t index) {
    return bits[index].test();
  });
}

IndexMask IndexMask::from_bools(Span<bool> bools, IndexMaskMemory &memory)
{
  return IndexMask::from_bools(bools.index_range(), bools, memory);
}

IndexMask IndexMask::from_bools(const VArray<bool> &bools, IndexMaskMemory &memory)
{
  return IndexMask::from_bools(bools.index_range(), bools, memory);
}

IndexMask IndexMask::from_bools(const IndexMask &universe,
                                Span<bool> bools,
                                IndexMaskMemory &memory)
{
  return IndexMask::from_predicate(
      universe, GrainSize(1024), memory, [bools](const int64_t index) { return bools[index]; });
}

IndexMask IndexMask::from_bools(const IndexMask &universe,
                                const VArray<bool> &bools,
                                IndexMaskMemory &memory)
{
  const CommonVArrayInfo info = bools.common_info();
  if (info.type == CommonVArrayInfo::Type::Single) {
    return *static_cast<const bool *>(info.data) ? universe : IndexMask();
  }
  if (info.type == CommonVArrayInfo::Type::Span) {
    const Span<bool> span(static_cast<const bool *>(info.data), bools.size());
    return IndexMask::from_bools(universe, span, memory);
  }
  return IndexMask::from_predicate(
      universe, GrainSize(512), memory, [&](const int64_t index) { return bools[index]; });
}

template<typename T> void IndexMask::to_indices(MutableSpan<T> r_indices) const
{
  BLI_assert(this->size() == r_indices.size());
  this->foreach_index_optimized(
      GrainSize(1024),
      [&](const int64_t i, const int64_t mask_i) mutable { r_indices[mask_i] = T(i); });
}

void IndexMask::to_bits(MutableBitSpan r_bits) const
{
  BLI_assert(r_bits.size() >= this->min_array_size());
  r_bits.reset_all();
  this->foreach_span_or_range([&](const auto mask_segment) {
    if constexpr (std::is_same_v<std::decay_t<decltype(mask_segment)>, IndexRange>) {
      const IndexRange range = mask_segment;
      r_bits.slice(range).set_all();
    }
    else {
      for (const int64_t i : mask_segment) {
        r_bits[i].set();
      }
    }
  });
}

void IndexMask::to_bools(MutableSpan<bool> r_bools) const
{
  BLI_assert(r_bools.size() >= this->min_array_size());
  r_bools.fill(false);
  this->foreach_index_optimized(GrainSize(2048), [&](const int64_t i) { r_bools[i] = true; });
}

Vector<IndexRange> IndexMask::to_ranges() const
{
  Vector<IndexRange> ranges;
  this->foreach_range([&](const IndexRange range) { ranges.append(range); });
  return ranges;
}

Vector<IndexRange> IndexMask::to_ranges_invert(const IndexRange universe) const
{
  IndexMaskMemory memory;
  return this->complement(universe, memory).to_ranges();
}

void IndexMask::to_ranges_and_spans_impl(Vector<IndexRange> &r_ranges,
                                         Vector<OffsetSpan<int64_t, int16_t>> &r_spans) const
{
  const IndexRangeChecker range_checker;
  this->foreach_span_or_range([&](const auto segment) {
    if constexpr (std::is_same_v<std::decay_t<decltype(segment)>, IndexRange>) {
      r_ranges.append(segment);
    }
    else {
      r_spans.append(segment);
    }
  });
}

IndexMask IndexMask::from_predicate_impl(
    const IndexMask &universe,
    const GrainSize grain_size,
    IndexMaskMemory &memory,
    const FunctionRef<int64_t(OffsetSpan<int64_t, int16_t> indices, int16_t *r_true_indices)>
        filter_indices)
{
  UNUSED_VARS(grain_size);

  if (universe.is_empty()) {
    return {};
  }

  const Span<int16_t> static_indices = get_static_indices_array();

  Vector<OffsetSpan<int64_t, int16_t>> segments(universe.data_.segments_num);

  auto process_segment = [&](const int64_t segment_i, LinearAllocator<> &allocator) {
    const OffsetSpan<int64_t, int16_t> universe_segment = universe.segment(segment_i);

    std::array<int16_t, max_segment_size> indices_array;
    const int64_t true_indices_num = filter_indices(universe_segment, indices_array.data());
    if (true_indices_num == 0) {
      return;
    }
    const Span<int16_t> true_indices{indices_array.data(), true_indices_num};
    if (const std::optional<IndexRange> range = unique_sorted_indices::non_empty_as_range_try(
            true_indices))
    {
      segments.append_as(universe_segment.offset(), static_indices.slice(*range));
    }
    else {
      segments.append_as(universe_segment.offset(), allocator.construct_array_copy(true_indices));
    }
  };

  for (const int64_t segment_i : IndexRange(universe.data_.segments_num)) {
    process_segment(segment_i, memory);
  }

  segments.remove_if(
      [](const OffsetSpan<int64_t, int16_t> segment) { return segment.is_empty(); });

  const int64_t segments_num = segments.size();
  if (segments_num == 0) {
    return {};
  }

  MutableSpan<const int16_t *> indices_by_segment = memory.allocate_array<const int16_t *>(
      segments_num);
  MutableSpan<int64_t> segment_offsets = memory.allocate_array<int64_t>(segments_num);
  MutableSpan<int64_t> cumulative_segment_sizes = memory.allocate_array<int64_t>(segments_num + 1);

  cumulative_segment_sizes[0] = 0;
  for (const int64_t segment_i : segments.index_range()) {
    const OffsetSpan<int64_t, int16_t> segment = segments[segment_i];
    indices_by_segment[segment_i] = segment.base_span().data();
    segment_offsets[segment_i] = segment.offset();
    cumulative_segment_sizes[segment_i + 1] = cumulative_segment_sizes[segment_i] + segment.size();
  }

  IndexMask mask;
  IndexMaskData &data = mask.data_for_inplace_construction();
  data.indices_num = cumulative_segment_sizes.last();
  data.segments_num = segments_num;
  data.indices_by_segment = indices_by_segment.data();
  data.segment_offsets = segment_offsets.data();
  data.cumulative_segment_sizes = cumulative_segment_sizes.data();
  data.begin_index_in_segment = 0;
  data.end_index_in_segment = segments.last().size();
  return mask;
}

std::optional<RawMaskIterator> IndexMask::find(const int64_t query_index) const
{
  if (this->is_empty()) {
    return std::nullopt;
  }
  if (query_index < this->first()) {
    return std::nullopt;
  }
  if (query_index > this->last()) {
    return std::nullopt;
  }

  const int64_t segment_i = std::upper_bound(data_.segment_offsets,
                                             data_.segment_offsets + data_.segments_num,
                                             query_index) -
                            data_.segment_offsets - 1;
  const OffsetSpan<int64_t, int16_t> segment = this->segment(segment_i);
  const Span<int16_t> local_segment = segment.base_span();
  const int64_t local_query_index = query_index - segment.offset();
  if (local_query_index > local_segment.last()) {
    return std::nullopt;
  }
  const int64_t index_in_segment = std::lower_bound(local_segment.begin(),
                                                    local_segment.end(),
                                                    local_query_index) -
                                   local_segment.begin();
  BLI_assert(index_in_segment < local_segment.size());
  if (local_segment[index_in_segment] != local_query_index) {
    return std::nullopt;
  }
  return RawMaskIterator{segment_i, int16_t(index_in_segment)};
}

bool IndexMask::contains(const int64_t query_index) const
{
  return this->find(query_index).has_value();
}

template IndexMask IndexMask::from_indices(Span<int32_t>, IndexMaskMemory &);
template IndexMask IndexMask::from_indices(Span<int64_t>, IndexMaskMemory &);
template void IndexMask::to_indices(MutableSpan<int32_t>) const;
template void IndexMask::to_indices(MutableSpan<int64_t>) const;

void do_benchmark(const int64_t total);
void do_benchmark(const int64_t /*total*/) {}

}  // namespace blender::index_mask
