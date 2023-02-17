/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <array>
#include <optional>
#include <variant>

#include "BLI_index_range.hh"
#include "BLI_linear_allocator.hh"
#include "BLI_offset_indices.hh"
#include "BLI_offset_span.hh"
#include "BLI_resource_scope.hh"
#include "BLI_span.hh"

namespace blender {
namespace index_mask {

class IndexMask;

/* Chunks contain up to 2^14 = 16384 indices. */
static constexpr int64_t chunk_size_shift = 14;
static constexpr int64_t chunk_mask_low = (1 << chunk_size_shift) - 1;
static constexpr int64_t chunk_mask_high = ~chunk_mask_low;
static constexpr int64_t chunk_capacity = (1 << chunk_size_shift);

std::array<int16_t, chunk_capacity> build_static_indices_array();
const IndexMask &get_static_index_mask_for_min_size(const int64_t min_size);

struct RawChunkIterator {
  int16_t segment_i;
  int16_t index_in_segment;
};

struct RawMaskIterator {
  int64_t chunk_i;
  RawChunkIterator chunk_it;
};

/**
 * A #Chunk contains an ordered list of segments. Each segment is an array of 16-bit integers.
 */
struct Chunk {
  int16_t segments_num;
  const int16_t **indices_by_segment;
  const int16_t *cumulative_segment_sizes;

  RawChunkIterator end_iterator() const;
  OffsetIndices<int16_t> segment_offsets() const;
  RawChunkIterator index_to_iterator(const int16_t index) const;
  int16_t iterator_to_index(const RawChunkIterator &it) const;
  int16_t size() const;
  int16_t segment_size(const int16_t segment_i) const;
};

struct IndexMaskData {
  int64_t chunks_num;
  int64_t indices_num;
  const Chunk *chunks;
  const int64_t *chunk_ids;
  const int64_t *cumulative_chunk_sizes;
  RawChunkIterator begin_it;
  RawChunkIterator end_it;
};

struct IndexMaskSegment {
  int64_t start_index;
  OffsetSpan<int64_t, int16_t> indices;
};

class IndexMask {
 private:
  IndexMaskData data_;

 public:
  IndexMask();
  IndexMask(int64_t size);
  IndexMask(IndexRange range);

  int64_t size() const;
  OffsetIndices<int64_t> chunk_offsets() const;
  int64_t first() const;
  int64_t last() const;
  int64_t min_array_size() const;

  RawMaskIterator index_to_iterator(const int64_t index) const;
  int64_t iterator_to_index(const RawMaskIterator &it) const;

  IndexMask slice(IndexRange range) const;

  template<typename Fn> void foreach_segment(Fn &&fn) const;
  template<typename Fn> void foreach_span(Fn &&fn) const;
  template<typename Fn> void foreach_index(Fn &&fn) const;
  template<typename Fn> void foreach_span_or_range(Fn &&fn) const;

  const IndexMaskData &data() const;
  IndexMaskData &data_for_inplace_construction();
};

std::ostream &operator<<(std::ostream &stream, const IndexMask &mask);

namespace unique_sorted_indices {

template<typename T> Vector<IndexRange> split_by_chunk(Span<T> indices);

template<typename T> IndexMask to_index_mask(Span<T> indices, ResourceScope &scope);
template<typename T> void from_index_mask(const IndexMask &mask, MutableSpan<T> r_indices);

template<typename T> using RangeOrSpanVariant = std::variant<IndexRange, Span<T>>;

template<typename T>
int64_t split_to_ranges_and_spans(Span<T> indices,
                                  int64_t range_threshold,
                                  Vector<RangeOrSpanVariant<T>> &r_parts);

template<typename T> inline bool non_empty_is_range(const Span<T> indices)
{
  BLI_assert(!indices.is_empty());
  return indices.last() - indices.first() == indices.size() - 1;
}

template<typename T> inline IndexRange non_empty_as_range(const Span<T> indices)
{
  BLI_assert(!indices.is_empty());
  BLI_assert(non_empty_is_range(indices));
  return IndexRange(indices.first(), indices.size());
}

template<typename T> inline int64_t find_size_of_next_range(const Span<T> indices)
{
  BLI_assert(!indices.is_empty());
  return std::lower_bound(
             indices.begin(),
             indices.end(),
             0,
             [indices, offset = indices[0]](const T &element, const int64_t /*dummy*/) {
               const int64_t element_index = &element - indices.begin();
               return element - offset == element_index;
             }) -
         indices.begin();
}

template<typename T>
inline int64_t find_size_until_next_range(const Span<T> indices, const int64_t min_range_size)
{
  BLI_assert(!indices.is_empty());
  int64_t current_range_size = 1;
  int64_t last_value = indices[0];
  for (const int64_t i : indices.index_range().drop_front(1)) {
    const T current_value = indices[i];
    if (current_value == last_value + 1) {
      current_range_size++;
      if (current_range_size >= min_range_size) {
        return i - min_range_size + 1;
      }
    }
    else {
      current_range_size = 1;
    }
    last_value = current_value;
  }
  return indices.size();
}

}  // namespace unique_sorted_indices

/* -------------------------------------------------------------------- */
/** \name Inline Utilities
 * \{ */

inline const std::array<int16_t, chunk_capacity> &get_static_indices_array()
{
  alignas(64) static const std::array<int16_t, chunk_capacity> data = build_static_indices_array();
  return data;
}

inline int64_t index_to_chunk_id(const int64_t i)
{
  return i >> chunk_size_shift;
}

inline int64_t size_to_chunk_num(const int64_t size)
{
  return (size + chunk_capacity - 1) >> chunk_size_shift;
}

class IndexRangeChecker {
 private:
  const int16_t *data_;
  uintptr_t adder_;

 public:
  IndexRangeChecker() : data_(get_static_indices_array().data())
  {
    adder_ = std::numeric_limits<uintptr_t>::max() -
             uintptr_t(get_static_indices_array().data() + chunk_capacity);
  }

  bool check(const Span<int16_t> indices) const;

  bool check_static(const Span<int16_t> indices) const;
};

inline bool IndexRangeChecker::check(const Span<int16_t> indices) const
{
  return indices.last() - indices.first() == indices.size() - 1;
}

inline bool IndexRangeChecker::check_static(const Span<int16_t> indices) const
{
  const uintptr_t indices_ptr = uintptr_t(indices.data());
  return indices_ptr + adder_ >
         std::numeric_limits<uintptr_t>::max() - chunk_capacity * sizeof(int16_t);
}

/* -------------------------------------------------------------------- */
/** \name #Chunk Inline Methods
 * \{ */

inline RawChunkIterator Chunk::index_to_iterator(const int16_t index) const
{
  BLI_assert(index >= 0);
  BLI_assert(index < this->segment_offsets().total_size());
  RawChunkIterator it;
  it.segment_i = this->segment_offsets().find_range_index(index);
  it.index_in_segment = index - this->cumulative_segment_sizes[it.segment_i];
  return it;
}

inline int16_t Chunk::iterator_to_index(const RawChunkIterator &it) const
{
  BLI_assert(it.segment_i >= 0);
  BLI_assert(it.segment_i < this->segments_num);
  BLI_assert(it.index_in_segment >= 0);
  BLI_assert(it.index_in_segment < this->segment_offsets().size(it.segment_i));
  return this->cumulative_segment_sizes[it.segment_i] + it.index_in_segment;
}

/* -------------------------------------------------------------------- */
/** \name #RawChunkIterator Inline Methods
 * \{ */

inline bool operator!=(const RawChunkIterator &a, const RawChunkIterator &b)
{
  return a.index_in_segment != b.index_in_segment || a.segment_i != b.segment_i;
}

/* -------------------------------------------------------------------- */
/** \name #RawMaskIterator Inline Methods
 * \{ */

inline bool operator!=(const RawMaskIterator &a, const RawMaskIterator &b)
{
  return a.chunk_it != b.chunk_it || a.chunk_i != b.chunk_i;
}

inline bool operator==(const RawMaskIterator &a, const RawMaskIterator &b)
{
  return !(a != b);
}

/* -------------------------------------------------------------------- */
/** \name #IndexMask Inline Methods
 * \{ */

inline IndexMask::IndexMask()
{
  static constexpr int64_t cumulative_sizes_for_empty_mask[1] = {0};

  data_.chunks_num = 0;
  data_.indices_num = 0;
  data_.chunks = nullptr;
  data_.chunk_ids = nullptr;
  data_.cumulative_chunk_sizes = cumulative_sizes_for_empty_mask;
  data_.begin_it.segment_i = 0;
  data_.begin_it.index_in_segment = 0;
  data_.end_it.segment_i = 0;
  data_.end_it.index_in_segment = 0;
}

inline IndexMask::IndexMask(const int64_t size)
{
  *this = get_static_index_mask_for_min_size(size);
  data_.chunks_num = size_to_chunk_num(size);
  data_.indices_num = size;
  data_.end_it.index_in_segment = (size == chunk_capacity) ? chunk_capacity :
                                                             size & chunk_mask_low;
}

inline IndexMask::IndexMask(const IndexRange range)
{
  if (range.is_empty()) {
    return;
  }
  *this = get_static_index_mask_for_min_size(range.one_after_last());

  const int64_t first_chunk_id = index_to_chunk_id(range.first());
  const int64_t last_chunk_id = index_to_chunk_id(range.last());

  data_.chunks_num = last_chunk_id - first_chunk_id + 1;
  data_.indices_num = range.size();
  data_.chunks -= first_chunk_id;
  data_.chunk_ids -= first_chunk_id;
  data_.cumulative_chunk_sizes -= first_chunk_id;
  data_.begin_it.segment_i = 0;
  data_.begin_it.index_in_segment = range.first() & chunk_mask_low;
  data_.end_it.segment_i = 0;
  data_.end_it.index_in_segment = range.one_after_last() & chunk_mask_low;
}

inline int64_t IndexMask::size() const
{
  return data_.indices_num;
}

inline OffsetIndices<int64_t> IndexMask::chunk_offsets() const
{
  return Span<int64_t>(data_.cumulative_chunk_sizes, data_.chunks_num + 1);
}

inline int64_t IndexMask::first() const
{
  BLI_assert(data_.indices_num > 0);
  return chunk_capacity * data_.chunk_ids[0] +
         data_.chunks[0]
             .indices_by_segment[data_.begin_it.segment_i][data_.begin_it.index_in_segment];
}

inline int64_t IndexMask::last() const
{
  BLI_assert(data_.indices_num > 0);
  const int64_t chunk_i = data_.chunks_num - 1;
  return chunk_capacity * data_.chunk_ids[chunk_i] +
         data_.chunks[chunk_i]
             .indices_by_segment[data_.end_it.segment_i][data_.end_it.index_in_segment - 1];
}

inline int64_t IndexMask::min_array_size() const
{
  if (data_.indices_num == 0) {
    return 0;
  }
  return this->last() + 1;
}

inline RawMaskIterator IndexMask::index_to_iterator(const int64_t index) const
{
  BLI_assert(index >= 0);
  BLI_assert(index < data_.indices_num);
  RawMaskIterator it;
  const int16_t begin_index = data_.chunks[0].iterator_to_index(data_.begin_it);
  it.chunk_i = this->chunk_offsets().find_range_index(index + begin_index +
                                                      data_.cumulative_chunk_sizes[0]);
  const Chunk &chunk = data_.chunks[it.chunk_i];
  it.chunk_it = chunk.index_to_iterator((index + begin_index) & chunk_mask_low);
  return it;
}

inline int64_t IndexMask::iterator_to_index(const RawMaskIterator &it) const
{
  BLI_assert(it.chunk_i >= 0);
  BLI_assert(it.chunk_i < data_.chunks_num);
  const int16_t begin_index = data_.chunks[0].iterator_to_index(data_.begin_it);
  return data_.cumulative_chunk_sizes[it.chunk_i] - data_.cumulative_chunk_sizes[0] - begin_index;
}

inline IndexMask IndexMask::slice(const IndexRange range) const
{
  if (range.is_empty()) {
    return {};
  }
  const RawMaskIterator first_it = this->index_to_iterator(range.first());
  const RawMaskIterator last_it = this->index_to_iterator(range.last());

  IndexMask sliced;
  sliced.data_.chunks_num = last_it.chunk_i - first_it.chunk_i + 1;
  sliced.data_.indices_num = range.size();
  sliced.data_.chunks = data_.chunks + first_it.chunk_i;
  sliced.data_.chunk_ids = data_.chunk_ids + first_it.chunk_i;
  sliced.data_.cumulative_chunk_sizes = data_.cumulative_chunk_sizes + first_it.chunk_i;
  sliced.data_.begin_it = first_it.chunk_it;
  sliced.data_.end_it.segment_i = last_it.chunk_it.segment_i;
  sliced.data_.end_it.index_in_segment = last_it.chunk_it.index_in_segment + 1;
  return sliced;
}

inline const IndexMaskData &IndexMask::data() const
{
  return data_;
}

inline IndexMaskData &IndexMask::data_for_inplace_construction()
{
  return const_cast<IndexMaskData &>(data_);
}

inline RawChunkIterator Chunk::end_iterator() const
{
  RawChunkIterator data;
  if (this->segments_num > 0) {
    data.segment_i = this->segments_num - 1;
    data.index_in_segment = this->segment_offsets().size(this->segments_num - 1);
  }
  else {
    data.segment_i = 0;
    data.index_in_segment = 0;
  }
  return data;
}

inline OffsetIndices<int16_t> Chunk::segment_offsets() const
{
  return Span<int16_t>(this->cumulative_segment_sizes, this->segments_num + 1);
}

inline int16_t Chunk::size() const
{
  return this->cumulative_segment_sizes[this->segments_num + 1] -
         this->cumulative_segment_sizes[0];
}

inline int16_t Chunk::segment_size(const int16_t segment_i) const
{
  return this->cumulative_segment_sizes[segment_i + 1] - this->cumulative_segment_sizes[segment_i];
}

template<typename Fn> inline void IndexMask::foreach_segment(Fn &&fn) const
{
  if (data_.indices_num == 0) {
    return;
  }

  int64_t chunk_i = 0;
  int64_t segment_i = data_.begin_it.segment_i;
  int16_t segment_drop_front = data_.begin_it.index_in_segment;
  const int16_t final_drop_back = data_.chunks[data_.chunks_num - 1].segment_size(
                                      data_.end_it.segment_i) -
                                  data_.end_it.index_in_segment;
  const int16_t final_segment_i = data_.end_it.segment_i;
  const int16_t final_segments_num = data_.end_it.segment_i + 1;

  int64_t counter = 0;
  while (chunk_i < data_.chunks_num) {
    const Chunk &chunk = data_.chunks[chunk_i];
    const int64_t chunk_id = data_.chunk_ids[chunk_i];
    const bool is_last_chunk = (chunk_i == data_.chunks_num - 1);
    const int16_t segments_num = is_last_chunk ? final_segments_num : chunk.segments_num;
    const int64_t offset = chunk_capacity * chunk_id;
    int16_t prev_cumulative_segment_size = chunk.cumulative_segment_sizes[segment_i];
    while (segment_i < segments_num) {
      const int16_t next_segment_i = segment_i + 1;
      const int16_t cumulative_segment_size = chunk.cumulative_segment_sizes[next_segment_i];
      const int16_t stored_segment_size = cumulative_segment_size - prev_cumulative_segment_size;
      const bool is_last_segment = is_last_chunk & (segment_i == final_segment_i);
      const int16_t segment_drop_back = is_last_segment * final_drop_back;
      const int16_t *indices_in_segment = chunk.indices_by_segment[segment_i] + segment_drop_front;
      const int16_t segment_size = stored_segment_size - segment_drop_front - segment_drop_back;
      const Span<int16_t> indices_span{indices_in_segment, segment_size};

      const IndexMaskSegment segment{counter, {offset, indices_span}};
      fn(segment);

      counter += segment_size;
      segment_drop_front = 0;
      segment_i = next_segment_i;
    }
    segment_i = 0;
    chunk_i++;
  }
}

template<typename Fn> inline void IndexMask::foreach_span(Fn &&fn) const
{
  this->foreach_segment([&](const IndexMaskSegment &segment) { fn(segment.indices); });
}

template<typename Fn> inline void IndexMask::foreach_index(Fn &&fn) const
{
  this->foreach_segment([&](const IndexMaskSegment &segment) {
    for (const int64_t index : segment.indices) {
      fn(index);
    }
  });
}

template<typename Fn> inline void IndexMask::foreach_span_or_range(Fn &&fn) const
{
  IndexRangeChecker is_index_mask;
  this->foreach_segment([&, is_index_mask](const IndexMaskSegment &segment) {
    const OffsetSpan<int64_t, int16_t> indices = segment.indices;
    if (is_index_mask.check(indices.base_span())) {
      fn(IndexRange(indices[0], indices.size()));
    }
    else {
      fn(indices);
    }
  });
}

}  // namespace index_mask

using index_mask::IndexMask;

}  // namespace blender
