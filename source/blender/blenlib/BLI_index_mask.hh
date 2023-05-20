/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <array>
#include <optional>
#include <variant>

#include "BLI_array.hh"
#include "BLI_bit_vector.hh"
#include "BLI_function_ref.hh"
#include "BLI_index_range.hh"
#include "BLI_linear_allocator.hh"
#include "BLI_offset_indices.hh"
#include "BLI_offset_span.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"
#include "BLI_unique_sorted_indices.hh"
#include "BLI_vector.hh"

namespace blender {

struct GrainSize {
  int64_t value;

  explicit constexpr GrainSize(const int64_t grain_size) : value(grain_size) {}
};

template<typename T> class VArray;

namespace index_mask {

class IndexMask;

static constexpr int64_t max_segment_size_shift = 14;
static constexpr int64_t max_segment_size = (1 << max_segment_size_shift); /* 16384 */
static constexpr int64_t max_segment_size_mask_low = max_segment_size - 1;
static constexpr int64_t max_segment_size_mask_high = ~max_segment_size_mask_low;

std::array<int16_t, max_segment_size> build_static_indices_array();
const IndexMask &get_static_index_mask_for_min_size(const int64_t min_size);

struct RawMaskIterator {
  int64_t segment_i;
  int16_t index_in_segment;
};

struct IndexMaskData {
  int64_t indices_num;
  int64_t segments_num;
  const int16_t **indices_by_segment;
  const int64_t *segment_offsets;
  const int64_t *cumulative_segment_sizes;
  int64_t begin_index_in_segment;
  int64_t end_index_in_segment;
};

class IndexMaskMemory : public LinearAllocator<> {
};

class IndexMask {
 private:
  IndexMaskData data_;

 public:
  IndexMask();
  IndexMask(int64_t size);
  IndexMask(IndexRange range);

  int64_t size() const;
  bool is_empty() const;
  IndexRange index_range() const;
  int64_t first() const;
  int64_t last() const;
  int64_t min_array_size() const;
  std::optional<RawMaskIterator> find(const int64_t query_index) const;
  bool contains(const int64_t query_index) const;

  RawMaskIterator index_to_iterator(const int64_t index) const;
  int64_t iterator_to_index(const RawMaskIterator &it) const;

  IndexMask slice(IndexRange range) const;
  IndexMask slice(int64_t start, int64_t size) const;
  IndexMask slice_and_offset(IndexRange range,
                             const int64_t offset,
                             IndexMaskMemory &memory) const;
  IndexMask slice_and_offset(int64_t start,
                             int64_t size,
                             const int64_t offset,
                             IndexMaskMemory &memory) const;
  IndexMask complement(const IndexRange universe, IndexMaskMemory &memory) const;

  int64_t operator[](const int64_t i) const;
  int64_t operator[](const RawMaskIterator &it) const;

  int64_t segments_num() const;
  OffsetSpan<int64_t, int16_t> segment(const int64_t segment_i) const;

  template<typename Fn> void foreach_span(Fn &&fn) const;
  template<typename Fn> void foreach_range(Fn &&fn) const;
  template<typename Fn> void foreach_span_or_range(Fn &&fn) const;
  template<typename Fn> void foreach_index(Fn &&fn) const;
  template<typename Fn> void foreach_index_optimized(Fn &&fn) const;

  template<typename Fn> void foreach_index(GrainSize grain_size, Fn &&fn) const;
  template<typename Fn> void foreach_span(GrainSize grain_size, Fn &&fn) const;
  template<typename Fn> void foreach_span_or_range(GrainSize grain_size, Fn &&fn) const;
  template<typename Fn> void foreach_index_optimized(GrainSize grain_size, Fn &&fn) const;

  template<typename T> static IndexMask from_indices(Span<T> indices, IndexMaskMemory &memory);
  static IndexMask from_bits(BitSpan bits, IndexMaskMemory &memory);
  static IndexMask from_bits(const IndexMask &universe, BitSpan bits, IndexMaskMemory &memory);
  static IndexMask from_bools(Span<bool> bools, IndexMaskMemory &memory);
  static IndexMask from_bools(const VArray<bool> &bools, IndexMaskMemory &memory);
  static IndexMask from_bools(const IndexMask &universe,
                              Span<bool> bools,
                              IndexMaskMemory &memory);
  static IndexMask from_bools(const IndexMask &universe,
                              const VArray<bool> &bools,
                              IndexMaskMemory &memory);
  template<typename Fn>
  static IndexMask from_predicate(IndexRange universe,
                                  GrainSize grain_size,
                                  IndexMaskMemory &memory,
                                  Fn &&predicate);
  template<typename Fn>
  static IndexMask from_predicate(const IndexMask &universe,
                                  GrainSize grain_size,
                                  IndexMaskMemory &memory,
                                  Fn &&predicate);
  template<typename T, typename Fn>
  static void from_groups(const IndexMask &universe,
                          IndexMaskMemory &memory,
                          Fn &&get_group_index,
                          MutableSpan<IndexMask> r_masks);

  template<typename T> void to_indices(MutableSpan<T> r_indices) const;
  void to_bits(MutableBitSpan r_bits) const;
  void to_bools(MutableSpan<bool> r_bools) const;
  std::optional<IndexRange> to_range() const;
  Vector<IndexRange> to_ranges() const;
  Vector<IndexRange> to_ranges_invert(IndexRange universe) const;
  void to_ranges_and_spans(Vector<IndexRange> &r_ranges,
                           Vector<OffsetSpan<int64_t, int16_t>> &r_spans) const;

  const IndexMaskData &data() const;
  IndexMaskData &data_for_inplace_construction();

 private:
  void foreach_span_impl(FunctionRef<void(OffsetSpan<int64_t, int16_t>)> fn) const;
  void to_ranges_and_spans_impl(Vector<IndexRange> &r_ranges,
                                Vector<OffsetSpan<int64_t, int16_t>> &r_spans) const;
  static IndexMask from_predicate_impl(
      const IndexMask &universe,
      const GrainSize grain_size,
      IndexMaskMemory &memory,
      FunctionRef<int64_t(OffsetSpan<int64_t, int16_t> indices, int16_t *r_true_indices)>
          filter_indices);
};

class IndexMaskFromSegment : NonCopyable, NonMovable {
 public:
  int64_t segment_offset;
  const int16_t *segment_indices;
  std::array<int64_t, 2> cumulative_segment_sizes;
  IndexMask mask;

  IndexMaskFromSegment();
  void update(OffsetSpan<int64_t, int16_t> segment);
};

inline IndexMaskFromSegment::IndexMaskFromSegment()
{
  IndexMaskData &data = mask.data_for_inplace_construction();
  this->cumulative_segment_sizes[0] = 0;
  data.segments_num = 1;
  data.indices_by_segment = &this->segment_indices;
  data.segment_offsets = &this->segment_offset;
  data.cumulative_segment_sizes = this->cumulative_segment_sizes.data();
  data.begin_index_in_segment = 0;
}

inline void IndexMaskFromSegment::update(const OffsetSpan<int64_t, int16_t> segment)
{
  const Span<int16_t> indices = segment.base_span();
  BLI_assert(!indices.is_empty());
  BLI_assert(std::is_sorted(indices.begin(), indices.end()));
  BLI_assert(indices[0] >= 0);
  BLI_assert(indices.last() < max_segment_size);
  const int64_t indices_num = indices.size();

  IndexMaskData &data = mask.data_for_inplace_construction();
  this->segment_offset = segment.offset();
  this->segment_indices = indices.data();
  this->cumulative_segment_sizes[1] = int16_t(indices_num);
  data.indices_num = indices_num;
  data.end_index_in_segment = indices_num;
}

std::ostream &operator<<(std::ostream &stream, const IndexMask &mask);

/* -------------------------------------------------------------------- */
/** \name Inline Utilities
 * \{ */

inline const std::array<int16_t, max_segment_size> &get_static_indices_array()
{
  alignas(64) static const std::array<int16_t, max_segment_size> data =
      build_static_indices_array();
  return data;
}

template<typename T>
inline void masked_fill(MutableSpan<T> data, const T &value, const IndexMask &mask)
{
  mask.foreach_index_optimized([&](const int64_t i) { data[i] = value; });
}

class IndexRangeChecker {
 private:
  const int16_t *data_;
  uintptr_t adder_;

 public:
  IndexRangeChecker() : data_(get_static_indices_array().data())
  {
    adder_ = std::numeric_limits<uintptr_t>::max() -
             uintptr_t(get_static_indices_array().data() + max_segment_size);
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
         std::numeric_limits<uintptr_t>::max() - max_segment_size * sizeof(int16_t);
}

/* -------------------------------------------------------------------- */
/** \name #RawMaskIterator Inline Methods
 * \{ */

inline bool operator!=(const RawMaskIterator &a, const RawMaskIterator &b)
{
  return a.segment_i != b.segment_i || a.index_in_segment != b.index_in_segment;
}

inline bool operator==(const RawMaskIterator &a, const RawMaskIterator &b)
{
  return !(a != b);
}

/* -------------------------------------------------------------------- */
/** \name #IndexMask Inline Methods
 * \{ */

inline void init_empty_mask(IndexMaskData &data)
{
  static constexpr int64_t cumulative_sizes_for_empty_mask[1] = {0};

  data.indices_num = 0;
  data.segments_num = 0;
  data.cumulative_segment_sizes = cumulative_sizes_for_empty_mask;
  /* Intentionally leave some pointer uninitialized which must not be accessed on empty masks
   * anyway. */
}

inline IndexMask::IndexMask()
{
  init_empty_mask(data_);
}

inline IndexMask::IndexMask(const int64_t size)
{
  if (size == 0) {
    init_empty_mask(data_);
    return;
  }
  *this = get_static_index_mask_for_min_size(size);
  data_.indices_num = size;
  data_.segments_num = ((size + max_segment_size - 1) >> max_segment_size_shift);
  data_.begin_index_in_segment = 0;
  data_.end_index_in_segment = size - ((size - 1) & max_segment_size_mask_high);
}

inline IndexMask::IndexMask(const IndexRange range)
{
  if (range.is_empty()) {
    init_empty_mask(data_);
    return;
  }
  const int64_t one_after_last = range.one_after_last();
  *this = get_static_index_mask_for_min_size(one_after_last);

  const int64_t first_segment_i = range.first() >> max_segment_size_shift;
  const int64_t last_segment_i = range.last() >> max_segment_size_shift;

  data_.indices_num = range.size();
  data_.segments_num = last_segment_i - first_segment_i + 1;
  data_.indices_by_segment += first_segment_i;
  data_.segment_offsets += first_segment_i;
  data_.cumulative_segment_sizes += first_segment_i;
  data_.begin_index_in_segment = range.first() & max_segment_size_mask_low;
  data_.end_index_in_segment = one_after_last -
                               ((one_after_last - 1) & max_segment_size_mask_high);
}

inline int64_t IndexMask::size() const
{
  return data_.indices_num;
}

inline bool IndexMask::is_empty() const
{
  return data_.indices_num == 0;
}

inline IndexRange IndexMask::index_range() const
{
  return IndexRange(data_.indices_num);
}

inline int64_t IndexMask::first() const
{
  BLI_assert(data_.indices_num > 0);
  return data_.segment_offsets[0] + data_.indices_by_segment[0][data_.begin_index_in_segment];
}

inline int64_t IndexMask::last() const
{
  BLI_assert(data_.indices_num > 0);
  const int64_t last_segment_i = data_.segments_num - 1;
  return data_.segment_offsets[last_segment_i] +
         data_.indices_by_segment[last_segment_i][data_.end_index_in_segment - 1];
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
  const int64_t full_index = index + data_.cumulative_segment_sizes[0] +
                             data_.begin_index_in_segment;
  it.segment_i = -1 +
                 binary_search::find_predicate_begin(
                     data_.cumulative_segment_sizes,
                     data_.cumulative_segment_sizes + data_.segments_num + 1,
                     [&](const int64_t cumulative_size) { return cumulative_size > full_index; });
  it.index_in_segment = full_index - data_.cumulative_segment_sizes[it.segment_i];
  return it;
}

inline int64_t IndexMask::iterator_to_index(const RawMaskIterator &it) const
{
  BLI_assert(it.segment_i >= 0);
  BLI_assert(it.segment_i < data_.segments_num);
  BLI_assert(it.index_in_segment >= 0);
  BLI_assert(it.index_in_segment < data_.cumulative_segment_sizes[it.segment_i + 1] -
                                       data_.cumulative_segment_sizes[it.segment_i]);
  return it.index_in_segment + data_.cumulative_segment_sizes[it.segment_i] -
         data_.cumulative_segment_sizes[0] - data_.begin_index_in_segment;
}

inline int64_t IndexMask::operator[](const int64_t i) const
{
  const RawMaskIterator it = this->index_to_iterator(i);
  return (*this)[it];
}

inline int64_t IndexMask::operator[](const RawMaskIterator &it) const
{
  return data_.segment_offsets[it.segment_i] +
         data_.indices_by_segment[it.segment_i][it.index_in_segment];
}

inline int64_t IndexMask::segments_num() const
{
  return data_.segments_num;
}

inline OffsetSpan<int64_t, int16_t> IndexMask::segment(const int64_t segment_i) const
{
  BLI_assert(segment_i >= 0);
  BLI_assert(segment_i < data_.segments_num);
  const int64_t full_segment_size = data_.cumulative_segment_sizes[segment_i + 1] -
                                    data_.cumulative_segment_sizes[segment_i];
  const int64_t begin_index = (segment_i == 0) ? data_.begin_index_in_segment : 0;
  const int64_t end_index = (segment_i == data_.segments_num - 1) ? data_.end_index_in_segment :
                                                                    full_segment_size;
  const int64_t segment_size = end_index - begin_index;
  return OffsetSpan<int64_t, int16_t>{
      data_.segment_offsets[segment_i],
      {data_.indices_by_segment[segment_i] + begin_index, segment_size}};
}

inline IndexMask IndexMask::slice(const IndexRange range) const
{
  return this->slice(range.start(), range.size());
}

inline const IndexMaskData &IndexMask::data() const
{
  return data_;
}

inline IndexMaskData &IndexMask::data_for_inplace_construction()
{
  return const_cast<IndexMaskData &>(data_);
}

template<typename Fn>
constexpr bool has_mask_segment_and_start_parameter =
    std::is_invocable_r_v<void, Fn, OffsetSpan<int64_t, int16_t>, int64_t> ||
    std::is_invocable_r_v<void, Fn, IndexRange, int64_t>;

template<typename Fn> inline void IndexMask::foreach_index(Fn &&fn) const
{
  this->foreach_span([&](const OffsetSpan<int64_t, int16_t> indices, const int64_t start) {
    if constexpr (std::is_invocable_r_v<void, Fn, int64_t, int64_t>) {
      for (const int64_t i : indices.index_range()) {
        fn(indices[i], start + i);
      }
    }
    else {
      for (const int64_t index : indices) {
        fn(index);
      }
    }
  });
}

template<typename Fn>
#if (defined(__GNUC__) && !defined(__clang__))
[[gnu::optimize("-funroll-loops")]] [[gnu::optimize("O3")]]
#endif
inline void
foreach_index_in_range(const IndexRange range, Fn &&fn)
{
  const int64_t start = range.start();
  const int64_t end = range.one_after_last();
  for (int64_t i = start; i < end; i++) {
    fn(i);
  }
}

template<typename Fn>
#if (defined(__GNUC__) && !defined(__clang__))
[[gnu::optimize("-funroll-loops")]] [[gnu::optimize("O3")]]
#endif
inline void
foreach_index_in_range(const IndexRange range, const int64_t offset, Fn &&fn)
{
  const int64_t start = range.start();
  const int64_t end = range.one_after_last();
  for (int64_t i = start, mask_i = offset; i < end; i++, mask_i++) {
    fn(i, mask_i);
  }
}

template<typename Fn> inline void IndexMask::foreach_index_optimized(Fn &&fn) const
{
  this->foreach_span_or_range([&](const auto mask_segment, const int64_t start) {
    constexpr bool is_range = std::is_same_v<std::decay_t<decltype(mask_segment)>, IndexRange>;
    if constexpr (std::is_invocable_r_v<void, Fn, int64_t, int64_t>) {
      if constexpr (is_range) {
        foreach_index_in_range(mask_segment, start, fn);
      }
      else {
        for (const int64_t i : mask_segment.index_range()) {
          fn(mask_segment[i], start + i);
        }
      }
    }
    else {
      if constexpr (is_range) {
        foreach_index_in_range(mask_segment, fn);
      }
      else {
        for (const int64_t index : mask_segment) {
          fn(index);
        }
      }
    }
  });
}

template<typename Fn> inline void IndexMask::foreach_span_or_range(Fn &&fn) const
{
  IndexRangeChecker is_index_mask;
  this->foreach_span(
      [&, is_index_mask](const OffsetSpan<int64_t, int16_t> mask_segment, const int64_t start) {
        if (is_index_mask.check_static(mask_segment.base_span())) {
          const IndexRange range(mask_segment[0], mask_segment.size());
          if constexpr (has_mask_segment_and_start_parameter<Fn>) {
            fn(range, start);
          }
          else {
            fn(range);
          }
        }
        else {
          if constexpr (has_mask_segment_and_start_parameter<Fn>) {
            fn(mask_segment, start);
          }
          else {
            fn(mask_segment);
          }
        }
      });
}

template<typename Fn> inline void IndexMask::foreach_span(Fn &&fn) const
{
  if constexpr (has_mask_segment_and_start_parameter<Fn>) {
    this->foreach_span_impl(
        [&, counter = int64_t(0)](const OffsetSpan<int64_t, int16_t> mask_segment) mutable {
          fn(mask_segment, counter);
          counter += mask_segment.size();
        });
  }
  else {
    this->foreach_span_impl(fn);
  }
}

template<typename Fn> inline void IndexMask::foreach_range(Fn &&fn) const
{
  this->foreach_span([&](const OffsetSpan<int64_t, int16_t> indices, int64_t start) {
    Span<int16_t> base_indices = indices.base_span();
    while (!base_indices.is_empty()) {
      const int64_t next_range_size = unique_sorted_indices::find_size_of_next_range(base_indices);
      const IndexRange range(int64_t(base_indices[0]) + indices.offset(), next_range_size);
      if constexpr (has_mask_segment_and_start_parameter<Fn>) {
        fn(range, start);
      }
      else {
        fn(range);
      }
      start += next_range_size;
      base_indices = base_indices.drop_front(next_range_size);
    }
  });
}

template<typename Fn>
inline void IndexMask::foreach_index(const GrainSize grain_size, Fn &&fn) const
{
  threading::parallel_for(this->index_range(), grain_size.value, [&](const IndexRange range) {
    const IndexMask sub_mask = this->slice(range);
    sub_mask.foreach_index([&](const int64_t i, const int64_t i_in_mask) {
      if constexpr (std::is_invocable_r_v<void, Fn, int64_t, int64_t>) {
        fn(i, i_in_mask + range.start());
      }
      else {
        fn(i);
      }
    });
  });
}

template<typename Fn>
inline void IndexMask::foreach_span(const GrainSize grain_size, Fn &&fn) const
{
  threading::parallel_for(this->index_range(), grain_size.value, [&](const IndexRange range) {
    const IndexMask sub_mask = this->slice(range);
    sub_mask.foreach_span(
        [&](const OffsetSpan<int64_t, int16_t> mask_segment, const int64_t start) {
          if constexpr (has_mask_segment_and_start_parameter<Fn>) {
            fn(mask_segment, start + range.start());
          }
          else {
            fn(mask_segment);
          }
        });
  });
}

template<typename Fn>
inline void IndexMask::foreach_span_or_range(const GrainSize grain_size, Fn &&fn) const
{
  threading::parallel_for(this->index_range(), grain_size.value, [&](const IndexRange range) {
    const IndexMask sub_mask = this->slice(range);
    sub_mask.foreach_span_or_range([&](const auto mask_segment, const int64_t start) {
      if constexpr (has_mask_segment_and_start_parameter<Fn>) {
        fn(mask_segment, start + range.start());
      }
      else {
        fn(mask_segment);
      }
    });
  });
}

template<typename Fn>
inline void IndexMask::foreach_index_optimized(const GrainSize grain_size, Fn &&fn) const
{
  threading::parallel_for(this->index_range(), grain_size.value, [&](const IndexRange range) {
    const IndexMask sub_mask = this->slice(range);
    if constexpr (std::is_invocable_r_v<void, Fn, int64_t, int64_t>) {
      sub_mask.foreach_index_optimized(
          [&](const int64_t i, const int64_t i_in_mask) { fn(i, i_in_mask + range.start()); });
    }
    else {
      sub_mask.foreach_index_optimized(fn);
    }
  });
}

template<typename Fn>
inline IndexMask IndexMask::from_predicate(const IndexRange universe,
                                           const GrainSize grain_size,
                                           IndexMaskMemory &memory,
                                           Fn &&predicate)
{
  return IndexMask::from_predicate(IndexMask(universe), grain_size, memory, predicate);
}

template<typename Fn>
inline IndexMask IndexMask::from_predicate(const IndexMask &universe,
                                           const GrainSize grain_size,
                                           IndexMaskMemory &memory,
                                           Fn &&predicate)
{
  return IndexMask::from_predicate_impl(
      universe,
      grain_size,
      memory,
      [&](const OffsetSpan<int64_t, int16_t> indices, int16_t *__restrict r_true_indices) {
        int16_t *r_current = r_true_indices;
        const int16_t *in_end = indices.base_span().end();
        const int64_t offset = indices.offset();
        for (const int16_t *in_current = indices.base_span().data(); in_current < in_end;
             in_current++) {
          const int16_t local_index = *in_current;
          const int64_t global_index = int64_t(local_index) + offset;
          const bool condition = predicate(global_index);
          *r_current = local_index;
          /* Branchless conditional increment. */
          r_current += condition;
        }
        const int16_t true_indices_num = int16_t(r_current - r_true_indices);
        return true_indices_num;
      });
}

template<typename T, typename Fn>
void IndexMask::from_groups(const IndexMask &universe,
                            IndexMaskMemory &memory,
                            Fn &&get_group_index,
                            MutableSpan<IndexMask> r_masks)
{
  Array<Vector<T>> indices_by_group(r_masks.size());
  universe.foreach_index([&](const int64_t i) {
    const int group_index = get_group_index(i);
    indices_by_group[group_index].append(T(i));
  });
  for (const int64_t i : r_masks.index_range()) {
    r_masks[i] = IndexMask::from_indices<T>(indices_by_group[i], memory);
  }
}

std::optional<IndexRange> inline IndexMask::to_range() const
{
  if (data_.indices_num == 0) {
    return IndexRange{};
  }
  const int64_t first_index = this->first();
  const int64_t last_index = this->last();
  if (last_index - first_index == data_.indices_num - 1) {
    return IndexRange(first_index, data_.indices_num);
  }
  return std::nullopt;
}

inline void IndexMask::to_ranges_and_spans(Vector<IndexRange> &r_ranges,
                                           Vector<OffsetSpan<int64_t, int16_t>> &r_spans) const
{
  if (std::optional<IndexRange> range = this->to_range()) {
    r_ranges.append(*range);
  }
  else {
    this->to_ranges_and_spans_impl(r_ranges, r_spans);
  }
}

}  // namespace index_mask

using index_mask::IndexMask;
using index_mask::IndexMaskMemory;

}  // namespace blender
