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
#include "BLI_vector.hh"

namespace blender {

struct GrainSize {
  int64_t value;

  explicit GrainSize(const int64_t grain_size) : value(grain_size) {}
};

template<typename T> class VArray;

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
  bool is_full() const;
  bool is_full_after_inclusive(const RawChunkIterator &it) const;
  bool is_full_until_exclusive(const RawChunkIterator &it) const;
  std::optional<RawChunkIterator> find(int16_t index) const;

  template<typename Fn> void foreach_span(Fn &&fn) const;
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

struct ChunkSlice {
  const Chunk *chunk;
  RawChunkIterator begin_it;
  RawChunkIterator end_it;

  template<typename Fn> void foreach_span(Fn &&fn) const;
};

struct Expr {
  enum class Type {
    Atomic,
    Union,
    Difference,
    Complement,
    Intersection,
  };
  Type type;

  Expr(const Type type) : type(type) {}
};

struct AtomicExpr : public Expr {
  const IndexMask *mask;

  AtomicExpr(const IndexMask &mask) : Expr(Type::Atomic), mask(&mask) {}
};

struct UnionExpr : public Expr {
  Vector<const Expr *> children;

  UnionExpr(Vector<const Expr *> children) : Expr(Type::Union), children(std::move(children)) {}
};

struct DifferenceExpr : public Expr {
  const Expr *base = nullptr;
  Vector<const Expr *> children;

  DifferenceExpr(const Expr &base, Vector<const Expr *> children)
      : Expr(Type::Difference), base(&base), children(std::move(children))
  {
  }
};

struct ComplementExpr : public Expr {
  const Expr *base = nullptr;

  ComplementExpr(const Expr &base) : Expr(Type::Complement), base(&base) {}
};

struct IntersectionExpr : public Expr {
  Vector<const Expr *> children;

  IntersectionExpr(Vector<const Expr *> children)
      : Expr(Type::Intersection), children(std::move(children))
  {
  }
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
  OffsetIndices<int64_t> chunk_offsets() const;
  int64_t first() const;
  int64_t last() const;
  int64_t min_array_size() const;
  std::optional<RawMaskIterator> find(const int64_t index) const;
  bool contains(const int64_t index) const;

  RawMaskIterator index_to_iterator(const int64_t index) const;
  int64_t iterator_to_index(const RawMaskIterator &it) const;

  IndexMask slice(IndexRange range) const;
  IndexMask slice(int64_t start, int64_t size) const;
  IndexMask slice_and_offset(IndexRange range, IndexMaskMemory &memory) const;
  IndexMask slice_and_offset(int64_t start, int64_t size, IndexMaskMemory &memory) const;
  IndexMask complement(const IndexRange universe, IndexMaskMemory &memory) const;

  ChunkSlice chunk(const int64_t chunk_i) const;

  int64_t operator[](const int64_t i) const;
  int64_t operator[](const RawMaskIterator &it) const;

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
  static IndexMask from_bits(BitSpan bits, IndexMaskMemory &memory, int64_t offset = 0);
  static IndexMask from_bools(Span<bool> bools, IndexMaskMemory &memory);
  static IndexMask from_bools(const VArray<bool> &bools, IndexMaskMemory &memory);
  static IndexMask from_bools(const IndexMask &universe,
                              Span<bool> bools,
                              IndexMaskMemory &memory);
  static IndexMask from_bools(const IndexMask &universe,
                              const VArray<bool> &bools,
                              IndexMaskMemory &memory);
  static IndexMask from_expr(const Expr &expr, IndexRange universe, IndexMaskMemory &memory);
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
  void to_bits(MutableBitSpan r_bits, int64_t offset = 0) const;
  void to_bools(MutableSpan<bool> r_bools, int64_t offset = 0) const;
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

  template<typename Fn> void foreach_span_template(Fn &&fn) const;
};

class IndexMaskFromSegment : NonCopyable, NonMovable {
 public:
  const int16_t *segment_indices;
  std::array<int16_t, 2> cumulative_segment_sizes;
  Chunk chunk;
  int64_t chunk_id;
  std::array<int64_t, 2> cumulative_chunk_sizes;
  IndexMask mask;

  IndexMaskFromSegment()
  {
    IndexMaskData &data = mask.data_for_inplace_construction();
    this->cumulative_segment_sizes[0] = 0;
    this->cumulative_chunk_sizes[0] = 0;
    this->chunk.segments_num = 1;
    this->chunk.indices_by_segment = &this->segment_indices;
    this->chunk.cumulative_segment_sizes = this->cumulative_segment_sizes.data();
    data.chunks = &chunk;
    data.chunks_num = 1;
    data.chunk_ids = &this->chunk_id;
    data.cumulative_chunk_sizes = this->cumulative_chunk_sizes.data();
  }

  void update(const int64_t chunk_id, const Span<int16_t> indices)
  {
    BLI_assert(!indices.is_empty());
    BLI_assert(std::is_sorted(indices.begin(), indices.end()));
    BLI_assert(indices[0] >= 0);
    BLI_assert(indices.last() < chunk_capacity);
    const int64_t indices_num = indices.size();

    IndexMaskData &data = mask.data_for_inplace_construction();
    this->segment_indices = indices.data();
    this->chunk_id = chunk_id;
    this->cumulative_segment_sizes[1] = int16_t(indices_num);
    this->cumulative_chunk_sizes[1] = indices_num;
    data.end_it.index_in_segment = indices_num;
    data.indices_num = indices_num;
  }
};

std::ostream &operator<<(std::ostream &stream, const IndexMask &mask);

namespace unique_sorted_indices {

template<typename T> Vector<IndexRange> split_by_chunk(Span<T> indices);

template<typename T>
int64_t split_to_ranges_and_spans(Span<T> indices,
                                  int64_t range_threshold,
                                  Vector<std::variant<IndexRange, Span<T>>> &r_segments);
template<typename T> bool non_empty_is_range(const Span<T> indices);
template<typename T> IndexRange non_empty_as_range(const Span<T> indices);
template<typename T> int64_t find_size_of_next_range(const Span<T> indices);
template<typename T>
int64_t find_size_until_next_range(const Span<T> indices, const int64_t min_range_size);

}  // namespace unique_sorted_indices

template<typename Fn>
inline IndexMask grow_indices_to_ranges(const IndexMask &mask,
                                        const Fn &fn,
                                        IndexMaskMemory &memory)
{
  Vector<int64_t> indices;
  mask.foreach_index([&](const int64_t i) {
    const IndexRange new_range = fn(i);
    for (const int64_t new_index : new_range) {
      indices.append(new_index);
    }
  });
  return IndexMask::from_indices<int64_t>(indices, memory);
}

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
/** \name Unique Sorted Indices Inline Methods
 * \{ */

namespace unique_sorted_indices {

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
  BLI_assert(it.index_in_segment < this->segment_offsets()[it.segment_i].size());
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

inline void init_empty_mask(IndexMaskData &data)
{
  static constexpr int64_t cumulative_sizes_for_empty_mask[1] = {0};

  data.chunks_num = 0;
  data.indices_num = 0;
  data.chunks = nullptr;
  data.chunk_ids = nullptr;
  data.cumulative_chunk_sizes = cumulative_sizes_for_empty_mask;
  data.begin_it.segment_i = 0;
  data.begin_it.index_in_segment = 0;
  data.end_it.segment_i = 0;
  data.end_it.index_in_segment = 0;
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
  data_.chunks_num = size_to_chunk_num(size);
  data_.indices_num = size;
  data_.end_it.index_in_segment = size - ((size - 1) & chunk_mask_high);
}

inline IndexMask::IndexMask(const IndexRange range)
{
  if (range.is_empty()) {
    init_empty_mask(data_);
    return;
  }
  const int64_t one_after_last = range.one_after_last();
  *this = get_static_index_mask_for_min_size(one_after_last);

  const int64_t first_chunk_id = index_to_chunk_id(range.first());
  const int64_t last_chunk_id = index_to_chunk_id(range.last());

  data_.chunks_num = last_chunk_id - first_chunk_id + 1;
  data_.indices_num = range.size();
  data_.chunks += first_chunk_id;
  data_.chunk_ids += first_chunk_id;
  data_.cumulative_chunk_sizes += first_chunk_id;
  data_.begin_it.segment_i = 0;
  data_.begin_it.index_in_segment = range.first() & chunk_mask_low;
  data_.end_it.segment_i = 0;
  data_.end_it.index_in_segment = one_after_last - ((one_after_last - 1) & chunk_mask_high);
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
  const int64_t full_index = index + data_.chunks[0].iterator_to_index(data_.begin_it) +
                             data_.cumulative_chunk_sizes[0];
  it.chunk_i = this->chunk_offsets().find_range_index(full_index);
  const Chunk &chunk = data_.chunks[it.chunk_i];
  it.chunk_it = chunk.index_to_iterator(full_index - data_.cumulative_chunk_sizes[it.chunk_i]);
  return it;
}

inline int64_t IndexMask::iterator_to_index(const RawMaskIterator &it) const
{
  BLI_assert(it.chunk_i >= 0);
  BLI_assert(it.chunk_i < data_.chunks_num);
  const Chunk &chunk = data_.chunks[it.chunk_i];
  BLI_assert(chunk.cumulative_segment_sizes[0] == 0);
  return it.chunk_it.index_in_segment + chunk.cumulative_segment_sizes[it.chunk_it.segment_i] +
         data_.cumulative_chunk_sizes[it.chunk_i] - data_.cumulative_chunk_sizes[0] -
         data_.chunks[0].iterator_to_index(data_.begin_it);
}

inline ChunkSlice IndexMask::chunk(const int64_t chunk_i) const
{
  BLI_assert(chunk_i >= 0);
  BLI_assert(chunk_i < data_.chunks_num);
  ChunkSlice chunk_slice;
  chunk_slice.chunk = data_.chunks + chunk_i;
  chunk_slice.begin_it = (chunk_i == 0) ? data_.begin_it : RawChunkIterator{0, 0};
  chunk_slice.end_it = (chunk_i == data_.chunks_num - 1) ? data_.end_it :
                                                           chunk_slice.chunk->end_iterator();
  return chunk_slice;
}

inline int64_t IndexMask::operator[](const int64_t i) const
{
  const RawMaskIterator it = this->index_to_iterator(i);
  return (*this)[it];
}

inline int64_t IndexMask::operator[](const RawMaskIterator &it) const
{
  return data_.chunks[it.chunk_i]
             .indices_by_segment[it.chunk_it.segment_i][it.chunk_it.index_in_segment] +
         (data_.chunk_ids[it.chunk_i] << chunk_size_shift);
}

inline IndexMask IndexMask::slice(const IndexRange range) const
{
  return this->slice(range.start(), range.size());
}

inline IndexMask IndexMask::slice(const int64_t start, const int64_t size) const
{
  if (size == 0) {
    return {};
  }
  const RawMaskIterator first_it = this->index_to_iterator(start);
  const RawMaskIterator last_it = this->index_to_iterator(start + size - 1);

  IndexMask sliced;
  sliced.data_.chunks_num = last_it.chunk_i - first_it.chunk_i + 1;
  sliced.data_.indices_num = size;
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
    data.index_in_segment = this->segment_offsets()[this->segments_num - 1].size();
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
  return this->cumulative_segment_sizes[this->segments_num] - this->cumulative_segment_sizes[0];
}

inline bool Chunk::is_full() const
{
  return this->size() == chunk_capacity;
}

inline bool Chunk::is_full_after_inclusive(const RawChunkIterator &it) const
{
  const Span<int16_t> indices{this->indices_by_segment[it.segment_i] + it.index_in_segment,
                              this->segment_size(it.segment_i) - it.index_in_segment};
  return unique_sorted_indices::non_empty_is_range(indices);
}

inline bool Chunk::is_full_until_exclusive(const RawChunkIterator &it) const
{
  if (it.segment_i > 0) {
    return false;
  }
  return this->indices_by_segment[0][it.index_in_segment] == it.index_in_segment;
}

inline int16_t Chunk::segment_size(const int16_t segment_i) const
{
  return this->cumulative_segment_sizes[segment_i + 1] - this->cumulative_segment_sizes[segment_i];
}

template<typename Fn> inline void Chunk::foreach_span(Fn &&fn) const
{
  for (const int64_t segment_i : IndexRange(this->segments_num)) {
    const Span<int16_t> indices{this->indices_by_segment[segment_i],
                                this->segment_size(segment_i)};
    fn(indices);
  }
}

template<typename Fn> inline void ChunkSlice::foreach_span(Fn &&fn) const
{
  if (this->begin_it.segment_i == this->end_it.segment_i) {
    const int64_t segment_i = this->begin_it.segment_i;
    const int64_t begin_i = this->begin_it.index_in_segment;
    const int64_t end_i = this->end_it.index_in_segment;
    const Span<int16_t> indices{this->chunk->indices_by_segment[segment_i] + begin_i,
                                end_i - begin_i};
    fn(indices);
  }
  else {
    {
      const int64_t first_segment_i = this->begin_it.segment_i;
      const int64_t begin_i = this->begin_it.index_in_segment;
      const int64_t end_i = this->chunk->segment_size(first_segment_i);
      const Span<int16_t> indices{this->chunk->indices_by_segment[first_segment_i] + begin_i,
                                  end_i - begin_i};
      fn(indices);
    }
    for (int64_t segment_i = this->begin_it.segment_i + 1; segment_i < this->end_it.segment_i;
         segment_i++)
    {
      const int64_t begin_i = 0;
      const int64_t end_i = this->chunk->segment_size(segment_i);
      const Span<int16_t> indices{this->chunk->indices_by_segment[segment_i] + begin_i,
                                  end_i - begin_i};
      fn(indices);
    }
    {
      const int64_t last_segment_i = this->end_it.segment_i;
      const int64_t begin_i = 0;
      const int64_t end_i = this->end_it.index_in_segment;
      const Span<int16_t> indices{this->chunk->indices_by_segment[last_segment_i] + begin_i,
                                  end_i - begin_i};
      fn(indices);
    }
  }
}

template<typename Fn>
constexpr bool has_mask_segment_and_start_parameter =
    std::is_invocable_r_v<void, Fn, OffsetSpan<int64_t, int16_t>, int64_t> ||
    std::is_invocable_r_v<void, Fn, IndexRange, int64_t>;

template<typename Fn> inline void IndexMask::foreach_span_template(Fn &&fn) const
{
  for (const int64_t chunk_i : IndexRange(data_.chunks_num)) {
    const int64_t chunk_id = data_.chunk_ids[chunk_i];
    const ChunkSlice chunk_slice = this->chunk(chunk_i);
    chunk_slice.foreach_span([&](const Span<int16_t> indices) { fn(chunk_id, indices); });
  }
}

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

template<typename Fn> inline void IndexMask::foreach_index_optimized(Fn &&fn) const
{
  this->foreach_span_or_range([&](const auto mask_segment, const int64_t start) {
    if constexpr (std::is_invocable_r_v<void, Fn, int64_t, int64_t>) {
      for (const int64_t i : mask_segment.index_range()) {
        fn(mask_segment[i], start + i);
      }
    }
    else if constexpr (std::is_same_v<std::decay_t<decltype(mask_segment)>, IndexRange>) {
      foreach_index_in_range(mask_segment, fn);
    }
    else {
      for (const int64_t index : mask_segment) {
        fn(index);
      }
    }
  });
}

template<typename Fn> inline void IndexMask::foreach_span_or_range(Fn &&fn) const
{
  IndexRangeChecker is_index_mask;
  this->foreach_span(
      [&, is_index_mask](const OffsetSpan<int64_t, int16_t> mask_segment, const int64_t start) {
        if (is_index_mask.check(mask_segment.base_span())) {
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
    if constexpr (has_mask_segment_and_start_parameter<Fn>) {
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
