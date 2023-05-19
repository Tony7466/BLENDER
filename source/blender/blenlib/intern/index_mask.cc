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
    static Array<int64_t> cumulative_chunk_sizes(chunks_num + 1);

    static const int16_t *static_offsets = get_static_indices_array().data();
    static const int16_t static_cumulative_segment_sizes[2] = {0, chunk_capacity};

    threading::isolate_task([&]() {
      threading::parallel_for(IndexRange(chunks_num), 1024, [&](const IndexRange range) {
        for (const int64_t i : range) {
          Chunk &chunk = chunks_array[i];
          chunk.segments_num = 1;
          chunk.indices_by_segment = &static_offsets;
          chunk.cumulative_segment_sizes = static_cumulative_segment_sizes;

          chunk_ids_array[i] = i;
          cumulative_chunk_sizes[i] = i * chunk_capacity;
        }
      });
    });
    cumulative_chunk_sizes.last() = max_size;

    IndexMask mask;
    IndexMaskData &mask_data = mask.data_for_inplace_construction();
    mask_data.chunks_num = chunks_num;
    mask_data.indices_num = max_size;
    mask_data.chunks = chunks_array.data();
    mask_data.chunk_ids = chunk_ids_array.data();
    mask_data.cumulative_chunk_sizes = cumulative_chunk_sizes.data();
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

static IndexMask strip_empty_chunks_and_create_mask(const Span<Chunk> possible_chunks,
                                                    const Span<int64_t> possible_chunk_ids,
                                                    IndexMaskMemory &memory)
{
  Vector<int64_t> non_empty_chunks;
  for (const int64_t i : possible_chunks.index_range()) {
    const Chunk &chunk = possible_chunks[i];
    if (chunk.size() > 0) {
      non_empty_chunks.append(i);
    }
  }
  const int64_t chunks_num = non_empty_chunks.size();
  if (chunks_num == 0) {
    return {};
  }

  MutableSpan<Chunk> final_chunks = memory.allocate_array<Chunk>(chunks_num);
  MutableSpan<int64_t> final_chunk_ids = memory.allocate_array<int64_t>(chunks_num);
  MutableSpan<int64_t> final_cumulative_chunk_sizes = memory.allocate_array<int64_t>(chunks_num +
                                                                                     1);
  int64_t counter = 0;
  for (const int64_t i : IndexRange(chunks_num)) {
    const int64_t i2 = non_empty_chunks[i];
    const Chunk &chunk = possible_chunks[i2];
    final_chunks[i] = chunk;
    final_chunk_ids[i] = possible_chunk_ids[i2];
    final_cumulative_chunk_sizes[i] = counter;
    counter += chunk.size();
  }
  final_cumulative_chunk_sizes.last() = counter;

  const Chunk &last_chunk = final_chunks.last();

  IndexMask mask;
  IndexMaskData &data = mask.data_for_inplace_construction();
  data.chunks_num = chunks_num;
  data.indices_num = counter;
  data.chunks = final_chunks.data();
  data.chunk_ids = final_chunk_ids.data();
  data.cumulative_chunk_sizes = final_cumulative_chunk_sizes.data();
  data.begin_it = RawChunkIterator{0, 0};
  data.end_it = last_chunk.end_iterator();
  return mask;
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
                                  Vector<std::variant<IndexRange, Span<T>>> &r_segments)
{
  BLI_assert(range_threshold >= 1);
  const int64_t old_segments_num = r_segments.size();
  Span<T> remaining_indices = indices;
  while (!remaining_indices.is_empty()) {
    if (non_empty_is_range(remaining_indices)) {
      /* All remaining indices are range. */
      r_segments.append(non_empty_as_range(remaining_indices));
      break;
    }
    if (non_empty_is_range(remaining_indices.take_front(range_threshold))) {
      /* Next segment is a range. Now find the place where the range ends. */
      const int64_t segment_size = find_size_of_next_range(remaining_indices);
      r_segments.append(IndexRange(remaining_indices[0], segment_size));
      remaining_indices = remaining_indices.drop_front(segment_size);
      continue;
    }
    /* Next segment is just indices. Now find the place where the next range starts. */
    const int64_t segment_size = find_size_until_next_range(remaining_indices, range_threshold);
    const Span<T> segment_indices = remaining_indices.take_front(segment_size);
    if (non_empty_is_range(segment_indices)) {
      r_segments.append(non_empty_as_range(segment_indices));
    }
    else {
      r_segments.append(segment_indices);
    }
    remaining_indices = remaining_indices.drop_front(segment_size);
  }
  return r_segments.size() - old_segments_num;
}

template<typename T> static void from_set(const Set<T> &set, MutableSpan<T> span)
{
  int64_t i = 0;
  for (const T value : set) {
    span[i] = value;
    i++;
  }
  std::sort(span.begin(), span.end());
}

}  // namespace unique_sorted_indices

void IndexMask::foreach_span_impl(const FunctionRef<void(OffsetSpan<int64_t, int16_t>)> fn) const
{
  this->foreach_span_template([fn](const int64_t chunk_id, const Span<int16_t> indices) {
    fn(OffsetSpan<int64_t, int16_t>(chunk_id << chunk_size_shift, indices));
  });
}

IndexMask IndexMask::slice_and_offset(const IndexRange range, IndexMaskMemory &memory) const
{
  return this->slice_and_offset(range.start(), range.size(), memory);
}

IndexMask IndexMask::slice_and_offset(const int64_t start,
                                      const int64_t size,
                                      IndexMaskMemory &memory) const
{
  if (size == 0) {
    return {};
  }
  if (this->to_range().has_value()) {
    return IndexMask(size);
  }
  const IndexMask sliced_mask = this->slice(start, size);
  const int64_t offset = sliced_mask.first();
  if (offset == 0) {
    return sliced_mask;
  }
  if (sliced_mask.to_range().has_value()) {
    return IndexMask(size);
  }
  const int64_t range_size = sliced_mask.last() - sliced_mask.first() + 1;
  BitVector bits(range_size);
  sliced_mask.to_bits(bits, offset);
  return IndexMask::from_bits(bits, memory);
}

IndexMask IndexMask::complement(const IndexRange universe, IndexMaskMemory &memory) const
{
  const AtomicExpr atomic_expr{*this};
  const ComplementExpr complement_expr{atomic_expr};
  return IndexMask::from_expr(complement_expr, universe, memory);
}

template<typename T>
IndexMask IndexMask::from_indices(const Span<T> indices, IndexMaskMemory &memory)
{
  using namespace unique_sorted_indices;

  if (indices.is_empty()) {
    /* Return early when there are no indices. */
    return {};
  }
  if (non_empty_is_range(indices)) {
    /* Use faster method of creating the mask when the input indices are a range. */
    return non_empty_as_range(indices);
  }

  /* Find indices that go into each chunk. */
  const Vector<IndexRange> split_ranges = split_by_chunk(indices);
  const int64_t chunks_num = split_ranges.size();

  MutableSpan<Chunk> chunks = memory.allocate_array<Chunk>(chunks_num);
  MutableSpan<int64_t> chunk_ids = memory.allocate_array<int64_t>(chunks_num);

  /* Code below may reuse memory from the global index array if possible. */
  static const int16_t *static_offsets = get_static_indices_array().data();
  /* If every index in a chunk is in the mask, it can be processed more efficiently. */
  const Chunk full_chunk_template = IndexMask(chunk_capacity).data().chunks[0];

  /* Process multiple chunks at once to reduce the number of memory allocations. */
  constexpr int64_t chunk_grain_size = 16;
  const auto build_chunk = [&](const int64_t chunk_i, LinearAllocator<> &allocator) {
    const IndexRange range_for_chunk = split_ranges[chunk_i];
    const Span<T> indices_in_chunk = indices.slice(range_for_chunk);
    BLI_assert(!indices_in_chunk.is_empty());

    const int64_t chunk_id = index_to_chunk_id(int64_t(indices_in_chunk[0]));
    BLI_assert(chunk_id == index_to_chunk_id(int64_t(indices_in_chunk.last())));
    Chunk &chunk = chunks[chunk_i];
    chunk_ids[chunk_i] = chunk_id;

    if (indices_in_chunk.size() == chunk_capacity) {
      /* Fast case when the chunk is full. */
      chunk = full_chunk_template;
      return;
    }

    const int64_t chunk_offset = chunk_ids[chunk_i] * chunk_capacity;

    /* Split indices in current chunk into segments. */
    Vector<std::variant<IndexRange, Span<T>>> segments;
    const int64_t segments_num = split_to_ranges_and_spans(indices_in_chunk, 64, segments);
    BLI_assert(segments_num > 0);

    MutableSpan<const int16_t *> indices_by_segment = allocator.allocate_array<const int16_t *>(
        segments_num);
    MutableSpan<int16_t> cumulative_segment_sizes = allocator.allocate_array<int16_t>(
        segments_num + 1);

    int64_t cumulative_size = 0;
    for (const int64_t segment_i : segments.index_range()) {
      const std::variant<IndexRange, Span<T>> &segment = segments[segment_i];
      cumulative_segment_sizes[segment_i] = int16_t(cumulative_size);

      if (std::holds_alternative<IndexRange>(segment)) {
        /* No extra allocations necessary because static memory is used. */
        const IndexRange range_in_segment = std::get<IndexRange>(segment);
        indices_by_segment[segment_i] = static_offsets + (range_in_segment.first() - chunk_offset);
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
        indices_by_segment[segment_i] = new_indices.data();
        cumulative_size += indices_in_segment.size();
      }
    }

    cumulative_segment_sizes.last() = int16_t(cumulative_size);

    chunk.segments_num = int16_t(segments_num);
    chunk.indices_by_segment = indices_by_segment.data();
    chunk.cumulative_segment_sizes = cumulative_segment_sizes.data();
  };

  if (chunks_num > chunk_grain_size) {
    /* There are many chunks, so use multi-threading and also use the global allocator for
     * allocations in each thread. */
    threading::EnumerableThreadSpecific<LinearAllocator<>> allocators;
    threading::parallel_for(
        IndexRange(chunks_num), chunk_grain_size, [&](const IndexRange chunks_range) {
          LinearAllocator<> &allocator = allocators.local();
          for (const int64_t chunk_i : chunks_range) {
            build_chunk(chunk_i, allocator);
          }
        });
    for (LinearAllocator<> &allocator : allocators) {
      memory.give_ownership_of_buffers_in(allocator);
    }
  }
  else {
    for (const int64_t chunk_i : IndexRange(chunks_num)) {
      build_chunk(chunk_i, memory);
    }
  }

  /* Fill cumulative chunk sizes of the entire index mask. */
  MutableSpan<int64_t> cumulative_chunk_sizes = memory.allocate_array<int64_t>(chunks_num + 1);
  int64_t cumulative_size = 0;
  for (const int64_t i : chunks.index_range()) {
    cumulative_chunk_sizes[i] = cumulative_size;
    cumulative_size += chunks[i].size();
  }
  cumulative_chunk_sizes.last() = cumulative_size;

  IndexMask mask;
  IndexMaskData &mask_data = mask.data_for_inplace_construction();
  mask_data.chunks_num = chunks_num;
  mask_data.indices_num = indices.size();
  mask_data.chunks = chunks.data();
  mask_data.chunk_ids = chunk_ids.data();
  mask_data.cumulative_chunk_sizes = cumulative_chunk_sizes.data();
  mask_data.begin_it = {0, 0};
  mask_data.end_it = chunks.last().end_iterator();
  return mask;
}

IndexMask IndexMask::from_bits(const BitSpan bits, IndexMaskMemory &memory, const int64_t offset)
{
  Vector<int64_t> indices;
  for (const int64_t i : bits.index_range()) {
    if (bits[i]) {
      indices.append(i + offset);
    }
  }
  return IndexMask::from_indices<int64_t>(indices, memory);
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
      universe, GrainSize(1024), memory, [&](const int64_t index) { return bools[index]; });
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
    return from_bools(universe, span, memory);
  }
  return IndexMask::from_predicate(
      universe, GrainSize(512), memory, [&](const int64_t index) { return bools[index]; });
}

static Set<int64_t> eval_expr(const Expr &base_expr, const IndexRange universe)
{
  Set<int64_t> result;
  switch (base_expr.type) {
    case Expr::Type::Atomic: {
      const AtomicExpr &expr = static_cast<const AtomicExpr &>(base_expr);
      expr.mask->foreach_index([&](const int64_t i) {
        BLI_assert(universe.contains(i));
        result.add_new(i);
      });
      break;
    }
    case Expr::Type::Union: {
      const UnionExpr &expr = static_cast<const UnionExpr &>(base_expr);
      for (const Expr *child : expr.children) {
        const Set<int64_t> child_result = eval_expr(*child, universe);
        for (const int64_t i : child_result) {
          result.add(i);
        }
      }
      break;
    }
    case Expr::Type::Difference: {
      const DifferenceExpr &expr = static_cast<const DifferenceExpr &>(base_expr);
      result = eval_expr(*expr.base, universe);
      for (const Expr *child : expr.children) {
        const Set<int64_t> child_result = eval_expr(*child, universe);
        for (const int64_t i : child_result) {
          result.remove(i);
        }
      }
      break;
    }
    case Expr::Type::Complement: {
      const ComplementExpr &expr = static_cast<const ComplementExpr &>(base_expr);
      const Set<int64_t> child_result = eval_expr(*expr.base, universe);
      for (const int64_t i : universe) {
        if (!child_result.contains(i)) {
          result.add_new(i);
        }
      }
      break;
    }
    case Expr::Type::Intersection: {
      const IntersectionExpr &expr = static_cast<const IntersectionExpr &>(base_expr);
      BLI_assert(!expr.children.is_empty());
      result = eval_expr(*expr.children.first(), universe);
      for (const Expr *child : expr.children.as_span().drop_front(1)) {
        const Set<int64_t> child_result = eval_expr(*child, universe);
        result.remove_if([&](const int64_t i) { return !child_result.contains(i); });
      }
      break;
    }
  }
  return result;
}

static Set<int64_t> find_chunk_ids_to_process(const Expr &base_expr, const IndexRange universe)
{
  Set<int64_t> result;
  switch (base_expr.type) {
    case Expr::Type::Atomic: {
      const AtomicExpr &expr = static_cast<const AtomicExpr &>(base_expr);
      for (const int64_t chunk_i : IndexRange(expr.mask->data().chunks_num)) {
        result.add_new(expr.mask->data().chunk_ids[chunk_i]);
      }
      break;
    }
    case Expr::Type::Union: {
      const UnionExpr &expr = static_cast<const UnionExpr &>(base_expr);
      for (const Expr *child : expr.children) {
        const Set<int64_t> child_result = find_chunk_ids_to_process(*child, universe);
        for (const int64_t chunk_id : child_result) {
          result.add(chunk_id);
        }
      }
      break;
    }
    case Expr::Type::Difference: {
      const DifferenceExpr &expr = static_cast<const DifferenceExpr &>(base_expr);
      result = find_chunk_ids_to_process(*expr.base, universe);
      break;
    }
    case Expr::Type::Complement: {
      const int64_t first_chunk_id = index_to_chunk_id(universe.first());
      const int64_t last_chunk_id = index_to_chunk_id(universe.last());
      for (const int64_t chunk_id : IndexRange(first_chunk_id, last_chunk_id - first_chunk_id + 1))
      {
        result.add(chunk_id);
      }
      break;
    }
    case Expr::Type::Intersection: {
      const IntersectionExpr &expr = static_cast<const IntersectionExpr &>(base_expr);
      BLI_assert(!expr.children.is_empty());
      result = find_chunk_ids_to_process(*expr.children.first(), universe);
      for (const Expr *child : expr.children.as_span().drop_front(1)) {
        const Set<int64_t> child_result = find_chunk_ids_to_process(*child, universe);
        result.remove_if([&](const int64_t chunk_id) { return !child_result.contains(chunk_id); });
      }
      break;
    }
  }
  return result;
}

static void find_chunk_ids_to_process(const Expr &base_expr,
                                      const IndexRange universe,
                                      MutableBitSpan r_chunk_is_full,
                                      MutableBitSpan r_chunk_non_empty)
{
  using TmpBitVector = BitVector<1024>;

  const int64_t max_chunk_id = index_to_chunk_id(universe.last());
  const int64_t r_size = max_chunk_id + 1;
  BLI_assert(r_chunk_is_full.size() == r_size);
  BLI_assert(r_chunk_non_empty.size() == r_size);

  switch (base_expr.type) {
    case Expr::Type::Atomic: {
      const AtomicExpr &expr = static_cast<const AtomicExpr &>(base_expr);
      const IndexMaskData &data = expr.mask->data();
      if (data.chunks_num == 0) {
        break;
      }
      if (data.chunks_num == 1) {
        const int64_t chunk_id = data.chunk_ids[0];
        r_chunk_non_empty[chunk_id].set();
        if (data.indices_num == chunk_capacity) {
          r_chunk_is_full[data.chunk_ids[0]].set();
        }
        break;
      }
      for (const int64_t chunk_i : IndexRange(data.chunks_num)) {
        const int64_t chunk_id = data.chunk_ids[chunk_i];
        const Chunk &chunk = data.chunks[chunk_i];
        r_chunk_non_empty[chunk_id].set();
        MutableBitRef is_full = r_chunk_is_full[chunk_id];
        if (chunk.is_full()) {
          if (chunk_i == 0 && data.begin_it.segment_i == 0 && data.begin_it.index_in_segment == 0)
          {
            is_full.set();
          }
          else if (chunk_i == data.chunks_num - 1 && data.end_it.segment_i == 0 &&
                   data.end_it.index_in_segment == chunk_capacity)
          {
            is_full.set();
          }
          else {
            is_full.set();
          }
        }
      }
      break;
    }
    case Expr::Type::Union: {
      const UnionExpr &expr = static_cast<const UnionExpr &>(base_expr);
      for (const Expr *child : expr.children) {
        TmpBitVector child_chunk_is_full(r_size, false);
        TmpBitVector child_chunk_non_empty(r_size, false);
        find_chunk_ids_to_process(*child, universe, child_chunk_is_full, child_chunk_non_empty);
        r_chunk_is_full |= child_chunk_is_full;
        r_chunk_non_empty |= child_chunk_non_empty;
      }
      break;
    }
    case Expr::Type::Difference: {
      const DifferenceExpr &expr = static_cast<const DifferenceExpr &>(base_expr);
      find_chunk_ids_to_process(*expr.base, universe, r_chunk_is_full, r_chunk_non_empty);
      for (const Expr *child : expr.children) {
        TmpBitVector child_chunk_is_full(r_size, false);
        TmpBitVector child_chunk_non_empty(r_size, false);
        find_chunk_ids_to_process(*child, universe, child_chunk_is_full, child_chunk_non_empty);
        r_chunk_is_full.clear_by_set_bits(child_chunk_non_empty);
        r_chunk_non_empty.clear_by_set_bits(child_chunk_is_full);
      }
      break;
    }
    case Expr::Type::Complement: {
      const ComplementExpr &expr = static_cast<const ComplementExpr &>(base_expr);
      /* The parameters are reversed intentionally. */
      find_chunk_ids_to_process(*expr.base, universe, r_chunk_non_empty, r_chunk_is_full);
      r_chunk_is_full.flip();
      r_chunk_non_empty.flip();
      break;
    }
    case Expr::Type::Intersection: {
      const IntersectionExpr &expr = static_cast<const IntersectionExpr &>(base_expr);
      BLI_assert(!expr.children.is_empty());
      find_chunk_ids_to_process(*expr.children[0], universe, r_chunk_is_full, r_chunk_non_empty);
      for (const Expr *child : expr.children.as_span().drop_front(1)) {
        TmpBitVector child_chunk_is_full(r_size, false);
        TmpBitVector child_chunk_non_empty(r_size, false);
        find_chunk_ids_to_process(*child, universe, child_chunk_is_full, child_chunk_non_empty);
        r_chunk_is_full &= child_chunk_is_full;
        r_chunk_non_empty &= child_chunk_non_empty;
      }
      break;
    }
  }
}

static Vector<int64_t> get_chunk_ids_to_evaluate_expression_in(const Expr & /*expr*/,
                                                               const IndexRange universe)
{
  const int64_t first_chunk_id = index_to_chunk_id(universe.first());
  const int64_t last_chunk_id = index_to_chunk_id(universe.last());
  Vector<int64_t> chunk_ids(last_chunk_id - first_chunk_id + 1);
  for (const int64_t i : chunk_ids.index_range()) {
    chunk_ids[i] = first_chunk_id + i;
  }
  return chunk_ids;
}

static std::optional<ChunkSlice> try_get_chunk_by_id(const IndexMask &mask, const int64_t chunk_id)
{
  const IndexMaskData &data = mask.data();
  const int64_t *chunk_id_iterator = std::lower_bound(
      data.chunk_ids, data.chunk_ids + data.chunks_num, chunk_id);
  const int64_t index = chunk_id_iterator - data.chunk_ids;
  if (index == data.chunks_num) {
    return std::nullopt;
  }
  if (data.chunk_ids[index] != chunk_id) {
    return std::nullopt;
  }
  return mask.chunk(index);
}

static Set<int16_t> eval_expr_for_chunk_id__index_set(const Expr &base_expr,
                                                      const IndexRange chunk_universe,
                                                      const int64_t chunk_id)
{
  Set<int16_t> result;
  switch (base_expr.type) {
    case Expr::Type::Atomic: {
      const AtomicExpr &expr = static_cast<const AtomicExpr &>(base_expr);
      const IndexMask &mask = *expr.mask;
      const std::optional<ChunkSlice> chunk_slice = try_get_chunk_by_id(mask, chunk_id);
      if (!chunk_slice.has_value()) {
        break;
      }
      chunk_slice->foreach_span([&](const Span<int16_t> indices) {
        for (const int16_t index : indices) {
          result.add_new(index);
        }
      });
      break;
    }
    case Expr::Type::Union: {
      const UnionExpr &expr = static_cast<const UnionExpr &>(base_expr);
      for (const Expr *child : expr.children) {
        const Set<int16_t> child_result = eval_expr_for_chunk_id__index_set(
            *child, chunk_universe, chunk_id);
        for (const int16_t index : child_result) {
          result.add(index);
        }
      }
      break;
    }
    case Expr::Type::Difference: {
      const DifferenceExpr &expr = static_cast<const DifferenceExpr &>(base_expr);
      result = eval_expr_for_chunk_id__index_set(*expr.base, chunk_universe, chunk_id);
      for (const Expr *child : expr.children) {
        const Set<int16_t> child_result = eval_expr_for_chunk_id__index_set(
            *child, chunk_universe, chunk_id);
        result.remove_if([&](const int16_t index) { return child_result.contains(index); });
      }
      break;
    }
    case Expr::Type::Complement: {
      const ComplementExpr &expr = static_cast<const ComplementExpr &>(base_expr);
      const Set<int16_t> child_result = eval_expr_for_chunk_id__index_set(
          *expr.base, chunk_universe, chunk_id);
      for (const int64_t index : chunk_universe) {
        if (!child_result.contains(int16_t(index))) {
          result.add_new(int16_t(index));
        }
      }
      break;
    }
    case Expr::Type::Intersection: {
      const IntersectionExpr &expr = static_cast<const IntersectionExpr &>(base_expr);
      result = eval_expr_for_chunk_id__index_set(*expr.children[0], chunk_universe, chunk_id);
      for (const Expr *child : expr.children.as_span().drop_front(1)) {
        const Set<int16_t> child_result = eval_expr_for_chunk_id__index_set(
            *child, chunk_universe, chunk_id);
        result.remove_if([&](const int16_t index) { return !child_result.contains(index); });
      }
      break;
    }
  }
  return result;
}

static void eval_expressions_for_chunk_ids(const Expr &expr,
                                           const IndexRange universe,
                                           const Span<int64_t> chunk_ids,
                                           MutableSpan<Chunk> r_chunks,
                                           IndexMaskMemory &memory,
                                           std::mutex &memory_mutex)
{
  BLI_assert(chunk_ids.size() == r_chunks.size());
  for (const int64_t chunk_i : chunk_ids.index_range()) {
    const int64_t chunk_id = chunk_ids[chunk_i];
    Chunk &chunk = r_chunks[chunk_i];

    const int64_t chunk_offset = chunk_id * chunk_capacity;
    const int64_t chunk_universe_start = std::max<int64_t>(0, universe.start() - chunk_offset);
    const int64_t chunk_universe_end = std::min<int64_t>(chunk_capacity,
                                                         universe.one_after_last() - chunk_offset);
    const IndexRange chunk_universe{chunk_universe_start,
                                    chunk_universe_end - chunk_universe_start};
    const Set<int16_t> indices_in_chunk = eval_expr_for_chunk_id__index_set(
        expr, chunk_universe, chunk_id);

    MutableSpan<const int16_t *> indices_by_segment;
    MutableSpan<int16_t> indices_in_segment;
    MutableSpan<int16_t> cumulative_segment_sizes;
    {
      std::lock_guard lock{memory_mutex};
      indices_by_segment = memory.allocate_array<const int16_t *>(1);
      indices_in_segment = memory.allocate_array<int16_t>(indices_in_chunk.size());
      cumulative_segment_sizes = memory.allocate_array<int16_t>(2);
    }

    indices_by_segment[0] = indices_in_segment.data();
    cumulative_segment_sizes[0] = 0;
    cumulative_segment_sizes[1] = int16_t(indices_in_chunk.size());
    unique_sorted_indices::from_set(indices_in_chunk, indices_in_segment);

    chunk.segments_num = 1;
    chunk.indices_by_segment = indices_by_segment.data();
    chunk.cumulative_segment_sizes = cumulative_segment_sizes.data();
  }
}

IndexMask IndexMask::from_expr(const Expr &expr,
                               const IndexRange universe,
                               IndexMaskMemory &memory)
{
  if (universe.is_empty()) {
    return {};
  }

  const Vector<int64_t> possible_chunk_ids = get_chunk_ids_to_evaluate_expression_in(expr,
                                                                                     universe);
  Array<Chunk> possible_chunks(possible_chunk_ids.size());
  std::mutex memory_mutex;
  threading::parallel_for(possible_chunk_ids.index_range(), 32, [&](const IndexRange range) {
    eval_expressions_for_chunk_ids(expr,
                                   universe,
                                   possible_chunk_ids.as_span().slice(range),
                                   possible_chunks.as_mutable_span().slice(range),
                                   memory,
                                   memory_mutex);
  });

  return strip_empty_chunks_and_create_mask(possible_chunks, possible_chunk_ids, memory);
}

template<typename T> void IndexMask::to_indices(MutableSpan<T> r_indices) const
{
  BLI_assert(this->size() == r_indices.size());
  this->foreach_index_optimized(
      GrainSize(1024),
      [&](const int64_t i, const int64_t mask_i) mutable { r_indices[mask_i] = T(i); });
}

void IndexMask::to_bits(MutableBitSpan r_bits, int64_t offset) const
{
  BLI_assert(r_bits.size() >= this->min_array_size() - offset);
  r_bits.reset_all();
  this->foreach_span_or_range([&](const auto mask_segment) {
    if constexpr (std::is_same_v<std::decay_t<decltype(mask_segment)>, IndexRange>) {
      const IndexRange range = mask_segment.shift(-offset);
      r_bits.slice(range).set_all();
    }
    else {
      for (const int64_t i : mask_segment) {
        r_bits[i - offset].set();
      }
    }
  });
}

void IndexMask::to_bools(MutableSpan<bool> r_bools, int64_t offset) const
{
  BLI_assert(r_bools.size() >= this->min_array_size() - offset);
  r_bools.fill(false);
  this->foreach_index_optimized([&](const int64_t i) { r_bools[i - offset] = true; });
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
  this->foreach_span_template(
      [&, range_checker](const int64_t chunk_id, const Span<int16_t> indices) {
        const int64_t chunk_offset = chunk_id << chunk_size_shift;
        if (range_checker.check(indices)) {
          r_ranges.append_as(indices[0] + chunk_offset, indices.size());
        }
        else {
          r_spans.append_as(chunk_offset, indices);
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

  const int64_t universe_chunks_num = universe.data_.chunks_num;
  Vector<Chunk> chunks;
  Vector<int64_t> chunk_ids;
  for (const int64_t chunk_i : IndexRange(universe_chunks_num)) {
    const int64_t chunk_id = universe.data_.chunk_ids[chunk_i];
    const int64_t chunk_offset = chunk_id << chunk_size_shift;
    const ChunkSlice universe_chunk = universe.chunk(chunk_i);

    std::array<int16_t, chunk_capacity> indices_array;
    int16_t *indices_ptr = indices_array.data();
    universe_chunk.foreach_span([&](const Span<int16_t> segment) {
      indices_ptr += filter_indices(OffsetSpan<int64_t, int16_t>(chunk_offset, segment),
                                    indices_ptr);
    });
    const int16_t indices_num = int16_t(indices_ptr - indices_array.data());
    if (indices_num == 0) {
      continue;
    }

    MutableSpan<int16_t> indices = memory.construct_array_copy<int16_t>(
        {indices_array.data(), indices_num});
    MutableSpan<int16_t> cumulative_segment_sizes = memory.construct_array_copy<int16_t>(
        {0, indices_num});
    MutableSpan<const int16_t *> indices_by_segment = memory.construct_array_copy<const int16_t *>(
        {indices.data()});

    Chunk chunk;
    chunk.segments_num = 1;
    chunk.cumulative_segment_sizes = cumulative_segment_sizes.data();
    chunk.indices_by_segment = indices_by_segment.data();
    chunks.append(chunk);
    chunk_ids.append(chunk_id);
  }

  const int64_t chunks_num = chunks.size();
  if (chunks_num == 0) {
    return {};
  }

  MutableSpan<int64_t> cumulative_chunk_sizes = memory.allocate_array<int64_t>(chunks_num + 1);
  cumulative_chunk_sizes[0] = 0;
  for (const int64_t chunk_i : IndexRange(chunks_num)) {
    cumulative_chunk_sizes[chunk_i + 1] = cumulative_chunk_sizes[chunk_i] + chunks[chunk_i].size();
  }

  IndexMask mask;
  IndexMaskData &mask_data = mask.data_for_inplace_construction();
  mask_data.chunks_num = chunks_num;
  mask_data.indices_num = cumulative_chunk_sizes.last();
  mask_data.chunks = memory.construct_array_copy<Chunk>(chunks).data();
  mask_data.chunk_ids = memory.construct_array_copy<int64_t>(chunk_ids).data();
  mask_data.cumulative_chunk_sizes = cumulative_chunk_sizes.data();
  mask_data.begin_it = RawChunkIterator{0, 0};
  mask_data.end_it = chunks.last().end_iterator();

  return mask;
}

template IndexMask IndexMask::from_indices(Span<int32_t>, IndexMaskMemory &);
template IndexMask IndexMask::from_indices(Span<int64_t>, IndexMaskMemory &);
template void IndexMask::to_indices(MutableSpan<int32_t>) const;
template void IndexMask::to_indices(MutableSpan<int64_t>) const;

namespace unique_sorted_indices {
template Vector<IndexRange> split_by_chunk(const Span<int32_t> indices);
template Vector<IndexRange> split_by_chunk(const Span<int64_t> indices);
}  // namespace unique_sorted_indices

void do_benchmark(const int64_t total);
void do_benchmark(const int64_t /*total*/) {}

}  // namespace blender::index_mask
