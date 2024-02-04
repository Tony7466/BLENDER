/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <fmt/format.h>

#include "BLI_array.hh"
#include "BLI_index_mask_expression.hh"
#include "BLI_multi_value_map.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"
#include "BLI_strict_flags.h"
#include "BLI_task.hh"
#include "BLI_vector_set.hh"

namespace blender::index_mask {

static IndexMask evaluate_with_bool_array(const Expr &expression, IndexMaskMemory &memory)
{
  switch (expression.type) {
    case Expr::Type::Atomic: {
      const auto &expr = static_cast<const AtomicExpr &>(expression);
      return *expr.mask;
    }
    case Expr::Type::Union: {
      const auto &expr = static_cast<const UnionExpr &>(expression);
      Vector<bool> values;
      for (const Expr *term : expr.terms) {
        const IndexMask term_mask = evaluate_with_bool_array(*term, memory);
        if (values.size() < term_mask.min_array_size()) {
          values.resize(term_mask.min_array_size(), false);
        }
        term_mask.foreach_index([&](const int64_t i) { values[i] = true; });
      }
      return IndexMask::from_bools(values, memory);
    }
    case Expr::Type::Intersection: {
      const auto &expr = static_cast<const IntersectionExpr &>(expression);
      const Expr &first_term = *expr.terms[0];
      const IndexMask first_term_mask = evaluate_with_bool_array(first_term, memory);
      Array<bool> values(first_term_mask.min_array_size(), false);
      first_term_mask.foreach_index([&](const int64_t i) { values[i] = true; });
      for (const Expr *term : expr.terms.as_span().drop_front(1)) {
        const IndexMask term_mask = evaluate_with_bool_array(*term, memory)
                                        .complement(IndexRange(first_term_mask.min_array_size()),
                                                    memory);
        term_mask.foreach_index([&](const int64_t i) { values[i] = false; });
      }
      return IndexMask::from_bools(values, memory);
    }
    case Expr::Type::Difference: {
      const auto &expr = static_cast<const DifferenceExpr &>(expression);
      const IndexMask main_term_mask = evaluate_with_bool_array(*expr.main_term, memory);
      Array<bool> values(main_term_mask.min_array_size(), false);
      main_term_mask.foreach_index([&](const int64_t i) { values[i] = true; });
      for (const Expr *subtract_term : expr.subtract_terms) {
        const IndexMask subtract_term_mask = evaluate_with_bool_array(*subtract_term, memory);
        subtract_term_mask.foreach_index([&](const int64_t i) {
          if (i < main_term_mask.min_array_size()) {
            values[i] = false;
          }
        });
      }
      return IndexMask::from_bools(values, memory);
    }
  }
  BLI_assert_unreachable();
  return {};
}

struct ChunkResult {
  Vector<IndexMaskSegment> segments;
  int16_t result_size = 0;
};

static ChunkResult evaluate_chunk(
    const Expr &root_expression,
    const IndexRange chunk_range,
    const MultiValueMap<const IndexMask *, IndexMaskSegment> &segments_by_mask,
    LinearAllocator<> &allocator)
{
  BLI_assert(chunk_range.size() <= max_segment_size);
  const std::array<int16_t, max_segment_size> &static_indices = get_static_indices_array();
  const int expr_array_size = root_expression.expression_array_size();

  Array<std::optional<ChunkResult>> chunk_results(expr_array_size);
  Stack<const Expr *> expressions_to_compute;
  expressions_to_compute.push(&root_expression);

  while (!expressions_to_compute.is_empty()) {
    const Expr &expression = *expressions_to_compute.peek();
    if (chunk_results[expression.index].has_value()) {
      expressions_to_compute.pop();
      continue;
    }
    switch (expression.type) {
      case Expr::Type::Atomic: {
        const auto &expr = expression.as_atomic();
        const Span<IndexMaskSegment> segments = segments_by_mask.lookup(expr.mask);
        ChunkResult &result = chunk_results[expr.index].emplace();
        for (const IndexMaskSegment &segment : segments) {
          result.segments.append(
              IndexMaskSegment(segment.offset() - chunk_range.start(), segment.base_span()));
          result.result_size += int16_t(segment.size());
        }
        break;
      }
      case Expr::Type::Union: {
        const auto &expr = expression.as_union();
        bool all_terms_computed = true;
        for (const Expr *term : expr.terms) {
          if (const std::optional<ChunkResult> &term_result = chunk_results[term->index]) {
            if (term_result->result_size == chunk_range.size()) {
              chunk_results[expression.index].emplace(*term_result);
              break;
            }
          }
          else {
            expressions_to_compute.push(term);
            all_terms_computed = false;
            break;
          }
        }
        if (all_terms_computed) {
          ChunkResult &result = chunk_results[expression.index].emplace();
          /* TODO: Optimize merge. */
          Set<int16_t> indices_set;
          for (const Expr *term : expr.terms) {
            const ChunkResult &term_result = *chunk_results[term->index];
            for (const IndexMaskSegment &segment : term_result.segments) {
              for (const int64_t i : segment) {
                BLI_assert(i < max_segment_size);
                indices_set.add(int16_t(i));
              }
            }
          }
          MutableSpan<int16_t> indices = allocator.allocate_array<int16_t>(indices_set.size());
          std::copy_n(indices_set.begin(), indices_set.size(), indices.begin());
          result.result_size = int16_t(indices.size());
          result.segments.append(IndexMaskSegment(0, indices));
        }
        break;
      }
      case Expr::Type::Intersection: {
        /* TODO */
        BLI_assert_unreachable();
        break;
      }
      case Expr::Type::Difference: {
        /* TODO */
        BLI_assert_unreachable();
        break;
      }
    }
  }
  return *chunk_results[root_expression.index];
}

static IndexMask evaluated_generic(const Expr &root_expression, IndexMaskMemory &memory)
{
  const int expr_array_size = root_expression.expression_array_size();

  VectorSet<const IndexMask *> masks;
  Array<bool> pushed_expressions(expr_array_size, false);
  Stack<const Expr *> expressions_to_check;
  expressions_to_check.push(&root_expression);
  pushed_expressions[root_expression.index] = true;
  while (!expressions_to_check.is_empty()) {
    const Expr &expression = *expressions_to_check.pop();
    switch (expression.type) {
      case Expr::Type::Atomic: {
        masks.add(expression.as_atomic().mask);
        break;
      }
      case Expr::Type::Union: {
        for (const Expr *term : expression.as_union().terms) {
          if (assign_if_different(pushed_expressions[term->index], true)) {
            expressions_to_check.push(term);
          }
        }
        break;
      }
      case Expr::Type::Intersection: {
        for (const Expr *term : expression.as_intersection().terms) {
          if (assign_if_different(pushed_expressions[term->index], true)) {
            expressions_to_check.push(term);
          }
        }
        break;
      }
      case Expr::Type::Difference: {
        {
          const Expr *term = expression.as_difference().main_term;
          if (assign_if_different(pushed_expressions[term->index], true)) {
            expressions_to_check.push(term);
          }
        }
        for (const Expr *term : expression.as_difference().subtract_terms) {
          if (assign_if_different(pushed_expressions[term->index], true)) {
            expressions_to_check.push(term);
          }
        }
        break;
      }
    }
  }

  VectorSet<int64_t> possible_chunk_starts;
  /* TODO: Only use masks which have a "positive" impact on the final result. */
  for (const IndexMask *mask : masks) {
    mask->foreach_segment([&](const IndexMaskSegment segment) {
      if (segment.is_empty()) {
        return;
      }
      const int64_t begin_chunk_offset = segment[0] & max_segment_size_mask_high;
      const int64_t end_chunk_offset = segment.last() & max_segment_size_mask_high;
      possible_chunk_starts.add(begin_chunk_offset);
      if (begin_chunk_offset != end_chunk_offset) {
        possible_chunk_starts.add(end_chunk_offset);
      }
    });
  }

  Map<int64_t, MultiValueMap<const IndexMask *, IndexMaskSegment>> segments_by_chunk;
  for (const IndexMask *mask : masks) {
    mask->foreach_segment([&](const IndexMaskSegment segment) {
      if (segment.is_empty()) {
        return;
      }
      const int64_t begin_chunk_offset = segment[0] & max_segment_size_mask_high;
      const int64_t end_chunk_offset = segment.last() & max_segment_size_mask_high;
      if (begin_chunk_offset == end_chunk_offset) {
        segments_by_chunk.lookup_or_add_default(begin_chunk_offset).add(mask, segment);
      }
      else {
        const int64_t split_index_in_segment = binary_search::find_predicate_begin(
            segment.base_span(),
            [&](const int16_t i) { return i >= end_chunk_offset - segment[0]; });
        const IndexMaskSegment segment_1 = IndexMaskSegment(
            segment.offset(), segment.base_span().take_front(split_index_in_segment));
        const IndexMaskSegment segment_2 = IndexMaskSegment(
            segment.offset(), segment.base_span().drop_front(split_index_in_segment));
        segments_by_chunk.lookup_or_add_default(begin_chunk_offset).add(mask, segment_1);
        segments_by_chunk.lookup_or_add_default(end_chunk_offset).add(mask, segment_2);
      }
    });
  }

  Vector<ChunkResult> all_chunk_results(possible_chunk_starts.size());

  threading::parallel_for(
      possible_chunk_starts.index_range(), 1, [&](const IndexRange chunk_start_range) {
        LinearAllocator<> local_allocator;
        for (const int64_t chunk_start_i : chunk_start_range) {
          const int64_t segment_start = possible_chunk_starts[chunk_start_i];
          all_chunk_results[chunk_start_i] = evaluate_chunk(
              root_expression,
              IndexRange(segment_start, max_segment_size),
              segments_by_chunk.lookup(segment_start),
              local_allocator);
        }
      });

  Vector<IndexMaskSegment> result_segments;
  for (const int64_t chunk_i : all_chunk_results.index_range()) {
    const int64_t chunk_offset = possible_chunk_starts[chunk_i];
    const ChunkResult &chunk_result = all_chunk_results[chunk_i];
    for (const IndexMaskSegment &segment : chunk_result.segments) {
      result_segments.append(IndexMaskSegment(segment.offset() + chunk_offset,
                                              memory.construct_array_copy(segment.base_span())));
    }
  }

  /* TODO: Consolidate segments. */

  return IndexMask::from_segments(result_segments, memory);
}

IndexMask evaluate_expression(const Expr &expression, IndexMaskMemory &memory)
{
  return evaluated_generic(expression, memory);
}

}  // namespace blender::index_mask
