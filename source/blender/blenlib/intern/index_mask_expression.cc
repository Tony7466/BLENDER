/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <fmt/format.h>

#include "BLI_array.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask_expression.hh"
#include "BLI_multi_value_map.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"
#include "BLI_strict_flags.h"
#include "BLI_task.hh"
#include "BLI_timeit.hh"
#include "BLI_vector_set.hh"

namespace blender::index_mask {

constexpr int64_t inline_expr_array_size = 16;

struct FastResultSegment {
  enum class Type {
    Unknown,
    Full,
    Copy,
  };
  Type type = Type::Unknown;
  IndexRange bounds;
  const IndexMask *mask = nullptr;
};

struct FastResult {
  Vector<FastResultSegment> segments;
};

struct Boundary {
  int64_t index;
  bool is_begin;
  const FastResultSegment *segment;
};

static void sort_boundaries(MutableSpan<Boundary> boundaries)
{
  std::sort(boundaries.begin(), boundaries.end(), [](const Boundary &a, const Boundary &b) {
    return a.index < b.index;
  });
}

BLI_NOINLINE static FastResult evaluate_fast_union(const Span<Boundary> boundaries)
{
  FastResult result;
  Vector<const FastResultSegment *, 16> active_segments;
  for (const Boundary &boundary : boundaries) {
    if (active_segments.is_empty()) {
      BLI_assert(boundary.is_begin);
      result.segments.append(
          {boundary.segment->type, IndexRange(boundary.index, 0), boundary.segment->mask});
      active_segments.append(boundary.segment);
    }
    else {
      FastResultSegment &prev_segment = result.segments.last();
      /* Previous segment goes at least until current boundary. */
      prev_segment.bounds = IndexRange::from_begin_end(prev_segment.bounds.start(),
                                                       boundary.index);
      if (boundary.is_begin) {
        active_segments.append(boundary.segment);
      }
      else {
        active_segments.remove_first_occurrence_and_reorder(boundary.segment);
      }
      int full_count = 0;
      int unknown_count = 0;
      int copy_count = 0;
      const IndexMask *copy_from_mask = nullptr;
      for (const FastResultSegment *active_segment : active_segments) {
        switch (active_segment->type) {
          case FastResultSegment::Type::Unknown: {
            unknown_count++;
            break;
          }
          case FastResultSegment::Type::Full: {
            full_count++;
            break;
          }
          case FastResultSegment::Type::Copy: {
            copy_count++;
            copy_from_mask = active_segment->mask;
            break;
          }
        }
      }
      if (full_count > 0) {
        if (prev_segment.type == FastResultSegment::Type::Full) {
          /* Do nothing. */
        }
        else {
          result.segments.append({FastResultSegment::Type::Full, IndexRange(boundary.index, 0)});
        }
      }
      else if (unknown_count > 0 || copy_count > 1) {
        if (prev_segment.type == FastResultSegment::Type::Unknown) {
          /* Do nothing. */
        }
        else {
          result.segments.append(
              {FastResultSegment::Type::Unknown, IndexRange(boundary.index, 0)});
        }
      }
      else if (copy_count == 1) {
        BLI_assert(copy_from_mask);
        result.segments.append(
            {FastResultSegment::Type::Copy, IndexRange(boundary.index, 0), copy_from_mask});
      }
    }
  }

  return result;
}

BLI_NOINLINE static FastResult evaluate_fast_intersection(const Span<const FastResult *> terms)
{
  if (terms.is_empty()) {
    return {};
  }
  /* TODO */
  int64_t bounds_min = std::numeric_limits<int64_t>::max();
  int64_t bounds_max = std::numeric_limits<int64_t>::min();
  for (const FastResult *term : terms) {
    if (!term->segments.is_empty()) {
      bounds_min = std::min(bounds_min, term->segments[0].bounds.start());
      bounds_max = std::max(bounds_max, term->segments.last().bounds.one_after_last());
    }
  }
  if (bounds_min > bounds_max) {
    return {};
  }
  FastResult result;
  result.segments.append(
      {FastResultSegment::Type::Unknown, IndexRange::from_begin_end(bounds_min, bounds_max)});
  return result;
}

BLI_NOINLINE static FastResult evaluate_fast_difference(
    const FastResult &main_term, const Span<const FastResult *> /*subtract_terms*/)
{
  /* TODO */
  return evaluate_fast_intersection({&main_term});
}

BLI_NOINLINE static FastResult evaluate_fast(
    const Expr &root_expression,
    const Span<const Expr *> eager_eval_order,
    const std::optional<IndexRange> eval_bounds = std::nullopt)
{
  Array<std::optional<FastResult>, inline_expr_array_size> expression_results(
      root_expression.expression_array_size());

  for (const Expr *expression : eager_eval_order) {
    FastResult &expr_result = expression_results[expression->index].emplace();
    switch (expression->type) {
      case Expr::Type::Atomic: {
        const AtomicExpr &expr = expression->as_atomic();

        IndexMask mask;
        if (eval_bounds.has_value()) {
          mask = expr.mask->slice_content(*eval_bounds);
        }
        else {
          mask = *expr.mask;
        }

        if (!mask.is_empty()) {
          const IndexRange bounds = mask.bounds();
          if (const std::optional<IndexRange> range = mask.to_range()) {
            expr_result.segments.append({FastResultSegment::Type::Full, bounds});
          }
          else {
            expr_result.segments.append({FastResultSegment::Type::Copy, bounds, expr.mask});
          }
        }
        break;
      }
      case Expr::Type::Union: {
        const UnionExpr &expr = expression->as_union();
        Vector<Boundary, 16> boundaries;
        for (const Expr *term : expr.terms) {
          const FastResult &term_result = *expression_results[term->index];
          for (const FastResultSegment &segment : term_result.segments) {
            boundaries.append({segment.bounds.first(), true, &segment});
            boundaries.append({segment.bounds.one_after_last(), false, &segment});
          }
        }
        sort_boundaries(boundaries);
        expr_result = evaluate_fast_union(boundaries);
        break;
      }
      case Expr::Type::Intersection: {
        const IntersectionExpr &expr = expression->as_intersection();
        Vector<const FastResult *> term_results;
        for (const Expr *term : expr.terms) {
          const FastResult &term_result = *expression_results[term->index];
          term_results.append(&term_result);
        }
        expr_result = evaluate_fast_intersection(term_results);
        break;
      }
      case Expr::Type::Difference: {
        const DifferenceExpr &expr = expression->as_difference();
        const FastResult &main_result = *expression_results[expr.terms[0]->index];
        Vector<const FastResult *> subtract_term_results;
        for (const Expr *term : expr.terms.as_span().drop_front(1)) {
          const FastResult &subtract_term_result = *expression_results[term->index];
          subtract_term_results.append(&subtract_term_result);
        }
        expr_result = evaluate_fast_difference(main_result, subtract_term_results);
        break;
      }
    }
  }

  const FastResult &final_result = *expression_results[root_expression.index];
  return final_result;
}

struct FinalResultSegment {
  enum class Type {
    Full,
    Copy,
    Indices,
  };

  Type type = Type::Indices;
  IndexRange bounds;
  const IndexMask *copy_mask = nullptr;
  IndexMaskSegment indices;
};

BLI_NOINLINE static IndexMaskSegment evaluate_segment(const Expr &root_expression,
                                                      IndexMaskMemory &memory,
                                                      const IndexRange bounds)
{
  BLI_assert(bounds.size() <= max_segment_size);
  const int64_t segment_offset = bounds.start();
  const int expr_array_size = root_expression.expression_array_size();
  Array<std::optional<IndexMaskSegment>, inline_expr_array_size> results(expr_array_size);
  Stack<const Expr *> expressions_to_compute;
  expressions_to_compute.push(&root_expression);

  while (!expressions_to_compute.is_empty()) {
    const Expr &expression = *expressions_to_compute.peek();
    if (results[expression.index].has_value()) {
      expressions_to_compute.pop();
      continue;
    }
    switch (expression.type) {
      case Expr::Type::Atomic: {
        const auto &expr = expression.as_atomic();
        const IndexMask sliced_mask = expr.mask->slice_content(bounds);
        const int64_t segments_num = sliced_mask.segments_num();
        IndexMaskSegment segment;
        if (segments_num == 1) {
          segment = sliced_mask.segment(0);
        }
        else if (segments_num > 1) {
          MutableSpan<int16_t> indices = memory.allocate_array<int16_t>(sliced_mask.size());
          sliced_mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
            const int64_t index = i - segment_offset;
            BLI_assert(index < max_segment_size);
            indices[pos] = int16_t(index);
          });
          segment = IndexMaskSegment(segment_offset, indices);
        }
        results[expr.index] = segment;
        break;
      }
      case Expr::Type::Union: {
        const auto &expr = expression.as_union();
        bool all_terms_computed = true;
        for (const Expr *term : expr.terms) {
          if (const std::optional<IndexMaskSegment> &term_result = results[term->index]) {
            if (term_result->size() == bounds.size()) {
              results[expr.index] = *term_result;
              break;
            }
          }
          else {
            expressions_to_compute.push(term);
            all_terms_computed = false;
            break;
          }
        }
        if (all_terms_computed && !results[expr.index]) {
          /* TODO: Generalize. */
          BLI_assert(expr.terms.size() == 2);
          const IndexMaskSegment segment_0 = *results[expr.terms[0]->index];
          const IndexMaskSegment segment_1 = *results[expr.terms[1]->index];
          Vector<int64_t, max_segment_size> indices_vec(max_segment_size);
          const int64_t indices_num = std::set_union(segment_0.begin(),
                                                     segment_0.end(),
                                                     segment_1.begin(),
                                                     segment_1.end(),
                                                     indices_vec.begin()) -
                                      indices_vec.begin();
          MutableSpan<int16_t> indices = memory.allocate_array<int16_t>(indices_num);
          for (const int64_t i : IndexRange(indices_num)) {
            indices[i] = int16_t(indices_vec[i] - segment_offset);
          }
          results[expr.index] = IndexMaskSegment(segment_offset, indices);
        }
        break;
      }
      case Expr::Type::Intersection: {
        const auto &expr = expression.as_intersection();
        bool all_terms_computed = true;
        for (const Expr *term : expr.terms) {
          if (const std::optional<IndexMaskSegment> &term_result = results[term->index]) {
            if (term_result->is_empty()) {
              results[expr.index] = IndexMaskSegment();
              break;
            }
          }
          else {
            expressions_to_compute.push(term);
            all_terms_computed = false;
          }
        }
        if (all_terms_computed && !results[expr.index]) {
          /* TODO: Generalize. */
          BLI_assert(expr.terms.size() == 2);
          const IndexMaskSegment segment_0 = *results[expr.terms[0]->index];
          const IndexMaskSegment segment_1 = *results[expr.terms[1]->index];
          Vector<int64_t, max_segment_size> indices_vec(max_segment_size);
          const int64_t indices_num = std::set_intersection(segment_0.begin(),
                                                            segment_0.end(),
                                                            segment_1.begin(),
                                                            segment_1.end(),
                                                            indices_vec.begin()) -
                                      indices_vec.begin();
          MutableSpan<int16_t> indices = memory.allocate_array<int16_t>(indices_num);
          for (const int64_t i : IndexRange(indices_num)) {
            indices[i] = int16_t(indices_vec[i] - segment_offset);
          }
          results[expr.index] = IndexMaskSegment(segment_offset, indices);
        }
        break;
      }
      case Expr::Type::Difference: {
        const auto &expr = expression.as_difference();
        bool all_terms_computed = true;
        if (const std::optional<IndexMaskSegment> main_term_result = results[expr.terms[0]->index])
        {
          if (main_term_result->is_empty()) {
            results[expr.index] = IndexMaskSegment();
            break;
          }
          for (const Expr *subtract_term : expr.terms.as_span().drop_front(1)) {
            if (const std::optional<IndexMaskSegment> subtract_term_result =
                    results[subtract_term->index])
            {
              if (subtract_term_result->size() == bounds.size()) {
                results[expr.index] = IndexMaskSegment();
                break;
              }
            }
            else {
              expressions_to_compute.push(subtract_term);
              all_terms_computed = false;
              break;
            }
          }
        }
        else {
          expressions_to_compute.push(expr.terms[0]);
          all_terms_computed = false;
          break;
        }
        if (all_terms_computed && !results[expr.index]) {
          /* TODO: Generalize. */
          BLI_assert(expr.terms.size() == 2);
          const IndexMaskSegment segment_main = *results[expr.terms[0]->index];
          const IndexMaskSegment segment_subtract = *results[expr.terms[1]->index];
          Vector<int64_t, max_segment_size> indices_vec(max_segment_size);
          const int64_t indices_num = std::set_difference(segment_main.begin(),
                                                          segment_main.end(),
                                                          segment_subtract.begin(),
                                                          segment_subtract.end(),
                                                          indices_vec.begin()) -
                                      indices_vec.begin();
          MutableSpan<int16_t> indices = memory.allocate_array<int16_t>(indices_num);
          for (const int64_t i : IndexRange(indices_num)) {
            indices[i] = int16_t(indices_vec[i] - segment_offset);
          }
          results[expr.index] = IndexMaskSegment(segment_offset, indices);
        }
        break;
      }
    }
  }

  return *results[root_expression.index];
}

BLI_NOINLINE static Vector<IndexMaskSegment> build_result_segments(
    const Span<FinalResultSegment> final_segments)
{
  const std::array<int16_t, max_segment_size> &static_indices_array = get_static_indices_array();

  Vector<IndexMaskSegment> result_segments;
  for (const FinalResultSegment &final_segment : final_segments) {
    switch (final_segment.type) {
      case FinalResultSegment::Type::Full: {
        const int64_t full_size = final_segment.bounds.size();
        for (int64_t i = 0; i < full_size; i += max_segment_size) {
          const int64_t size = std::min(i + max_segment_size, full_size) - i;
          result_segments.append(IndexMaskSegment(final_segment.bounds.first() + i,
                                                  Span(static_indices_array).take_front(size)));
        }
        break;
      }
      case FinalResultSegment::Type::Copy: {
        const IndexMask sliced_mask = final_segment.copy_mask->slice_content(final_segment.bounds);
        sliced_mask.foreach_segment(
            [&](const IndexMaskSegment &segment) { result_segments.append(segment); });
        break;
      }
      case FinalResultSegment::Type::Indices: {
        result_segments.append(final_segment.indices);
        break;
      }
    }
  }
  return result_segments;
}

BLI_NOINLINE static Vector<const Expr *, inline_expr_array_size> compute_eager_eval_order(
    const Expr &root_expression)
{
  Vector<const Expr *, inline_expr_array_size> eval_order;
  if (root_expression.type == Expr::Type::Atomic) {
    eval_order.append(&root_expression);
    return eval_order;
  }

  Array<bool, inline_expr_array_size> is_evaluated_states(root_expression.expression_array_size(),
                                                          false);
  Stack<const Expr *, inline_expr_array_size> expr_stack;
  expr_stack.push(&root_expression);

  while (!expr_stack.is_empty()) {
    const Expr &expression = *expr_stack.peek();
    bool &is_evaluated = is_evaluated_states[expression.index];
    if (is_evaluated) {
      expr_stack.pop();
      continue;
    }
    bool all_terms_evaluated = true;
    for (const Expr *term : expression.terms) {
      bool &term_evaluated = is_evaluated_states[term->index];
      if (!term_evaluated) {
        if (term->type == Expr::Type::Atomic) {
          eval_order.append(term);
          term_evaluated = true;
        }
        else {
          expr_stack.push(term);
          all_terms_evaluated = false;
        }
      }
    }
    if (all_terms_evaluated) {
      eval_order.append(&expression);
      is_evaluated = true;
      expr_stack.pop();
    }
  }

  return eval_order;
}

BLI_NOINLINE static IndexMask evaluate_expression_impl(const Expr &root_expression,
                                                       IndexMaskMemory &memory)
{
  Vector<FinalResultSegment> final_segments;
  Stack<IndexRange> long_unknown_segments;
  Vector<IndexRange> short_unknown_segments;

  const Vector<const Expr *, inline_expr_array_size> eager_eval_order = compute_eager_eval_order(
      root_expression);

  auto handle_fast_result = [&](const FastResult &fast_result) {
    for (const FastResultSegment &segment : fast_result.segments) {
      switch (segment.type) {
        case FastResultSegment::Type::Unknown: {
          if (segment.bounds.size() > max_segment_size) {
            long_unknown_segments.push(segment.bounds);
          }
          else {
            short_unknown_segments.append(segment.bounds);
          }
          break;
        }
        case FastResultSegment::Type::Copy: {
          BLI_assert(segment.mask);
          final_segments.append({FinalResultSegment::Type::Copy, segment.bounds, segment.mask});
          break;
        }
        case FastResultSegment::Type::Full: {
          final_segments.append({FinalResultSegment::Type::Full, segment.bounds});
          break;
        }
      }
    }
  };
  const FastResult initial_fast_result = evaluate_fast(root_expression, eager_eval_order);
  handle_fast_result(initial_fast_result);

  while (!long_unknown_segments.is_empty()) {
    const IndexRange unknown_bounds = long_unknown_segments.pop();
    const int64_t split_pos = unknown_bounds.size() / 2;
    const IndexRange left_half = unknown_bounds.take_front(split_pos);
    const IndexRange right_half = unknown_bounds.drop_front(split_pos);
    const FastResult left_result = evaluate_fast(root_expression, eager_eval_order, left_half);
    const FastResult right_result = evaluate_fast(root_expression, eager_eval_order, right_half);
    handle_fast_result(left_result);
    handle_fast_result(right_result);
  }

  for (const IndexRange &unknown_bounds : short_unknown_segments) {
    const IndexMaskSegment indices = evaluate_segment(root_expression, memory, unknown_bounds);
    if (!indices.is_empty()) {
      final_segments.append({FinalResultSegment::Type::Indices, unknown_bounds, nullptr, indices});
    }
  }

  if (final_segments.is_empty()) {
    return {};
  }
  if (final_segments.size() == 1) {
    const FinalResultSegment &final_segment = final_segments[0];
    switch (final_segment.type) {
      case FinalResultSegment::Type::Full: {
        return IndexMask(IndexRange(final_segment.bounds));
      }
      case FinalResultSegment::Type::Copy: {
        return final_segment.copy_mask->slice_content(final_segment.bounds);
      }
      case FinalResultSegment::Type::Indices: {
        return IndexMask::from_segments({final_segment.indices}, memory);
      }
    }
  }

  std::sort(final_segments.begin(),
            final_segments.end(),
            [](const FinalResultSegment &a, const FinalResultSegment &b) {
              return a.bounds.start() < b.bounds.start();
            });

  Vector<IndexMaskSegment> result_segments = build_result_segments(final_segments);
  return IndexMask::from_segments(result_segments, memory);
}

IndexMask evaluate_expression(const Expr &expression, IndexMaskMemory &memory)
{
  return evaluate_expression_impl(expression, memory);
}

}  // namespace blender::index_mask
