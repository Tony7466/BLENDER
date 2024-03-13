/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Expression evaluation has multiple phases:
 * 1. A coarse evaluation that tries to find segments which can be trivially evaluated. For
 *    example, taking the union of two overlapping ranges can be done in O(1) time.
 * 2. For all segments which can't be fully evaluated using coarse evaluation, an exact evaluation
 *    is done. This uses either an index-based or bit-based approach depending on a heuristic.
 * 3. Construct the final index mask based on the resulting intermediate segments.
 */

#include <fmt/format.h>
#include <iostream>
#include <mutex>

#include "BLI_array.hh"
#include "BLI_bit_group_vector.hh"
#include "BLI_bit_span_ops.hh"
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

struct CoarseSegment {
  enum class Type {
    Unknown,
    Full,
    Copy,
  };
  Type type = Type::Unknown;
  IndexRange bounds;
  const IndexMask *mask = nullptr;
};

struct CoarseResult {
  Vector<CoarseSegment> segments;
};

struct CourseBoundary {
  int64_t index;
  bool is_begin;
  const CoarseSegment *segment;
};

struct EvaluatedSegment {
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

enum class ExactEvalMode {
  Indices,
  Bits,
};

static void sort_boundaries(MutableSpan<CourseBoundary> boundaries)
{
  std::sort(boundaries.begin(),
            boundaries.end(),
            [](const CourseBoundary &a, const CourseBoundary &b) { return a.index < b.index; });
}

static CoarseSegment &evaluate_coarse_make_full_segment(CoarseSegment *prev_segment,
                                                        const int64_t prev_boundary_index,
                                                        const int64_t current_boundary_index,
                                                        CoarseResult &result)
{
  if (prev_segment && prev_segment->type == CoarseSegment::Type::Full &&
      prev_segment->bounds.one_after_last() == prev_boundary_index)
  {
    /* Extend previous segment. */
    prev_segment->bounds = IndexRange::from_begin_end(prev_segment->bounds.first(),
                                                      current_boundary_index);
    return *prev_segment;
  }
  result.segments.append(
      {CoarseSegment::Type::Full,
       IndexRange::from_begin_end(prev_boundary_index, current_boundary_index)});
  return result.segments.last();
}

static CoarseSegment &evaluate_coarse_make_unknown_segment(CoarseSegment *prev_segment,
                                                           const int64_t prev_boundary_index,
                                                           const int64_t current_boundary_index,
                                                           CoarseResult &result)
{
  if (prev_segment && prev_segment->type == CoarseSegment::Type::Unknown &&
      prev_segment->bounds.one_after_last() == prev_boundary_index)
  {
    /* Extend previous segment. */
    prev_segment->bounds = IndexRange::from_begin_end(prev_segment->bounds.first(),
                                                      current_boundary_index);
    return *prev_segment;
  }
  result.segments.append(
      {CoarseSegment::Type::Unknown,
       IndexRange::from_begin_end(prev_boundary_index, current_boundary_index)});
  return result.segments.last();
}

static CoarseSegment &evaluate_coarse_make_copy_segment(CoarseSegment *prev_segment,
                                                        const int64_t prev_boundary_index,
                                                        const int64_t current_boundary_index,
                                                        const IndexMask &copy_from_mask,
                                                        CoarseResult &result)
{
  if (prev_segment && prev_segment->type == CoarseSegment::Type::Copy &&
      prev_segment->bounds.one_after_last() == prev_boundary_index &&
      prev_segment->mask == &copy_from_mask)
  {
    /* Extend previous segment. */
    prev_segment->bounds = IndexRange::from_begin_end(prev_segment->bounds.first(),
                                                      current_boundary_index);
    return *prev_segment;
  }
  result.segments.append({CoarseSegment::Type::Copy,
                          IndexRange::from_begin_end(prev_boundary_index, current_boundary_index),
                          &copy_from_mask});
  return result.segments.last();
}

BLI_NOINLINE static void evaluate_coarse_union(const Span<CourseBoundary> boundaries,
                                               CoarseResult &r_result)
{
  if (boundaries.is_empty()) {
    return;
  }

  CoarseResult &result = r_result;
  CoarseSegment *prev_segment = nullptr;
  Vector<const CoarseSegment *, 16> active_segments;
  int64_t prev_boundary_index = boundaries[0].index;

  for (const CourseBoundary &boundary : boundaries) {
    if (prev_boundary_index < boundary.index) {
      bool has_full = false;
      bool has_unknown = false;
      bool copy_from_mask_unique = true;
      const IndexMask *copy_from_mask = nullptr;
      for (const CoarseSegment *active_segment : active_segments) {
        switch (active_segment->type) {
          case CoarseSegment::Type::Unknown: {
            has_unknown = true;
            break;
          }
          case CoarseSegment::Type::Full: {
            has_full = true;
            break;
          }
          case CoarseSegment::Type::Copy: {
            if (copy_from_mask != nullptr && copy_from_mask != active_segment->mask) {
              copy_from_mask_unique = false;
            }
            copy_from_mask = active_segment->mask;
            break;
          }
        }
      }
      if (has_full) {
        prev_segment = &evaluate_coarse_make_full_segment(
            prev_segment, prev_boundary_index, boundary.index, result);
      }
      else if (has_unknown || !copy_from_mask_unique) {
        prev_segment = &evaluate_coarse_make_unknown_segment(
            prev_segment, prev_boundary_index, boundary.index, result);
      }
      else if (copy_from_mask != nullptr && copy_from_mask_unique) {
        prev_segment = &evaluate_coarse_make_copy_segment(
            prev_segment, prev_boundary_index, boundary.index, *copy_from_mask, result);
      }

      prev_boundary_index = boundary.index;
    }

    if (boundary.is_begin) {
      active_segments.append(boundary.segment);
    }
    else {
      active_segments.remove_first_occurrence_and_reorder(boundary.segment);
    }
  }
}

BLI_NOINLINE static void evaluate_coarse_intersection(const Span<CourseBoundary> boundaries,
                                                      const int64_t terms_num,
                                                      CoarseResult &r_result)
{
  if (boundaries.is_empty()) {
    return;
  }

  CoarseResult &result = r_result;
  CoarseSegment *prev_segment = nullptr;
  Vector<const CoarseSegment *, 16> active_segments;
  int64_t prev_boundary_index = boundaries[0].index;

  for (const CourseBoundary &boundary : boundaries) {
    if (prev_boundary_index < boundary.index) {
      /* Only if one segment of each term is active, it's possible that the output contains
       * anything. */
      if (active_segments.size() == terms_num) {
        int full_count = 0;
        int unknown_count = 0;
        int copy_count = 0;
        bool copy_from_mask_unique = true;
        const IndexMask *copy_from_mask = nullptr;
        for (const CoarseSegment *active_segment : active_segments) {
          switch (active_segment->type) {
            case CoarseSegment::Type::Unknown: {
              unknown_count++;
              break;
            }
            case CoarseSegment::Type::Full: {
              full_count++;
              break;
            }
            case CoarseSegment::Type::Copy: {
              copy_count++;
              if (copy_from_mask != nullptr && copy_from_mask != active_segment->mask) {
                copy_from_mask_unique = false;
              }
              copy_from_mask = active_segment->mask;
              break;
            }
          }
        }
        BLI_assert(full_count + unknown_count + copy_count == terms_num);
        if (full_count == terms_num) {
          prev_segment = &evaluate_coarse_make_full_segment(
              prev_segment, prev_boundary_index, boundary.index, result);
        }
        else if (unknown_count > 0 || copy_count < terms_num || !copy_from_mask_unique) {
          prev_segment = &evaluate_coarse_make_unknown_segment(
              prev_segment, prev_boundary_index, boundary.index, result);
        }
        else if (copy_count == terms_num && copy_from_mask_unique) {
          prev_segment = &evaluate_coarse_make_copy_segment(
              prev_segment, prev_boundary_index, boundary.index, *copy_from_mask, result);
        }
      }

      prev_boundary_index = boundary.index;
    }

    if (boundary.is_begin) {
      active_segments.append(boundary.segment);
    }
    else {
      active_segments.remove_first_occurrence_and_reorder(boundary.segment);
    }
  }
}

/* TODO: Use struct instead of pair. */
BLI_NOINLINE static void evaluate_coarse_difference(
    const Span<std::pair<CourseBoundary, bool>> boundaries, CoarseResult &r_result)
{
  if (boundaries.is_empty()) {
    return;
  }

  CoarseResult &result = r_result;
  CoarseSegment *prev_segment = nullptr;
  Vector<const CoarseSegment *> active_main_segments;
  Vector<const CoarseSegment *, 16> active_subtract_segments;
  int64_t prev_boundary_index = boundaries[0].first.index;

  for (const std::pair<CourseBoundary, bool> &boundary : boundaries) {
    if (prev_boundary_index < boundary.first.index) {
      BLI_assert(active_main_segments.size() <= 1);
      if (active_main_segments.size() == 1) {
        const CoarseSegment &active_main_segment = *active_main_segments[0];
        bool has_subtract_full = false;
        bool subtract_copy_from_mask_unique = true;
        const IndexMask *subtract_copy_from_mask = nullptr;
        for (const CoarseSegment *active_subtract_segment : active_subtract_segments) {
          switch (active_subtract_segment->type) {
            case CoarseSegment::Type::Unknown: {
              break;
            }
            case CoarseSegment::Type::Full: {
              has_subtract_full = true;
              break;
            }
            case CoarseSegment::Type::Copy: {
              if (subtract_copy_from_mask != nullptr &&
                  subtract_copy_from_mask != active_subtract_segment->mask)
              {
                subtract_copy_from_mask_unique = false;
              }
              subtract_copy_from_mask = active_subtract_segment->mask;
              break;
            }
          }
        }

        if (has_subtract_full) {
          /* Do nothing. */
        }
        else {
          switch (active_main_segment.type) {
            case CoarseSegment::Type::Unknown: {
              prev_segment = &evaluate_coarse_make_unknown_segment(
                  prev_segment, prev_boundary_index, boundary.first.index, result);
              break;
            }
            case CoarseSegment::Type::Full: {
              if (active_subtract_segments.is_empty()) {
                prev_segment = &evaluate_coarse_make_full_segment(
                    prev_segment, prev_boundary_index, boundary.first.index, result);
              }
              else {
                prev_segment = &evaluate_coarse_make_unknown_segment(
                    prev_segment, prev_boundary_index, boundary.first.index, result);
              }
              break;
            }
            case CoarseSegment::Type::Copy: {
              if (active_subtract_segments.is_empty()) {
                prev_segment = &evaluate_coarse_make_copy_segment(prev_segment,
                                                                  prev_boundary_index,
                                                                  boundary.first.index,
                                                                  *active_main_segment.mask,
                                                                  result);
              }
              else if (subtract_copy_from_mask == active_main_segment.mask &&
                       subtract_copy_from_mask_unique)
              {
                /* Do nothing. */
              }
              else {
                prev_segment = &evaluate_coarse_make_unknown_segment(
                    prev_segment, prev_boundary_index, boundary.first.index, result);
              }
              break;
            }
          }
        }
      }

      prev_boundary_index = boundary.first.index;
    }

    if (boundary.second) {
      if (boundary.first.is_begin) {
        active_main_segments.append(boundary.first.segment);
      }
      else {
        active_main_segments.remove_first_occurrence_and_reorder(boundary.first.segment);
      }
    }
    else {
      if (boundary.first.is_begin) {
        active_subtract_segments.append(boundary.first.segment);
      }
      else {
        active_subtract_segments.remove_first_occurrence_and_reorder(boundary.first.segment);
      }
    }
  }
}

BLI_NOINLINE static CoarseResult evaluate_coarse(
    const Expr &root_expression,
    const Span<const Expr *> eager_eval_order,
    const std::optional<IndexRange> eval_bounds = std::nullopt)
{
  Array<std::optional<CoarseResult>, inline_expr_array_size> expression_results(
      root_expression.expression_array_size());

  for (const Expr *expression : eager_eval_order) {
    CoarseResult &expr_result = expression_results[expression->index].emplace();
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
            expr_result.segments.append({CoarseSegment::Type::Full, bounds});
          }
          else {
            expr_result.segments.append({CoarseSegment::Type::Copy, bounds, expr.mask});
          }
        }
        break;
      }
      case Expr::Type::Union: {
        const UnionExpr &expr = expression->as_union();
        Vector<CourseBoundary, 16> boundaries;
        for (const Expr *term : expr.terms) {
          const CoarseResult &term_result = *expression_results[term->index];
          for (const CoarseSegment &segment : term_result.segments) {
            boundaries.append({segment.bounds.first(), true, &segment});
            boundaries.append({segment.bounds.one_after_last(), false, &segment});
          }
        }
        sort_boundaries(boundaries);
        evaluate_coarse_union(boundaries, expr_result);
        break;
      }
      case Expr::Type::Intersection: {
        const IntersectionExpr &expr = expression->as_intersection();
        Vector<CourseBoundary, 16> boundaries;
        for (const Expr *term : expr.terms) {
          const CoarseResult &term_result = *expression_results[term->index];
          for (const CoarseSegment &segment : term_result.segments) {
            boundaries.append({segment.bounds.first(), true, &segment});
            boundaries.append({segment.bounds.one_after_last(), false, &segment});
          }
        }
        sort_boundaries(boundaries);
        evaluate_coarse_intersection(boundaries, expr.terms.size(), expr_result);
        break;
      }
      case Expr::Type::Difference: {
        const DifferenceExpr &expr = expression->as_difference();
        Vector<std::pair<CourseBoundary, bool>, 16> boundaries;
        const CoarseResult &main_term_result = *expression_results[expr.terms[0]->index];
        for (const CoarseSegment &segment : main_term_result.segments) {
          boundaries.append({{segment.bounds.first(), true, &segment}, true});
          boundaries.append({{segment.bounds.one_after_last(), false, &segment}, true});
        }
        for (const Expr *term : expr.terms.as_span().drop_front(1)) {
          const CoarseResult &term_result = *expression_results[term->index];
          for (const CoarseSegment &segment : term_result.segments) {
            boundaries.append({{segment.bounds.first(), true, &segment}, false});
            boundaries.append({{segment.bounds.one_after_last(), false, &segment}, false});
          }
        }
        std::sort(boundaries.begin(),
                  boundaries.end(),
                  [](const std::pair<CourseBoundary, bool> &a,
                     const std::pair<CourseBoundary, bool> &b) {
                    return a.first.index < b.first.index;
                  });
        evaluate_coarse_difference(boundaries, expr_result);
        break;
      }
    }
  }

  const CoarseResult &final_result = *expression_results[root_expression.index];
  return final_result;
}

BLI_NOINLINE static void indices_to_bits(const int16_t *indices,
                                         const int64_t indices_num,
                                         uint64_t *r_bits,
                                         const int64_t offset)
{
  for (int64_t i = 0; i < indices_num; i++) {
    const uint64_t index = uint64_t(indices[i] + offset);
    r_bits[index >> bits::BitToIntIndexShift] |= bits::mask_single_bit(index & bits::BitIndexMask);
  }
}

BLI_NOINLINE static void mask_to_bits(const IndexMask &mask,
                                      MutableBitSpan r_bits,
                                      const int64_t offset)
{
  mask.foreach_segment_optimized([&](const auto segment) {
    if constexpr (std::is_same_v<std::decay_t<decltype(segment)>, IndexRange>) {
      const IndexRange range = segment;
      const IndexRange shifted_range = range.shift(offset);
      r_bits.slice(shifted_range).set_all();
    }
    else {
      const IndexMaskSegment indices = segment;
      indices_to_bits(
          indices.base_span().data(), indices.size(), r_bits.data(), indices.offset() + offset);
    }
  });
}

BLI_NOINLINE static Span<int16_t> bits_to_indices(const BoundedBitSpan bits,
                                                  LinearAllocator<> &allocator)
{
  Vector<int16_t, max_segment_size> indices_vec;
  bits::foreach_1_index(bits, [&](const int64_t i) {
    BLI_assert(i < max_segment_size);
    indices_vec.append(int16_t(i));
  });
  return allocator.construct_array_copy<int16_t>(indices_vec);
}

BLI_NOINLINE static IndexMaskSegment evaluate_exact_with_bits(
    const Expr &root_expression,
    LinearAllocator<> &allocator,
    const IndexRange bounds,
    const Span<const Expr *> eager_eval_order)
{
  BLI_assert(bounds.size() <= max_segment_size);
  const int64_t segment_offset = bounds.start();
  const int expr_array_size = root_expression.expression_array_size();
  const int64_t ints_in_bounds = ceil_division(bounds.size(), bits::BitsPerInt);

  BitGroupVector<1024 * 16> expression_results(
      expr_array_size, ints_in_bounds * bits::BitsPerInt, false);

  for (const Expr *expression : eager_eval_order) {
    MutableBoundedBitSpan expr_result = expression_results[expression->index];
    switch (expression->type) {
      case Expr::Type::Atomic: {
        const auto &expr = expression->as_atomic();
        const IndexMask mask = expr.mask->slice_content(bounds);
        mask_to_bits(mask, expr_result, -segment_offset);
        break;
      }
      case Expr::Type::Union: {
        for (const Expr *term : expression->terms) {
          expr_result |= expression_results[term->index];
        }
        break;
      }
      case Expr::Type::Intersection: {
        bits::copy_from_or(expr_result, expression_results[expression->terms[0]->index]);
        for (const Expr *term : expression->terms.as_span().drop_front(1)) {
          expr_result &= expression_results[term->index];
        }
        break;
      }
      case Expr::Type::Difference: {
        bits::copy_from_or(expr_result, expression_results[expression->terms[0]->index]);
        for (const Expr *term : expression->terms.as_span().drop_front(1)) {
          bits::mix_into_first_expr(
              [](const bits::BitInt a, const bits::BitInt b) { return a & ~b; },
              expr_result,
              expression_results[term->index]);
        }
        break;
      }
    }
  }
  const BoundedBitSpan final_bits = expression_results[root_expression.index];
  const Span<int16_t> indices = bits_to_indices(final_bits, allocator);
  return IndexMaskSegment(segment_offset, indices);
}

static int64_t union_index_mask_segments(const Span<IndexMaskSegment> segments, int16_t *r_values)
{
  if (segments.is_empty()) {
    return 0;
  }
  if (segments.size() == 1) {
    const IndexMaskSegment segment = segments[0];
    std::copy(segment.begin(), segment.end(), r_values);
    return segment.size();
  }
  if (segments.size() == 2) {
    const IndexMaskSegment a = segments[0];
    const IndexMaskSegment b = segments[1];
    return std::set_union(a.begin(), a.end(), b.begin(), b.end(), r_values) - r_values;
  }

  Vector<IndexMaskSegment> sorted_segments(segments);
  std::sort(
      sorted_segments.begin(),
      sorted_segments.end(),
      [](const IndexMaskSegment &a, const IndexMaskSegment &b) { return a.size() < b.size(); });

  std::array<int16_t, max_segment_size> tmp_indices;
  int16_t *buffer_a = r_values;
  int16_t *buffer_b = tmp_indices.data();

  if (sorted_segments.size() % 2 == 1) {
    std::swap(buffer_a, buffer_b);
  }

  int64_t count = 0;
  {
    /* Initial union. */
    const IndexMaskSegment a = sorted_segments[0];
    const IndexMaskSegment b = sorted_segments[1];
    int16_t *dst = buffer_a;
    count = std::set_union(a.begin(), a.end(), b.begin(), b.end(), dst) - dst;
  }

  for (const int64_t segment_i : sorted_segments.index_range().drop_front(2)) {
    const int16_t *a = buffer_a;
    const IndexMaskSegment b = sorted_segments[segment_i];
    int16_t *dst = buffer_b;
    count = std::set_union(a, a + count, b.begin(), b.end(), dst) - dst;
    std::swap(buffer_a, buffer_b);
  }
  return count;
}

static int64_t intersect_index_mask_segments(const Span<IndexMaskSegment> segments,
                                             int16_t *r_values)
{
  if (segments.is_empty()) {
    return 0;
  }
  if (segments.size() == 1) {
    const IndexMaskSegment segment = segments[0];
    std::copy(segment.begin(), segment.end(), r_values);
    return segment.size();
  }
  if (segments.size() == 2) {
    const IndexMaskSegment a = segments[0];
    const IndexMaskSegment b = segments[1];
    return std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), r_values) - r_values;
  }

  Vector<IndexMaskSegment> sorted_segments(segments);
  std::sort(
      sorted_segments.begin(),
      sorted_segments.end(),
      [](const IndexMaskSegment &a, const IndexMaskSegment &b) { return a.size() < b.size(); });

  std::array<int16_t, max_segment_size> tmp_indices;
  int16_t *buffer_a = r_values;
  int16_t *buffer_b = tmp_indices.data();

  int64_t count = 0;
  {
    /* Initial intersection. */
    const IndexMaskSegment a = sorted_segments[0];
    const IndexMaskSegment b = sorted_segments[1];
    int16_t *dst = buffer_a;
    count = std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), dst) - dst;
  }

  for (const int64_t segment_i : sorted_segments.index_range().drop_front(2)) {
    const int16_t *a = buffer_a;
    const IndexMaskSegment b = sorted_segments[segment_i];
    int16_t *dst = buffer_b;
    count = std::set_intersection(a, a + count, b.begin(), b.end(), dst) - dst;
    std::swap(buffer_a, buffer_b);
  }
  return count;
}

BLI_NOINLINE static IndexMaskSegment evaluate_exact_with_indices(
    const Expr &root_expression,
    LinearAllocator<> &allocator,
    const IndexRange bounds,
    const Span<const Expr *> eager_eval_order)
{
  BLI_assert(bounds.size() <= max_segment_size);
  const int64_t bound_min = bounds.start();
  const int expr_array_size = root_expression.expression_array_size();
  Array<IndexMaskSegment, inline_expr_array_size> results(expr_array_size);
  for (const Expr *expression : eager_eval_order) {
    switch (expression->type) {
      case Expr::Type::Atomic: {
        const auto &expr = expression->as_atomic();
        const IndexMask mask = expr.mask->slice_content(bounds);
        /* The caller should make sure that the bounds are aligned to segment bounds. */
        BLI_assert(mask.segments_num() <= 1);
        if (mask.segments_num() == 1) {
          results[expression->index] = mask.segment(0);
        }
        break;
      }
      case Expr::Type::Union: {
        const auto &expr = expression->as_union();
        Array<IndexMaskSegment> term_segments(expr.terms.size());
        int64_t result_size_upper_bound = 0;
        bool used_short_circuit = false;
        for (const int64_t term_i : expr.terms.index_range()) {
          const Expr &term = *expr.terms[term_i];
          const IndexMaskSegment term_segment = results[term.index];
          if (term_segment.size() == bounds.size()) {
            results[expression->index] = term_segment;
            used_short_circuit = true;
            break;
          }
          result_size_upper_bound += term_segment.size();
          term_segments[term_i] = IndexMaskSegment(term_segment.offset() - bound_min,
                                                   term_segment.base_span());
        }
        if (used_short_circuit) {
          break;
        }
        result_size_upper_bound = std::min(result_size_upper_bound, bounds.size());
        int16_t *dst = allocator.allocate_array<int16_t>(result_size_upper_bound).data();
        const int64_t result_size = union_index_mask_segments(term_segments, dst);
        results[expression->index] = IndexMaskSegment(bound_min, {dst, result_size});
        break;
      }
      case Expr::Type::Intersection: {
        const auto &expr = expression->as_intersection();
        Array<IndexMaskSegment> term_segments(expr.terms.size());
        int64_t result_size_upper_bound = bounds.size();
        bool used_short_circuit = false;
        for (const int64_t term_i : expr.terms.index_range()) {
          const Expr &term = *expr.terms[term_i];
          const IndexMaskSegment term_segment = results[term.index];
          if (term_segment.is_empty()) {
            results[expression->index] = {};
            used_short_circuit = true;
            break;
          }
          result_size_upper_bound = std::min(result_size_upper_bound, term_segment.size());
          term_segments[term_i] = IndexMaskSegment(term_segment.offset() - bound_min,
                                                   term_segment.base_span());
        }
        if (used_short_circuit) {
          break;
        }
        int16_t *dst = allocator.allocate_array<int16_t>(result_size_upper_bound).data();
        const int64_t result_size = intersect_index_mask_segments(term_segments, dst);
        results[expression->index] = IndexMaskSegment(bound_min, {dst, result_size});
        break;
      }
      case Expr::Type::Difference: {
        break;
      }
    }
  }
  return results[root_expression.index];
}

BLI_NOINLINE static Vector<IndexMaskSegment> build_result_mask_segments(
    const Span<EvaluatedSegment> evaluated_segments)
{
  const std::array<int16_t, max_segment_size> &static_indices_array = get_static_indices_array();

  Vector<IndexMaskSegment> result_mask_segments;
  for (const EvaluatedSegment &evaluated_segment : evaluated_segments) {
    switch (evaluated_segment.type) {
      case EvaluatedSegment::Type::Full: {
        const int64_t full_size = evaluated_segment.bounds.size();
        for (int64_t i = 0; i < full_size; i += max_segment_size) {
          const int64_t size = std::min(i + max_segment_size, full_size) - i;
          result_mask_segments.append(IndexMaskSegment(
              evaluated_segment.bounds.first() + i, Span(static_indices_array).take_front(size)));
        }
        break;
      }
      case EvaluatedSegment::Type::Copy: {
        const IndexMask sliced_mask = evaluated_segment.copy_mask->slice_content(
            evaluated_segment.bounds);
        sliced_mask.foreach_segment(
            [&](const IndexMaskSegment &segment) { result_mask_segments.append(segment); });
        break;
      }
      case EvaluatedSegment::Type::Indices: {
        result_mask_segments.append(evaluated_segment.indices);
        break;
      }
    }
  }
  return result_mask_segments;
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

static ExactEvalMode determine_exact_eval_mode(const Expr &root_expression)
{
  for (const Expr *term : root_expression.terms) {
    if (!term->terms.is_empty()) {
      /* Use bits when there are nested expressions as this is often faster. */
      return ExactEvalMode::Bits;
    }
  }
  return ExactEvalMode::Indices;
}

static IndexMaskSegment evaluate_exact_segment(const Expr &root_expression,
                                               LinearAllocator<> &allocator,
                                               const ExactEvalMode eval_mode,
                                               const IndexRange bounds,
                                               const Span<const Expr *> eager_eval_order)
{
  switch (eval_mode) {
    case ExactEvalMode::Bits:
      return evaluate_exact_with_bits(root_expression, allocator, bounds, eager_eval_order);
    case ExactEvalMode::Indices:
      return evaluate_exact_with_indices(root_expression, allocator, bounds, eager_eval_order);
  }
  BLI_assert_unreachable();
  return {};
}

BLI_NOINLINE static IndexMask evaluate_expression_impl(const Expr &root_expression,
                                                       IndexMaskMemory &memory)
{
  Vector<EvaluatedSegment, 16> evaluated_segments;
  Stack<IndexRange, 16> long_unknown_segments;
  Vector<IndexRange, 16> short_unknown_segments;

  const ExactEvalMode exact_eval_mode = determine_exact_eval_mode(root_expression);
  const int64_t long_segment_size_threshold = (exact_eval_mode == ExactEvalMode::Indices) ?
                                                  max_segment_size :
                                                  1024;

  const Vector<const Expr *, inline_expr_array_size> eager_eval_order = compute_eager_eval_order(
      root_expression);

  auto handle_coarse_result = [&](const CoarseResult &coarse_result) {
    for (const CoarseSegment &segment : coarse_result.segments) {
      switch (segment.type) {
        case CoarseSegment::Type::Unknown: {
          if (segment.bounds.size() > long_segment_size_threshold) {
            long_unknown_segments.push(segment.bounds);
          }
          else {
            short_unknown_segments.append(segment.bounds);
          }
          break;
        }
        case CoarseSegment::Type::Copy: {
          BLI_assert(segment.mask);
          evaluated_segments.append({EvaluatedSegment::Type::Copy, segment.bounds, segment.mask});
          break;
        }
        case CoarseSegment::Type::Full: {
          evaluated_segments.append({EvaluatedSegment::Type::Full, segment.bounds});
          break;
        }
      }
    }
  };
  const CoarseResult initial_coarse_result = evaluate_coarse(root_expression, eager_eval_order);
  handle_coarse_result(initial_coarse_result);

  while (!long_unknown_segments.is_empty()) {
    const IndexRange unknown_bounds = long_unknown_segments.pop();
    const int64_t split_pos = unknown_bounds.size() / 2;
    const IndexRange left_half = unknown_bounds.take_front(split_pos);
    const IndexRange right_half = unknown_bounds.drop_front(split_pos);
    const CoarseResult left_result = evaluate_coarse(root_expression, eager_eval_order, left_half);
    const CoarseResult right_result = evaluate_coarse(
        root_expression, eager_eval_order, right_half);
    handle_coarse_result(left_result);
    handle_coarse_result(right_result);
  }

  auto evaluate_unknown_segment = [&](const IndexRange bounds,
                                      LinearAllocator<> &allocator,
                                      Vector<EvaluatedSegment, 16> &r_evaluated_segments) {
    const IndexMaskSegment indices = evaluate_exact_segment(
        root_expression, allocator, exact_eval_mode, bounds, eager_eval_order);
    if (!indices.is_empty()) {
      r_evaluated_segments.append({EvaluatedSegment::Type::Indices, bounds, nullptr, indices});
    }
  };

  const int64_t unknown_segment_eval_grain_size = 8;
  if (short_unknown_segments.size() < unknown_segment_eval_grain_size) {
    for (const IndexRange &bounds : short_unknown_segments) {
      evaluate_unknown_segment(bounds, memory, evaluated_segments);
    }
  }
  else {
    struct LocalData {
      LinearAllocator<> allocator;
      Vector<EvaluatedSegment, 16> evaluated_segments;
    };
    threading::EnumerableThreadSpecific<LocalData> data_by_thread;
    threading::parallel_for(
        short_unknown_segments.index_range(),
        unknown_segment_eval_grain_size,
        [&](const IndexRange range) {
          LocalData &data = data_by_thread.local();
          for (const IndexRange &bounds : short_unknown_segments.as_span().slice(range)) {
            evaluate_unknown_segment(bounds, data.allocator, data.evaluated_segments);
          }
        });
    for (LocalData &data : data_by_thread) {
      if (!data.evaluated_segments.is_empty()) {
        evaluated_segments.extend(data.evaluated_segments);
        memory.transfer_ownership_from(data.allocator);
      }
    }
  }

  if (evaluated_segments.is_empty()) {
    return {};
  }
  if (evaluated_segments.size() == 1) {
    const EvaluatedSegment &evaluated_segment = evaluated_segments[0];
    switch (evaluated_segment.type) {
      case EvaluatedSegment::Type::Full: {
        return IndexMask(IndexRange(evaluated_segment.bounds));
      }
      case EvaluatedSegment::Type::Copy: {
        return evaluated_segment.copy_mask->slice_content(evaluated_segment.bounds);
      }
      case EvaluatedSegment::Type::Indices: {
        return IndexMask::from_segments({evaluated_segment.indices}, memory);
      }
    }
  }

  std::sort(evaluated_segments.begin(),
            evaluated_segments.end(),
            [](const EvaluatedSegment &a, const EvaluatedSegment &b) {
              return a.bounds.start() < b.bounds.start();
            });

  Vector<IndexMaskSegment> result_segments = build_result_mask_segments(evaluated_segments);
  return IndexMask::from_segments(result_segments, memory);
}

IndexMask evaluate_expression(const Expr &expression, IndexMaskMemory &memory)
{
  return evaluate_expression_impl(expression, memory);
}

}  // namespace blender::index_mask
