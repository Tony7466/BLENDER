/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"
#include "BLI_math_geom.h"
#include "BLI_set.hh"
#include "BLI_stack.hh"

#include "BKE_curves_utils.hh"

#include "GEO_simplify_curves.hh"

namespace blender::geometry {

/**
 * An implementation of the Ramer-Douglas-Peucker algorithm.
 *
 * \param range: The range to simplify.
 * \param epsilon: The threshold distance from the coord between two points for when a point
 * in-between needs to be kept.
 * \param dist_function: A function that computes the distance to a point at an index in the range.
 * The IndexRange is a subrange of \a range and the index is an index relative to the subrange.
 * \param points_to_delete: Writes true to the indices for which the points should be removed.
 */
static void ramer_douglas_peucker(
    const IndexRange range,
    const float epsilon,
    const FunctionRef<float(int64_t, int64_t, int64_t)> dist_function,
    MutableSpan<bool> points_to_delete)
{
  /* Mark all points to be kept. */
  points_to_delete.slice(range).fill(false);

  Stack<IndexRange> stack;
  stack.push(range);
  while (!stack.is_empty()) {
    const IndexRange sub_range = stack.pop();
    /* Skip ranges with less than 3 points. All points are kept. */
    if (sub_range.size() < 3) {
      continue;
    }
    const IndexRange inside_range = sub_range.drop_front(1).drop_back(1);
    /* Compute the maximum distance and the corresponding index. */
    float max_dist = -1.0f;
    int max_index = -1;
    for (const int64_t index : inside_range) {
      const float dist = dist_function(sub_range.first(), sub_range.last(), index);
      if (dist > max_dist) {
        max_dist = dist;
        max_index = index - sub_range.first();
      }
    }

    if (max_dist > epsilon) {
      /* Found point outside the epsilon-sized strip. The point at `max_index` will be kept, repeat
       * the search on the left & right side. */
      stack.push(sub_range.slice(0, max_index + 1));
      stack.push(sub_range.slice(max_index, sub_range.size() - max_index));
    }
    else {
      /* Points in `sub_range` are inside the epsilon-sized strip. Mark them to be deleted. */
      points_to_delete.slice(inside_range).fill(true);
    }
  }
}

static void curve_simplifiy(const IndexRange points,
                            const bool cyclic,
                            const float epsilon,
                            const FunctionRef<float(int64_t, int64_t, int64_t)> dist_function,
                            MutableSpan<bool> points_to_delete)
{
  const Span<bool> curve_selection = points_to_delete.slice(points);
  if (!curve_selection.contains(true)) {
    return;
  }

  const bool is_last_segment_selected = (curve_selection.first() && curve_selection.last());

  const Vector<IndexRange> selection_ranges = array_utils::find_all_ranges(curve_selection, true);
  threading::parallel_for(
      selection_ranges.index_range(), 1024, [&](const IndexRange range_of_ranges) {
        for (const IndexRange range : selection_ranges.as_span().slice(range_of_ranges)) {
          ramer_douglas_peucker(
              range.shift(points.start()), epsilon, dist_function, points_to_delete);
        }
      });

  /* For cyclic curves, handle the last segment. */
  if (cyclic && points.size() > 2 && is_last_segment_selected) {
    const float dist = dist_function(points.last(1), points.first(), points.last());
    if (dist <= epsilon) {
      points_to_delete[points.last()] = true;
    }
  }
}

CurvesGeometry curves_simplify(const CurvesGeometry &src_curves,
                               const IndexMask &selection,
                               const float epsilon)
{
  CurvesGeometry dst_curves(src_curves);

  const Span<float3> positions = src_curves.positions();
  const VArray<bool> cyclic = src_curves.cyclic();
  const OffsetIndices<int> points_by_curve = src_curves.points_by_curve();

  /* Distance functions for `ramer_douglas_peucker_simplify`. */
  const auto dist_function_positions =
      [positions](int64_t first_index, int64_t last_index, int64_t index) {
        const float dist_position = dist_to_line_v3(
            positions[index], positions[first_index], positions[last_index]);
        return dist_position;
      };

  Array<bool> points_to_delete(src_curves.points_num(), false);
  bke::curves::fill_points(points_by_curve, selection, true, points_to_delete.as_mutable_span());

  selection.foreach_index(GrainSize(512), [&](const int64_t curve_i) {
    const IndexRange points = points_by_curve[curve_i];
    if (epsilon > 0.0f) {
      curve_simplifiy(points,
                      cyclic[curve_i],
                      epsilon,
                      dist_function_positions,
                      points_to_delete.as_mutable_span());
    }
    else {
      points_to_delete.as_mutable_span().slice(points).fill(false);
    }
  });

  IndexMaskMemory memory;
  dst_curves.remove_points(IndexMask::from_bools(points_to_delete, memory), {});
  return dst_curves;
}

}  // namespace blender::geometry
