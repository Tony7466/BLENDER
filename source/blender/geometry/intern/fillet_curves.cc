/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"

#include "BLI_math_rotation_legacy.hh"
#include "BLI_task.hh"

#include "GEO_fillet_curves.hh"

namespace blender::geometry {
static void duplicate_fillet_point_data(const OffsetIndices<int> src_points_by_curve,
                                        const OffsetIndices<int> dst_points_by_curve,
                                        const IndexMask &curve_selection,
                                        const Span<int> all_point_offsets,
                                        const GSpan src,
                                        GMutableSpan dst)
{
  curve_selection.foreach_index(GrainSize(512), [&](const int curve_i) {
    const IndexRange src_points = src_points_by_curve[curve_i];
    const IndexRange dst_points = dst_points_by_curve[curve_i];
    const IndexRange offsets_range = bke::curves::per_curve_point_offsets_range(src_points,
                                                                                curve_i);
    bke::attribute_math::gather_to_groups(all_point_offsets.slice(offsets_range),
                                          IndexRange(src_points.size()),
                                          src.slice(src_points),
                                          dst.slice(dst_points));
  });
}

static void calculate_result_offsets(const OffsetIndices<int> curve_start_indices,
                                     const IndexMask &selection,
                                     const IndexMask &unselected,
                                     const VArray<float> &radii,
                                     const VArray<int> &counts,
                                     const Span<bool> cyclic,
                                     MutableSpan<int> dst_curve_offsets,
                                     MutableSpan<int> dst_point_offsets)
{
  /* Fill the offsets array with the curve point counts, then accumulate them to form offsets. */
  offset_indices::copy_group_sizes(curve_start_indices, unselected, dst_curve_offsets);
  selection.foreach_index(GrainSize(512), [&](const int curve_i) {
    // This is the points representing a single curve in the curve list
    const IndexRange src_points = curve_start_indices[curve_i];
    // ??? [INSPECT] I don't understand this one
    const IndexRange offsets_range = bke::curves::per_curve_point_offsets_range(src_points,
                                                                                curve_i);

    // Get the mutable span of point offsets for the destination
    MutableSpan<int> point_offsets = dst_point_offsets.slice(offsets_range);
    // The last elements is the total count of points in the curve
    MutableSpan<int> point_counts = point_offsets.drop_back(1);

    // Copy the "counts" array into the "dst_point_offsets" array at the indices defined by
    // "src_points_by_curve"
    counts.materialize_compressed(src_points, point_counts);
    for (int &count : point_counts) {
      /* Make sure the number of cuts is greater than zero and add one for the existing point. */
      count = std::max(count, 0) + 1;
    }

    if (!cyclic[curve_i]) {
      /* Endpoints on non-cyclic curves cannot be filleted. */
      point_counts.first() = 1;
      point_counts.last() = 1;
    }

    /* Implicitly "deselect" points with zero radius. */
    devirtualize_varray(radii, [&](const auto radii) {
      for (const int i : IndexRange(src_points.size())) {
        if (radii[src_points[i]] == 0.0f) {
          point_counts[i] = 1;
        }
      }
    });

    offset_indices::accumulate_counts_to_offsets(point_offsets);

    dst_curve_offsets[curve_i] = point_offsets.last();
  });
  offset_indices::accumulate_counts_to_offsets(dst_curve_offsets);
}

static void calculate_directions(const Span<float3> positions, MutableSpan<float3> directions)
{
  for (const int i : positions.index_range().drop_back(1)) {
    directions[i] = math::normalize(positions[i + 1] - positions[i]);
  }
  directions.last() = math::normalize(positions.first() - positions.last());
}

static void calculate_angles(const Span<float3> directions, MutableSpan<float> angles)
{
  angles.first() = M_PI - angle_v3v3(-directions.last(), directions.first());
  for (const int i : directions.index_range().drop_front(1)) {
    angles[i] = M_PI - angle_v3v3(-directions[i - 1], directions[i]);
  }
}

static void limit_radii(const Span<float3> positions,
                        const Span<float> angles,
                        const Span<float> radii,
                        const bool cyclic,
                        MutableSpan<float> radii_clamped,
                        MutableSpan<bool> isDuplicated)
{
  /* Previous, current, and next values will be needed simultaneously.
   * Calculate the variables needed at each stage in advance.
   */
  Array<float> displacements(positions.size());
  for (const int i : positions.index_range()) {
    displacements[i] = radii[i] * std::tan(angles[i] / 2.0f);
  }

  IndexRange positions_range = positions.index_range();
  const int i_last = positions_range.last();
  for (const int i : positions_range) {
    int i_prev = i - 1;
    int i_next = i + 1;

    /* Handle the first point. */
    if (i == 0) {
      if (cyclic) {
        /* The previous point on cyclic curves */
        i_prev = i_last;
      }
      else {
        /* Use a zero radius for the first and last points, because they don't have fillets.
         * This logic could potentially be unrolled, but it doesn't seem worth it. */
        radii_clamped.first() = 0.0f;
        continue;
      }
    }

    /* Handle the last point */
    if (i == i_last) {
      if (cyclic) {
        /* The next point on cyclic curves */
        i_next = 0;
      }
      else {
        radii_clamped.last() = 0.0f;
        continue;
      }
    }

    /**
     * Find the portion of the previous and next segments used by the current and next point
     * fillets. If more than the total length of the segment would be used, scale the current
     * point's radius just enough to make the two points meet in the middle.
     */
    const float radius = radii[i];
    const float3 position = positions[i];
    const float displacement = displacements[i];

    const float3 position_prev = positions[i_prev];
    const float displacement_prev = displacements[i_prev];
    const float segment_length_prev = math::distance(position, position_prev);
    const float total_displacement_prev = displacement_prev + displacement;
    const float factor_prev = std::clamp(
        math::safe_divide(segment_length_prev, total_displacement_prev), 0.0f, 1.0f);

    const float3 position_next = positions[i_next];
    const float displacement_next = displacements[i_next];
    const float segment_length_next = math::distance(position, position_next);
    const float total_displacement_next = displacement_next + displacement;
    const float factor_next = std::clamp(
        math::safe_divide(segment_length_next, total_displacement_next), 0.0f, 1.0f);

    /*
     * The lower of the two factors is the location of the duplicate point
     * If it is the next location, that point should be removed.
     */

    const float min_factor = std::min(factor_prev, factor_next);
    radii_clamped[i] = radius * min_factor;
    isDuplicated[i] = min_factor < 1 && factor_next <= factor_prev;
  }
}

static void calculate_fillet_positions(const Span<float3> src_positions,
                                       const Span<float> angles,
                                       const Span<float> radii,
                                       const Span<float3> directions,
                                       const OffsetIndices<int> dst_offsets,
                                       MutableSpan<float3> dst)
{
  const int i_src_last = src_positions.index_range().last();
  threading::parallel_for(src_positions.index_range(), 512, [&](IndexRange range) {
    for (const int i_src : range) {
      const IndexRange arc = dst_offsets[i_src];
      const float3 &src = src_positions[i_src];
      if (arc.size() == 1) {
        dst[arc.first()] = src;
        continue;
      }

      const int i_src_prev = i_src == 0 ? i_src_last : i_src - 1;
      const float angle = angles[i_src];
      const float radius = radii[i_src];
      const float displacement = radius * std::tan(angle / 2.0f);
      const float3 prev_dir = -directions[i_src_prev];
      const float3 &next_dir = directions[i_src];
      const float3 arc_start = src + prev_dir * displacement;
      const float3 arc_end = src + next_dir * displacement;

      dst[arc.first()] = arc_start;
      dst[arc.last()] = arc_end;

      const IndexRange middle = arc.drop_front(1).drop_back(1);
      if (middle.is_empty()) {
        continue;
      }

      const float3 axis = -math::normalize(math::cross(prev_dir, next_dir));
      const float3 center_direction = math::normalize(math::midpoint(next_dir, prev_dir));
      const float distance_to_center = std::sqrt(pow2f(radius) + pow2f(displacement));
      const float3 center = src + center_direction * distance_to_center;

      /* Rotate each middle fillet point around the center. */
      const float segment_angle = angle / (middle.size() + 1);
      for (const int i : IndexRange(middle.size())) {
        const int point_i = middle[i];
        dst[point_i] = math::rotate_around_axis(arc_start, center, axis, segment_angle * (i + 1));
      }
    }
  });
}

/**
 * Set handles for the "Bezier" mode where we rely on setting the inner handles to approximate a
 * circular arc. The outer (previous and next) handles outside the result fillet segment are set
 * to vector handles.
 */
static void calculate_bezier_handles_bezier_mode(const Span<float3> src_handles_l,
                                                 const Span<float3> src_handles_r,
                                                 const Span<int8_t> src_types_l,
                                                 const Span<int8_t> src_types_r,
                                                 const Span<float> angles,
                                                 const Span<float> radii,
                                                 const Span<float3> directions,
                                                 const OffsetIndices<int> dst_offsets,
                                                 const Span<float3> dst_positions,
                                                 MutableSpan<float3> dst_handles_l,
                                                 MutableSpan<float3> dst_handles_r,
                                                 MutableSpan<int8_t> dst_types_l,
                                                 MutableSpan<int8_t> dst_types_r)
{
  const int i_src_last = src_handles_l.index_range().last();
  const int i_dst_last = dst_positions.index_range().last();
  threading::parallel_for(src_handles_l.index_range(), 512, [&](IndexRange range) {
    for (const int i_src : range) {
      const IndexRange arc = dst_offsets[i_src];
      if (arc.size() == 1) {
        dst_handles_l[arc.first()] = src_handles_l[i_src];
        dst_handles_r[arc.first()] = src_handles_r[i_src];
        dst_types_l[arc.first()] = src_types_l[i_src];
        dst_types_r[arc.first()] = src_types_r[i_src];
        continue;
      }
      BLI_assert(arc.size() == 2);
      const int i_dst_a = arc.first();
      const int i_dst_b = arc.last();

      const int i_src_prev = i_src == 0 ? i_src_last : i_src - 1;
      const float angle = angles[i_src];
      const float radius = radii[i_src];
      const float3 prev_dir = -directions[i_src_prev];
      const float3 &next_dir = directions[i_src];

      const float3 &arc_start = dst_positions[arc.first()];
      const float3 &arc_end = dst_positions[arc.last()];

      /* Calculate the point's handles on the outside of the fillet segment,
       * connecting to the next or previous result points. */
      const int i_dst_prev = i_dst_a == 0 ? i_dst_last : i_dst_a - 1;
      const int i_dst_next = i_dst_b == i_dst_last ? 0 : i_dst_b + 1;
      dst_handles_l[i_dst_a] = bke::curves::bezier::calculate_vector_handle(
          dst_positions[i_dst_a], dst_positions[i_dst_prev]);
      dst_handles_r[i_dst_b] = bke::curves::bezier::calculate_vector_handle(
          dst_positions[i_dst_b], dst_positions[i_dst_next]);
      dst_types_l[i_dst_a] = BEZIER_HANDLE_VECTOR;
      dst_types_r[i_dst_b] = BEZIER_HANDLE_VECTOR;

      /* The inner handles are aligned with the aligned with the outer vector
       * handles, but have a specific length to best approximate a circle. */
      const float handle_length = (4.0f / 3.0f) * radius * std::tan(angle / 4.0f);
      dst_handles_r[i_dst_a] = arc_start - prev_dir * handle_length;
      dst_handles_l[i_dst_b] = arc_end - next_dir * handle_length;
      dst_types_r[i_dst_a] = BEZIER_HANDLE_ALIGN;
      dst_types_l[i_dst_b] = BEZIER_HANDLE_ALIGN;
    }
  });
}

/**
 * In the poly fillet mode, all the inner handles are set to vector handles, along with the "outer"
 * (previous and next) handles at each fillet.
 */
static void calculate_bezier_handles_poly_mode(const Span<float3> src_handles_l,
                                               const Span<float3> src_handles_r,
                                               const Span<int8_t> src_types_l,
                                               const Span<int8_t> src_types_r,
                                               const OffsetIndices<int> dst_offsets,
                                               const Span<float3> dst_positions,
                                               MutableSpan<float3> dst_handles_l,
                                               MutableSpan<float3> dst_handles_r,
                                               MutableSpan<int8_t> dst_types_l,
                                               MutableSpan<int8_t> dst_types_r)
{
  const int i_dst_last = dst_positions.index_range().last();
  threading::parallel_for(src_handles_l.index_range(), 512, [&](IndexRange range) {
    for (const int i_src : range) {
      const IndexRange arc = dst_offsets[i_src];
      if (arc.size() == 1) {
        dst_handles_l[arc.first()] = src_handles_l[i_src];
        dst_handles_r[arc.first()] = src_handles_r[i_src];
        dst_types_l[arc.first()] = src_types_l[i_src];
        dst_types_r[arc.first()] = src_types_r[i_src];
        continue;
      }

      /* The fillet's next and previous handles are vector handles, as are the inner handles. */
      dst_types_l.slice(arc).fill(BEZIER_HANDLE_VECTOR);
      dst_types_r.slice(arc).fill(BEZIER_HANDLE_VECTOR);

      /* Calculate the point's handles on the outside of the fillet segment. This point
       * won't be selected for a fillet if it is the first or last in a non-cyclic curve. */

      const int i_dst_prev = arc.first() == 0 ? i_dst_last : arc.one_before_start();
      const int i_dst_next = arc.last() == i_dst_last ? 0 : arc.one_after_last();
      dst_handles_l[arc.first()] = bke::curves::bezier::calculate_vector_handle(
          dst_positions[arc.first()], dst_positions[i_dst_prev]);
      dst_handles_r[arc.last()] = bke::curves::bezier::calculate_vector_handle(
          dst_positions[arc.last()], dst_positions[i_dst_next]);

      /* Set the values for the inner handles. */
      const IndexRange middle = arc.drop_front(1).drop_back(1);
      for (const int i : middle) {
        dst_handles_r[i] = bke::curves::bezier::calculate_vector_handle(dst_positions[i],
                                                                        dst_positions[i - 1]);
        dst_handles_l[i] = bke::curves::bezier::calculate_vector_handle(dst_positions[i],
                                                                        dst_positions[i + 1]);
      }
    }
  });
}

/** Print contents of span
 */
static void print_spanf(const Span<float> span)
{
  for (const int i : span.index_range()) {
    printf("%f ", span[i]);
  }
  printf("\n");
}

/** Print contents of span
 */
static void print_span3f(const Span<float3> span)
{
  for (const int i : span.index_range()) {
    printf("%f %f %f\n", span[i].x, span[i].y, span[i].z);
  }
  printf("\n");
}

/** Print contents of span
 */
static void print_spani(const Span<int> span)
{
  for (const int i : span.index_range()) {
    printf("%d ", span[i]);
  }
  printf("\n");
}

static bke::CurvesGeometry fillet_curves(
    const bke::CurvesGeometry &src_curves,
    const IndexMask &curve_selection,
    const VArray<float> &radius_input,
    const VArray<int> &counts,
    const bool limit_radius,
    bool remove_duplicated_points,
    const bool use_bezier_mode,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  // This is the start index of every curve
  const OffsetIndices curve_start_indices = src_curves.points_by_curve();
  // This is the contiguous array of all positions available
  const Span<float3> positions = src_curves.positions();
  // Info about each curve, whether it is cyclic or not
  const VArraySpan<bool> cyclic{src_curves.cyclic()};
  // ?? What is an attribute in this context?
  const bke::AttributeAccessor src_attributes = src_curves.attributes();
  // ?? What kind of memory?
  IndexMaskMemory memory;
  const IndexMask unselected = curve_selection.complement(src_curves.curves_range(), memory);

  // Create mostly un-initialized curve geometry
  bke::CurvesGeometry dst_curves = bke::curves::copy_only_curve_domain(src_curves);
  bke::CurvesGeometry dst_curves_2;
  /* Stores the offset of every result point for every original point.
   * The extra length is used in order to store an extra zero for every curve. */
  Array<int> dst_point_offsets(src_curves.points_num() + src_curves.curves_num());
  MutableSpan<int> dst_curve_offsets = dst_curves.offsets_for_write();
  calculate_result_offsets(curve_start_indices,
                           curve_selection,
                           unselected,
                           radius_input,
                           counts,
                           cyclic,
                           dst_curve_offsets,
                           dst_point_offsets);
  OffsetIndices dst_points_by_curve = dst_curves.points_by_curve();
  Span<int> all_point_offsets = dst_point_offsets.as_span();
  const int dst_curve_size = dst_curve_offsets.last();

  dst_curves.resize(dst_curve_size, dst_curves.curves_num());
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();
  MutableSpan<float3> dst_positions = dst_curves.positions_for_write();

  VArraySpan<int8_t> src_types_l;
  VArraySpan<int8_t> src_types_r;
  Span<float3> src_handles_l;
  Span<float3> src_handles_r;
  MutableSpan<int8_t> dst_types_l;
  MutableSpan<int8_t> dst_types_r;
  MutableSpan<float3> dst_handles_l;
  MutableSpan<float3> dst_handles_r;
  if (src_curves.has_curve_with_type(CURVE_TYPE_BEZIER)) {
    src_types_l = src_curves.handle_types_left();
    src_types_r = src_curves.handle_types_right();
    src_handles_l = src_curves.handle_positions_left();
    src_handles_r = src_curves.handle_positions_right();

    dst_types_l = dst_curves.handle_types_left_for_write();
    dst_types_r = dst_curves.handle_types_right_for_write();
    dst_handles_l = dst_curves.handle_positions_left_for_write();
    dst_handles_r = dst_curves.handle_positions_right_for_write();
  }

  // Kludge - copy over the point counts for unselected curves
  Array<int> dst_2_curve_sizes(dst_curves.curves_num());
  Array<int> src_to_dst2_offsets(all_point_offsets.size());
  for (const int to_i : all_point_offsets.index_range()) {
    src_to_dst2_offsets[to_i] = all_point_offsets[to_i];
  }

  // Initialize to 0
  dst_2_curve_sizes.fill(0);
  unselected.foreach_segment(GrainSize(512), [&](const IndexMaskSegment segment) {
    for (const int curve_i : segment) {
      IndexRange curve_range = curve_start_indices[curve_i];
      dst_2_curve_sizes[curve_i] = curve_range.size();
    }
  });

  curve_selection.foreach_segment(GrainSize(512), [&](const IndexMaskSegment segment) {
    Array<float3> directions;
    Array<float> angles;
    Array<float> radii;
    Array<float> input_radii_buffer;
    Array<bool> is_duplicated_r;

    for (const int curve_i : segment) {
      const IndexRange src_points = curve_start_indices[curve_i];
      const IndexRange offsets_range = bke::curves::per_curve_point_offsets_range(src_points,
                                                                                  curve_i);
      const OffsetIndices<int> offsets(all_point_offsets.slice(offsets_range));
      const IndexRange dst_points = dst_points_by_curve[curve_i];
      const Span<float3> src_positions = positions.slice(src_points);

      directions.reinitialize(src_points.size());
      calculate_directions(src_positions, directions);

      angles.reinitialize(src_points.size());
      calculate_angles(directions, angles);

      radii.reinitialize(src_points.size());
      is_duplicated_r.reinitialize(src_points.size());

      if (limit_radius) {
        input_radii_buffer.reinitialize(src_points.size());
        radius_input.materialize_compressed(src_points, input_radii_buffer);
        limit_radii(
            src_positions, angles, input_radii_buffer, cyclic[curve_i], radii, is_duplicated_r);
      }
      else {
        radius_input.materialize_compressed(src_points, radii);
      }

      calculate_fillet_positions(positions.slice(src_points),
                                 angles,
                                 radii,
                                 directions,
                                 offsets,
                                 dst_positions.slice(dst_points));

      MutableSpan<float3> dst_positions_write = dst_positions.slice(dst_points);
      MutableSpan<float3> dst_handles_l_write = dst_handles_l.slice(dst_points);
      MutableSpan<float3> dst_handles_r_write = dst_handles_r.slice(dst_points);
      MutableSpan<int8_t> dst_types_l_write = dst_types_l.slice(dst_points);
      MutableSpan<int8_t> dst_types_r_write = dst_types_r.slice(dst_points);

      if (src_curves.has_curve_with_type(CURVE_TYPE_BEZIER)) {
        if (use_bezier_mode) {
          calculate_bezier_handles_bezier_mode(src_handles_l.slice(src_points),
                                               src_handles_r.slice(src_points),
                                               src_types_l.slice(src_points),
                                               src_types_r.slice(src_points),
                                               angles,
                                               radii,
                                               directions,
                                               offsets,
                                               dst_positions_write,
                                               dst_handles_l_write,
                                               dst_handles_r_write,
                                               dst_types_l_write,
                                               dst_types_r_write);
        }
        else {
          calculate_bezier_handles_poly_mode(src_handles_l.slice(src_points),
                                             src_handles_r.slice(src_points),
                                             src_types_l.slice(src_points),
                                             src_types_r.slice(src_points),
                                             offsets,
                                             dst_positions_write,
                                             dst_handles_l_write,
                                             dst_handles_r_write,
                                             dst_types_l_write,
                                             dst_types_r_write);
        }
      }

      if (limit_radius && remove_duplicated_points) {
        // Get input counts for the curve
        int new_curve_size = 0;

        int dst_i = 0;
        const int src_point_num = src_positions.size();

        bool skip_point = false;
        for (int src_i = 0; src_i < src_point_num; src_i++) {
          IndexRange dst_src_range = offsets[src_i];
          const bool skip_last_point = is_duplicated_r[src_i];
          int dst_src_i_last = dst_src_range.last();

          for (const int dst_src_i : dst_src_range) {
            if (!skip_point) {
              float3 handle_l = dst_handles_l_write[dst_src_i];
              int8_t type_l = dst_types_l_write[dst_src_i];

              dst_handles_l_write[dst_i] = handle_l;
              dst_types_l_write[dst_i] = type_l;
            }

            bool is_last_point = dst_src_i == dst_src_i_last;
            /* Skip the the last point of a curve and its right-side handles.
             * On the next loop, the left side handles will be skipped
             */
            skip_point = skip_last_point && is_last_point;
            if (!skip_point) {
              float3 pos = dst_positions_write[dst_src_i];
              float3 handle_r = dst_handles_r_write[dst_src_i];
              int8_t type_r = dst_types_r_write[dst_src_i];

              dst_positions_write[dst_i] = pos;
              dst_handles_r_write[dst_i] = handle_r;
              dst_types_r_write[dst_i] = type_r;
              dst_i++;
              new_curve_size++;
            }
            else {
              // Decrement all offsets
              int offsets_size = src_to_dst2_offsets.size();
              for (int dec_i = src_i; dec_i < offsets_size; dec_i++) {
                src_to_dst2_offsets[dec_i] = src_to_dst2_offsets[dec_i] - 1;
              }
            }
          }
        }
        dst_2_curve_sizes[curve_i] = new_curve_size;
      }
    }
  });

  OffsetIndices<int> curve_ranges_2;
  if (limit_radius && remove_duplicated_points) {
    // Temporary... figure out how to make a new set of destination curves based on what I now know
    dst_curves_2 = bke::curves::copy_only_curve_domain(src_curves);

    // Copy the new positions, handles, and types to the new destination curve object
    // This is a hack to understand how to work the destination in the final and is not the final
    MutableSpan<int> dst_curves_2_offsets = dst_curves_2.offsets_for_write();

    dst_curves_2_offsets[0] = 0;
    for (const int i : dst_2_curve_sizes.index_range()) {
      dst_curves_2_offsets[i + 1] = dst_curves_2_offsets[i] + dst_2_curve_sizes[i];
    }

    // Resize the new destination curves
    const int dst_curves_2_size = dst_curves_2_offsets.last();
    const int curve_num_2 = dst_2_curve_sizes.size();
    dst_curves_2.resize(dst_curves_2_size, curve_num_2);
    dst_attributes = dst_curves_2.attributes_for_write();

    // Duplicate points using destination curves 2
    OffsetIndices<int> curve_ranges = dst_curves.points_by_curve();
    curve_ranges_2 = dst_curves_2.points_by_curve();
    for (const int curve_i : curve_ranges_2.index_range()) {
      IndexRange dst_range = curve_ranges[curve_i];
      IndexRange dst_2_range = curve_ranges_2[curve_i];

      MutableSpan<float3> handle_l_w = dst_curves_2.handle_positions_left_for_write();
      MutableSpan<float3> handle_r_w = dst_curves_2.handle_positions_right_for_write();
      MutableSpan<int8_t> handle_l_t = dst_curves_2.handle_types_left_for_write();
      MutableSpan<int8_t> handle_r_t = dst_curves_2.handle_types_right_for_write();
      MutableSpan<float3> pos_w = dst_curves_2.positions_for_write();

      Span<float3> handle_l = dst_curves.handle_positions_left();
      Span<float3> handle_r = dst_curves.handle_positions_right();
      VArray<int8_t> handle_l_t_s = dst_curves.handle_types_left();
      VArray<int8_t> handle_r_t_s = dst_curves.handle_types_right();
      Span<float3> pos = dst_curves.positions();

      int i = dst_range.start();
      int end_2 = dst_2_range.last();
      for (int i_2 = dst_2_range.start(); i_2 <= end_2; i_2++) {
        printf("i: %d i_2: %d\n", i, i_2);
        handle_l_w[i_2] = handle_l[i];
        handle_r_w[i_2] = handle_r[i];
        handle_l_t[i_2] = handle_l_t_s[i];
        handle_r_t[i_2] = handle_r_t_s[i];
        pos_w[i_2] = pos[i];
        i++;
      }
    }

    dst_curves = dst_curves_2;
    dst_points_by_curve = curve_ranges_2;
    all_point_offsets = src_to_dst2_offsets.as_span();
  }

  printf("Right side Handles");
  print_span3f(dst_curves.handle_positions_right());

  printf("Left side Handles");
  print_span3f(dst_curves.handle_positions_left());

  printf("Positions");
  print_span3f(dst_curves.positions());

  for (auto &attribute : bke::retrieve_attributes_for_transfer(
           src_attributes,
           dst_attributes,
           ATTR_DOMAIN_MASK_POINT,
           propagation_info,
           {"position", "handle_type_left", "handle_type_right", "handle_right", "handle_left"}))
  {
    duplicate_fillet_point_data(curve_start_indices,
                                dst_points_by_curve,
                                curve_selection,
                                all_point_offsets,
                                attribute.src,
                                attribute.dst.span);

    attribute.dst.finish();
  }

  bke::copy_attributes_group_to_group(src_attributes,
                                      bke::AttrDomain::Point,
                                      propagation_info,
                                      {},
                                      curve_start_indices,
                                      dst_points_by_curve,
                                      unselected,
                                      dst_attributes);

  return dst_curves;
}

bke::CurvesGeometry fillet_curves_poly(
    const bke::CurvesGeometry &src_curves,
    const IndexMask &curve_selection,
    const VArray<float> &radius,
    const VArray<int> &count,
    const bool limit_radius,
    const bool remove_duplicated_points,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  return fillet_curves(src_curves,
                       curve_selection,
                       radius,
                       count,
                       limit_radius,
                       remove_duplicated_points,
                       false,
                       propagation_info);
}

bke::CurvesGeometry fillet_curves_bezier(
    const bke::CurvesGeometry &src_curves,
    const IndexMask &curve_selection,
    const VArray<float> &radius,
    const bool limit_radius,
    const bool remove_duplicated_points,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  return fillet_curves(src_curves,
                       curve_selection,
                       radius,
                       VArray<int>::ForSingle(1, src_curves.points_num()),
                       limit_radius,
                       remove_duplicated_points,
                       true,
                       propagation_info);
}
}  // namespace blender::geometry
