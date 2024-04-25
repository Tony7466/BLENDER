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
                                     MutableSpan<int> dst_point_offsets,
                                     Span<bool> ends_with_zero_edge)
{
  /* Fill the offsets array with the curve point counts, then accumulate them to form offsets. */
  offset_indices::copy_group_sizes(curve_start_indices, unselected, dst_curve_offsets);
  selection.foreach_index(GrainSize(512), [&](const int curve_i) {
    const IndexRange src_points = curve_start_indices[curve_i];
    const IndexRange offsets_range = bke::curves::per_curve_point_offsets_range(src_points,
                                                                                curve_i);

    MutableSpan<int> point_offsets = dst_point_offsets.slice(offsets_range);
    MutableSpan<int> point_counts = point_offsets.drop_back(1);
    counts.materialize_compressed(src_points, point_counts);

    Span<bool> zero_edge_buffer = ends_with_zero_edge.slice(src_points);
    for (const int i : src_points.index_range()) {
      int &count = point_counts[i];
      count = std::max(count, 0) + 1;

      bool has_zero_edge = zero_edge_buffer[i];
      if (has_zero_edge) {
        count -= 1;
      }
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
                        MutableSpan<bool> has_zero_length_edge)
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
        has_zero_length_edge[i] = false;
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
        has_zero_length_edge[i] = false;
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

    has_zero_length_edge[i] = min_factor < 1 && factor_next <= factor_prev;
  }
}

static void calculate_fillet_positions(const Span<float3> src_positions,
                                       const Span<float> angles,
                                       const Span<float> radii,
                                       const Span<float3> directions,
                                       const OffsetIndices<int> dst_offsets,
                                       Span<bool> zero_edge_buffer,
                                       MutableSpan<float3> dst)
{
  const int i_src_last = src_positions.index_range().last();
  threading::parallel_for(src_positions.index_range(), 512, [&](IndexRange range) {
    for (const int i_src : range) {
      const IndexRange arc = dst_offsets[i_src];
      const float3 &src = src_positions[i_src];
      bool is_zero_edge = zero_edge_buffer[i_src];
      if (arc.size() == 1 && !is_zero_edge) {
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
      IndexRange middle = arc.drop_front(1);
      /* If this is a zero-length edge, the endpoint of the arc is the startpoint of the following
       * point. Therefore last point is actually still considered the "middle" of the arc
       */
      if (!is_zero_edge) {
        dst[arc.last()] = arc_end;
        middle = middle.drop_back(1);
      }

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
                                                 const Span<bool> zero_edge_buffer,
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
      bool is_zero_edge = zero_edge_buffer[i_src];
      const int i_dst_a = arc.first();

      if (arc.size() == 1 && !is_zero_edge) {
        dst_handles_l[i_dst_a] = src_handles_l[i_src];
        dst_handles_r[i_dst_a] = src_handles_r[i_src];
        dst_types_l[i_dst_a] = src_types_l[i_src];
        dst_types_r[i_dst_a] = src_types_r[i_src];
        continue;
      }

      /* Calculate the point's handles on the outside of the fillet segment,
       * connecting to the next or previous result points.
       *
       * The inner handles are aligned with the aligned with the outer vector
       * handles, but have a specific length to best approximate a circle.
       *
       * When two fillets meet and zero-length edges are removed
       */

      const int i_dst_b = arc.last() + (is_zero_edge ? 1 : 0);

      /* The first point can only be filleted if the curve is cyclic */
      bool is_previous_zero_edge = i_src == 0 ? zero_edge_buffer[i_src_last] :
                                                zero_edge_buffer[i_src - 1];
      const int i_src_prev = i_src == 0 ? i_src_last : i_src - 1;
      const float angle = angles[i_src];
      const float radius = radii[i_src];
      const float handle_length = (4.0f / 3.0f) * radius * std::tan(angle / 4.0f);
      const float3 prev_dir = -directions[i_src_prev];
      const float3 &next_dir = directions[i_src];

      const float3 &arc_start = dst_positions[i_dst_a];
      const float3 &arc_end = dst_positions[i_dst_b];

      if (!is_previous_zero_edge) {
        const int i_dst_prev = i_dst_a == 0 ? i_dst_last : i_dst_a - 1;

        dst_handles_l[i_dst_a] = bke::curves::bezier::calculate_vector_handle(
            dst_positions[i_dst_a], dst_positions[i_dst_prev]);
        dst_types_l[i_dst_a] = BEZIER_HANDLE_VECTOR;
      }

      dst_handles_r[i_dst_a] = arc_start - prev_dir * handle_length;
      dst_types_r[i_dst_a] = BEZIER_HANDLE_ALIGN;

      const int i_dst_next = i_dst_b == i_dst_last ? 0 : i_dst_b + 1;

      dst_handles_r[i_dst_b] = bke::curves::bezier::calculate_vector_handle(
          dst_positions[i_dst_b], dst_positions[i_dst_next]);
      dst_types_r[i_dst_b] = BEZIER_HANDLE_VECTOR;

      dst_handles_l[i_dst_b] = arc_end - next_dir * handle_length;
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
    printf("[%d] = %f ", i, span[i]);
  }
  printf("\n");
}

/** Print contents of span
 */
static void print_span3f(const Span<float3> span)
{
  for (const int i : span.index_range()) {
    printf("[%d] = (%f,%f,%f)\n", i, span[i].x, span[i].y, span[i].z);
  }
  printf("\n");
}

/** Print contents of span
 */
static void print_spani(const Span<int> span)
{
  for (const int i : span.index_range()) {
    printf("[%d] = %d ", i, span[i]);
  }
  printf("\n");
}

/** Print contents of span
 */
static void print_spanb(const Span<bool> span)
{
  for (const int i : span.index_range()) {
    printf("[%d] = %d ", i, span[i]);
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

  Array<float3> directions;
  Array<float> angles;
  Array<float> radii;
  Array<bool> ends_with_zero_edge;
  const int src_points_num = src_curves.points_num();
  bool has_zero_edge_removal = limit_radius && remove_duplicated_points;
  /*
   * Pre-calculate radii for all curves if zero-length edges will be removed.
   * Allocating these arrays on a curve-by-curve basis would conserve memory
   * and is possible if zero-length edges are not removed
   *
   * Currently, memory is allocated for all curves, even those that are not selected.
   */
  directions = Array<float3>(src_points_num);
  angles = Array<float>(src_points_num);
  radii = Array<float>(src_points_num);
  ends_with_zero_edge = Array<bool>(src_points_num);

  curve_selection.foreach_index(GrainSize(512), [&](const int curve_i) {
    const IndexRange src_points = curve_start_indices[curve_i];
    const Span<float3> positions_buffer = positions.slice(src_points);

    MutableSpan<float3> directions_buffer = directions.as_mutable_span().slice(src_points);
    calculate_directions(positions_buffer, directions_buffer);

    MutableSpan<float> angles_buffer = angles.as_mutable_span().slice(src_points);
    calculate_angles(directions_buffer, angles_buffer);

    MutableSpan<float> radii_buffer = radii.as_mutable_span().slice(src_points);
    MutableSpan<bool> zero_edge_buffer = ends_with_zero_edge.as_mutable_span().slice(src_points);

    if (limit_radius) {
      Array<float> input_radii_buffer = Array<float>(src_points.size());
      radius_input.materialize_compressed(src_points, input_radii_buffer);

      limit_radii(positions_buffer,
                  angles_buffer,
                  input_radii_buffer,
                  cyclic[curve_i],
                  radii_buffer,
                  zero_edge_buffer);

      printf("Radii: \n");
      print_spanf(radii_buffer);

      printf("Zero edge: \n");
      print_spanb(zero_edge_buffer);

      if (!remove_duplicated_points) {
        zero_edge_buffer.fill(false);
      }
    }
    else {
      radius_input.materialize_compressed(src_points, radii);
      zero_edge_buffer.fill(false);
    }
  });

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
                           dst_point_offsets,
                           ends_with_zero_edge);

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

  curve_selection.foreach_segment(GrainSize(512), [&](const IndexMaskSegment segment) {
    for (const int curve_i : segment) {
      const IndexRange src_points = curve_start_indices[curve_i];
      const IndexRange offsets_range = bke::curves::per_curve_point_offsets_range(src_points,
                                                                                  curve_i);
      const OffsetIndices<int> offsets(all_point_offsets.slice(offsets_range));
      const IndexRange dst_points = dst_points_by_curve[curve_i];
      const Span<float3> src_positions = positions.slice(src_points);

      Span<float3> directions_buffer = directions.as_span().slice(src_points);
      Span<float> angles_buffer = angles.as_span().slice(src_points);
      Span<float> radii_buffer = radii.as_span().slice(src_points);
      Span<bool> zero_edge_buffer = ends_with_zero_edge.as_span().slice(src_points);

      calculate_fillet_positions(positions.slice(src_points),
                                 angles_buffer,
                                 radii_buffer,
                                 directions_buffer,
                                 offsets,
                                 zero_edge_buffer,
                                 dst_positions.slice(dst_points));


      if (src_curves.has_curve_with_type(CURVE_TYPE_BEZIER)) {
        MutableSpan<float3> dst_positions_write = dst_positions.slice(dst_points);
        MutableSpan<float3> dst_handles_l_write = dst_handles_l.slice(dst_points);
        MutableSpan<float3> dst_handles_r_write = dst_handles_r.slice(dst_points);
        MutableSpan<int8_t> dst_types_l_write = dst_types_l.slice(dst_points);
        MutableSpan<int8_t> dst_types_r_write = dst_types_r.slice(dst_points);
        if (use_bezier_mode) {
          calculate_bezier_handles_bezier_mode(src_handles_l.slice(src_points),
                                               src_handles_r.slice(src_points),
                                               src_types_l.slice(src_points),
                                               src_types_r.slice(src_points),
                                               angles_buffer,
                                               radii_buffer,
                                               directions_buffer,
                                               zero_edge_buffer,
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
    }
  });

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
