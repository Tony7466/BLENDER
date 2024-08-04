/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include <algorithm>

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_math_base.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector_types.hh"
#include "BLI_offset_indices.hh"
#include "BLI_vector.hh"

#include "BLI_polygon_clipping_2d.hh"

namespace blender::polygonboolean {

static int intersect(const float2 &P1,
                     const float2 &P2,
                     const float2 &Q1,
                     const float2 &Q2,
                     float *r_alpha_P,
                     float *r_alpha_Q)
{
  double r_lambda;
  double r_mu;
  const int val = isect_seg_seg_v2_lambda_mu_db(
      double2(P1), double2(P2), double2(Q1), double2(Q2), &r_lambda, &r_mu);

  *r_alpha_P = r_lambda;
  *r_alpha_Q = r_mu;

  return val;
}

static bool inside(const float2 &point, Span<float2> poly)
{
  return isect_point_poly_v2(point, reinterpret_cast<const float(*)[2]>(poly.data()), poly.size());
}

struct ExtendedIntersectionPoint {
  int point_a;
  int point_b;
  float alpha_a;
  float alpha_b;
  int sorted_id_a;
  int sorted_id_b;
  bool A_entry_exit;
  bool B_entry_exit;
};

static ExtendedIntersectionPoint CreateIntersection(const int point_a,
                                                    const int point_b,
                                                    const float alpha_a,
                                                    const float alpha_b)
{
  ExtendedIntersectionPoint inter_point;
  inter_point.point_a = point_a;
  inter_point.point_b = point_b;
  inter_point.alpha_a = alpha_a;
  inter_point.alpha_b = alpha_b;

  return inter_point;
}

#define EXIT false
#define ENTRY true

static BooleanResult result_None(Span<float2> /*curve_a*/, Span<float2> /*curve_b*/)
{
  BooleanResult result;

  const Array<Vertex> verts(0);
  const Array<int> offsets = {0};
  const Array<IntersectionPoint> intersections(0);

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = intersections;
  result.valid_geometry = true;

  return result;
}

static BooleanResult result_A(Span<float2> curve_a, Span<float2> /*curve_b*/)
{
  BooleanResult result;

  const int len_a = curve_a.size();

  Array<Vertex> verts(len_a);
  const Array<int> offsets = {0, len_a};
  const Array<IntersectionPoint> intersections(0);

  for (const int i : IndexRange(len_a)) {
    verts[i] = {VertexType::PointA, i};
  }

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = intersections;
  result.valid_geometry = true;

  return result;
}

static BooleanResult result_B(Span<float2> /*curve_a*/, Span<float2> curve_b)
{
  BooleanResult result;

  const int len_b = curve_b.size();

  Array<Vertex> verts(len_b);
  const Array<int> offsets = {0, len_b};
  const Array<IntersectionPoint> intersections(0);

  for (const int i : IndexRange(len_b)) {
    verts[i] = {VertexType::PointB, i};
  }

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = intersections;
  result.valid_geometry = true;

  return result;
}

static BooleanResult result_AB(Span<float2> curve_a, Span<float2> curve_b)
{
  BooleanResult result;

  const int len_a = curve_a.size();
  const int len_b = curve_b.size();

  Array<Vertex> verts(len_a + len_b);
  const Array<int> offsets = {0, len_a, len_a + len_b};
  const Array<IntersectionPoint> intersections(0);

  for (const int i : IndexRange(len_a)) {
    verts[i] = {VertexType::PointA, i};
  }

  for (const int i : IndexRange(len_b)) {
    verts[i + len_a] = {VertexType::PointB, i};
  }

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = intersections;
  result.valid_geometry = true;

  return result;
}

static BooleanResult result_BA(Span<float2> curve_a, Span<float2> curve_b)
{
  BooleanResult result;

  const int len_a = curve_a.size();
  const int len_b = curve_b.size();

  Array<Vertex> verts(len_b + len_a);
  const Array<int> offsets = {0, len_b, len_b + len_a};
  const Array<IntersectionPoint> intersections(0);

  for (const int i : IndexRange(len_b)) {
    verts[i] = {VertexType::PointB, i};
  }

  for (const int i : IndexRange(len_a)) {
    verts[i + len_b] = {VertexType::PointA, i};
  }

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = intersections;
  result.valid_geometry = true;

  return result;
}

static BooleanResult non_intersecting_result(const InputMode input_mode,
                                             Span<float2> curve_a,
                                             Span<float2> curve_b)
{
  const bool is_a_in_b = inside(curve_a.first(), curve_b);
  const bool is_b_in_a = inside(curve_b.first(), curve_a);

  if (is_a_in_b) {
    if (input_mode.boolean_mode == A_AND_B) {
      return result_A(curve_a, curve_b);
    }
    else if (input_mode.boolean_mode == A_NOT_B) {
      return result_None(curve_a, curve_b);
    }
    else if (input_mode.boolean_mode == B_NOT_A) {
      if (input_mode.hole_mode == WITHOUT_HOLES) {
        return result_B(curve_a, curve_b);
      }
      return result_BA(curve_a, curve_b);
    }
    else if (input_mode.boolean_mode == A_OR_B) {
      return result_B(curve_a, curve_b);
    }
  }
  else if (is_b_in_a) {
    if (input_mode.boolean_mode == A_AND_B) {
      return result_B(curve_a, curve_b);
    }
    else if (input_mode.boolean_mode == A_NOT_B) {
      if (input_mode.hole_mode == WITHOUT_HOLES) {
        return result_A(curve_a, curve_b);
      }
      return result_AB(curve_a, curve_b);
    }
    else if (input_mode.boolean_mode == B_NOT_A) {
      return result_None(curve_a, curve_b);
    }
    else if (input_mode.boolean_mode == A_OR_B) {
      return result_A(curve_a, curve_b);
    }
  }
  else if (!is_a_in_b && !is_b_in_a) {
    if (input_mode.boolean_mode == A_AND_B) {
      return result_None(curve_a, curve_b);
    }
    else if (input_mode.boolean_mode == A_NOT_B) {
      return result_A(curve_a, curve_b);
    }
    else if (input_mode.boolean_mode == B_NOT_A) {
      return result_B(curve_a, curve_b);
    }
    else if (input_mode.boolean_mode == A_OR_B) {
      return result_AB(curve_a, curve_b);
    }
  }

  BLI_assert_unreachable();
  return result_None(curve_a, curve_b);
}

static BooleanResult invalided_result(const BooleanMode mode,
                                      Span<float2> curve_a,
                                      Span<float2> curve_b)
{
  BooleanResult result;

  if (mode == A_AND_B) {
    result = result_AB(curve_a, curve_b);
  }
  else if (mode == A_NOT_B) {
    result = result_A(curve_a, curve_b);
  }
  else if (mode == B_NOT_A) {
    result = result_B(curve_a, curve_b);
  }
  else if (mode == A_OR_B) {
    result = result_AB(curve_a, curve_b);
  }

  result.valid_geometry = false;
  return result;
}

static std::pair<bool, bool> get_AB_mode(const BooleanMode mode)
{
  if (mode == A_AND_B) {
    return {false, false};
  }
  else if (mode == A_NOT_B) {
    return {true, false};
  }
  else if (mode == B_NOT_A) {
    return {false, true};
  }
  else if (mode == A_OR_B) {
    return {true, true};
  }

  BLI_assert_unreachable();
  return {false, false};
}

static bool is_point_in_others(const int polygon_a,
                               const OffsetIndices<int> points_by_polygon,
                               const Span<float2> points)
{
  const float2 &point = points[points_by_polygon[polygon_a].first()];
  for (const int polygon_b : points_by_polygon.index_range()) {
    if (polygon_b == polygon_a) {
      continue;
    }
    if (inside(point, points.slice(points_by_polygon[polygon_b]))) {
      return true;
    }
  }
  return false;
}

static int result_find_base_id(const BooleanResult &results,
                               const Span<float2> curve_a,
                               const Span<float2> curve_b)
{
  const OffsetIndices<int> points_by_polygon = OffsetIndices<int>(results.offsets);
  const Array<float2> points = interpolate_data_from_ab_result<float2>(curve_a, curve_b, results);

  /**
   * The base is the polygon that all others are inside of, and therefor it is not in any others.
   */
  for (const int polygon_a : points_by_polygon.index_range()) {
    if (!is_point_in_others(polygon_a, points_by_polygon, points)) {
      return polygon_a;
    }
  }

  BLI_assert_unreachable();
  return -1;
}

static BooleanResult result_remove_holes(const BooleanResult &in_results,
                                         const Span<float2> curve_a,
                                         const Span<float2> curve_b)
{
  BooleanResult result;

  const int base_id = result_find_base_id(in_results, curve_a, curve_b);

  const int base_start = in_results.offsets[base_id];
  const int base_end = in_results.offsets[base_id + 1];
  const int base_size = base_end - base_start;

  Array<Vertex> verts(base_size);
  const Array<int> offsets = {0, base_size};

  const int og_intersection_size = in_results.intersections_data.size();
  Array<int> reverse_map(og_intersection_size, -1);
  int new_intersection_size = 0;

  for (const int i : IndexRange(base_size)) {
    const Vertex &vert = in_results.verts[i + base_start];
    if (vert.type != VertexType::Intersection) {
      verts[i] = vert;
      continue;
    }

    if (reverse_map[vert.point_id] == -1) {
      reverse_map[vert.point_id] = new_intersection_size;
      new_intersection_size++;
    }
    verts[i] = {VertexType::Intersection, reverse_map[vert.point_id]};
  }

  Array<IntersectionPoint> intersections(new_intersection_size);
  for (const int i : IndexRange(og_intersection_size)) {
    if (reverse_map[i] != -1) {
      intersections[reverse_map[i]] = in_results.intersections_data[i];
    }
  }

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = intersections;
  result.valid_geometry = true;

  return result;
}

static BooleanResult result_sort_holes(const BooleanResult &in_results,
                                       const Span<float2> curve_a,
                                       const Span<float2> curve_b)
{
  BooleanResult result;

  const int base_id = result_find_base_id(in_results, curve_a, curve_b);

  const OffsetIndices<int> points_by_polygon = OffsetIndices<int>(in_results.offsets);
  const IndexRange polygons_before_base = IndexRange(base_id);
  const IndexRange polygons_after_base = IndexRange::from_begin_end(base_id + 1,
                                                                    points_by_polygon.size());

  const IndexRange base_points = points_by_polygon[base_id];

  Array<Vertex> verts(in_results.verts.size());
  Array<int> offsets(in_results.offsets.size());

  int i = 0;
  int curr_off = 0;

  offsets[curr_off++] = i;
  for (const int j : base_points.index_range()) {
    verts[i++] = in_results.verts[base_points[j]];
  }

  for (const int polygon_j : polygons_before_base.index_range()) {
    offsets[curr_off++] = i;
    for (const int j : points_by_polygon[polygon_j]) {
      verts[i++] = in_results.verts[j];
    }
  }
  for (const int polygon_j : polygons_after_base.index_range()) {
    offsets[curr_off++] = i;
    for (const int j : points_by_polygon[polygon_j]) {
      verts[i++] = in_results.verts[j];
    }
  }

  offsets[curr_off++] = i;

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = in_results.intersections_data;
  result.valid_geometry = true;

  return result;
}

template<typename T>
void interpolate_data_from_ab_result(const VArray<T> data_a,
                                     const VArray<T> data_b,
                                     const BooleanResult &result,
                                     MutableSpan<T> dst_attr)
{
  for (const int i : result.verts.index_range()) {
    const Vertex &vert = result.verts[i];
    const VertexType &type = vert.type;

    if (type == VertexType::PointA) {
      dst_attr[i] = data_a[vert.point_id];
    }
    else if (type == VertexType::PointB) {
      dst_attr[i] = data_b[vert.point_id];
    }
    else if (type == VertexType::Intersection) {
      const IntersectionPoint &inter_point = result.intersections_data[vert.point_id];

      const T a0 = data_a[inter_point.point_a];
      const T a1 = data_a[(inter_point.point_a + 1) % data_a.size()];
      const float alpha_a = inter_point.alpha_a;

      const T b0 = data_b[inter_point.point_b];
      const T b1 = data_b[(inter_point.point_b + 1) % data_b.size()];
      const float alpha_b = inter_point.alpha_b;

      dst_attr[i] = math::interpolate(
          math::interpolate(a0, a1, alpha_a), math::interpolate(b0, b1, alpha_b), 0.5f);
    }
  }
}

template<typename T>
Array<T> interpolate_data_from_ab_result(const Span<T> data_a,
                                         const Span<T> data_b,
                                         const BooleanResult &result)
{
  Array<T> data_out(result.verts.size());

  interpolate_data_from_ab_result(
      VArray<T>::ForSpan(data_a), VArray<T>::ForSpan(data_b), result, data_out.as_mutable_span());

  return data_out;
}

/**
 * Utility class to avoid passing large number of parameters between functions.
 */
struct CurveBooleanExecutor {
  int len_a;
  int len_b;

  Vector<ExtendedIntersectionPoint> intersections;
  int num_intersects;

  Array<int> A_inter_sorted_ids;
  Array<int> B_inter_sorted_ids;

  Vector<Vertex> verts;
  Vector<int> vertex_offsets;

  void newPolygon()
  {
    vertex_offsets.append(verts.size());
  }

  void newVertexID(int point_id, bool is_curve_A)
  {
    verts.append({is_curve_A ? VertexType::PointA : VertexType::PointB, point_id});
  }

  void newVertexIntersection(int point_id)
  {
    verts.append({VertexType::Intersection, point_id});
  }

  int get_next_intersection_id(int id0, bool is_curve_A)
  {
    const ExtendedIntersectionPoint &vertex0 = intersections[id0];

    const int int_sorted = is_curve_A ? vertex0.sorted_id_a : vertex0.sorted_id_b;
    const bool direction = is_curve_A ? vertex0.A_entry_exit : vertex0.B_entry_exit;

    const int next_sorted_int = (int_sorted + (direction == ENTRY ? 1 : num_intersects - 1)) %
                                num_intersects;

    return is_curve_A ? A_inter_sorted_ids[next_sorted_int] : B_inter_sorted_ids[next_sorted_int];
  }

  void Add_Between_Points(int id0, bool is_curve_A)
  {
    const int id1 = get_next_intersection_id(id0, is_curve_A);

    const ExtendedIntersectionPoint &vertex0 = intersections[id0];
    const ExtendedIntersectionPoint &vertex1 = intersections[id1];

    const bool direction = is_curve_A ? vertex0.A_entry_exit : vertex0.B_entry_exit;

    const int i0 = is_curve_A ? vertex0.point_a : vertex0.point_b;
    const int i1 = is_curve_A ? vertex1.point_a : vertex1.point_b;

    const int curve_len = is_curve_A ? len_a : len_b;

    /* If both intersection points are on the same segment, there's ether no points between or all
     * of the points. */
    if (i0 == i1) {
      const float a0 = is_curve_A ? vertex0.alpha_a : vertex0.alpha_b;
      const float a1 = is_curve_A ? vertex1.alpha_a : vertex1.alpha_b;

      if (direction == ENTRY && a0 < a1) {
        return;
      }
      if (direction == EXIT && a0 > a1) {
        return;
      }
    }

    /* Both start and end are included. */
    int start = i0;
    int end = i1;

    /* Add one to not include the point past the segment. */
    if (direction == ENTRY) {
      start = i0 + 1;
    }
    else { /* direction == EXIT */
      end = i1 + 1;
    }

    /* When a full loop is need. */
    if (i0 >= i1 && direction == ENTRY) {
      end = i1 + curve_len;
    }
    if (i0 <= i1 && direction == EXIT) {
      start = i0 + curve_len;
    }

    if (direction == ENTRY) {
      for (int i = start; i <= end; i++) {
        newVertexID(i % curve_len, is_curve_A);
      }
    }
    else { /* direction == EXIT */
      for (int i = start; i >= end; i--) {
        newVertexID(i % curve_len, is_curve_A);
      }
    }
  }

  BooleanResult copy_data_to_result()
  {
    Array<Vertex> verts_out(verts.size());
    array_utils::copy(verts.as_span(), verts_out.as_mutable_span());

    Array<int> offsets(vertex_offsets.size());
    array_utils::copy(vertex_offsets.as_span(), offsets.as_mutable_span());

    Array<IntersectionPoint> intersections_data(intersections.size());
    for (const int i : IndexRange(intersections.size())) {
      const ExtendedIntersectionPoint &inter = intersections[i];
      intersections_data[i] = {inter.point_a, inter.point_b, inter.alpha_a, inter.alpha_b};
    }

    BooleanResult result;

    result.verts = verts_out;
    result.offsets = offsets;
    result.intersections_data = intersections_data;
    result.valid_geometry = true;

    return result;
  }

  void sort_a_intersections()
  {
    A_inter_sorted_ids = Array<int>(num_intersects);
    array_utils::fill_index_range<int>(A_inter_sorted_ids);

    std::sort(A_inter_sorted_ids.begin(), A_inter_sorted_ids.end(), [&](int i1, int i2) {
      return intersections[i1].point_a + intersections[i1].alpha_a <
             intersections[i2].point_a + intersections[i2].alpha_a;
    });

    for (const int id : IndexRange(num_intersects)) {
      intersections[A_inter_sorted_ids[id]].sorted_id_a = id;
    }
  }

  void sort_b_intersections()
  {
    B_inter_sorted_ids = Array<int>(num_intersects);
    array_utils::fill_index_range<int>(B_inter_sorted_ids);

    std::sort(B_inter_sorted_ids.begin(), B_inter_sorted_ids.end(), [&](int i1, int i2) {
      return intersections[i1].point_b + intersections[i1].alpha_b <
             intersections[i2].point_b + intersections[i2].alpha_b;
    });

    for (const int id : IndexRange(num_intersects)) {
      intersections[B_inter_sorted_ids[id]].sorted_id_b = id;
    }
  }

  void set_a_directions(Span<float2> curve_a, Span<float2> curve_b, bool A_mode)
  {
    const bool is_a_in_b = inside(curve_a.first(), curve_b);

    bool A_status = is_a_in_b ^ A_mode ? EXIT : ENTRY;

    for (const int j : IndexRange(num_intersects)) {
      intersections[A_inter_sorted_ids[j]].A_entry_exit = A_status;

      A_status = !A_status;
    }
  }

  void set_b_directions(Span<float2> curve_a, Span<float2> curve_b, bool B_mode)
  {
    const bool is_b_in_a = inside(curve_b.first(), curve_a);

    bool B_status = is_b_in_a ^ B_mode ? EXIT : ENTRY;

    for (const int j : IndexRange(num_intersects)) {
      intersections[B_inter_sorted_ids[j]].B_entry_exit = B_status;

      B_status = !B_status;
    }
  }

  BooleanResult execute_boolean(const InputMode input_mode,
                                Span<float2> curve_a,
                                Span<float2> curve_b)
  {
    len_a = curve_a.size();
    len_b = curve_b.size();

    /* ---- ---- ---- Phase One ---- ---- ---- */

    for (const int i : IndexRange(len_a)) {
      for (const int j : IndexRange(len_b)) {
        float alpha_a, alpha_b;
        const int val = intersect(curve_a[i],
                                  curve_a[(i + 1) % len_a],
                                  curve_b[j],
                                  curve_b[(j + 1) % len_b],
                                  &alpha_a,
                                  &alpha_b);
        if (val == ISECT_LINE_LINE_CROSS) {
          intersections.append(CreateIntersection(i, j, alpha_a, alpha_b));
        }
        else if (val == ISECT_LINE_LINE_EXACT) {
          return invalided_result(input_mode.boolean_mode, curve_a, curve_b);
        }
      }
    }

    if (intersections.is_empty()) {
      return non_intersecting_result(input_mode, curve_a, curve_b);
    }

    num_intersects = intersections.size();

    sort_a_intersections();
    sort_b_intersections();

    /* ---- ---- ---- Phase Two ---- ---- ---- */

    const auto [A_mode, B_mode] = get_AB_mode(input_mode.boolean_mode);

    set_a_directions(curve_a, curve_b, A_mode);
    set_b_directions(curve_a, curve_b, B_mode);

    /* ---- ---- ---- Phase Three ---- ---- ---- */

    Array<bool> unprocessed_intersection_unsorted_ids(num_intersects, true);
    int inter_id = 0;

    while (inter_id != -1) {
      bool is_curve_A = true;

      /* Both of these are in unsorted space. */
      int curr_int_id = inter_id;
      const int start_int_id = curr_int_id;

      newPolygon();

      bool PolygonClosed = false;
      while (!PolygonClosed) {
        Add_Between_Points(curr_int_id, is_curve_A);

        curr_int_id = get_next_intersection_id(curr_int_id, is_curve_A);
        unprocessed_intersection_unsorted_ids[curr_int_id] = false;

        /* Switch to the other curve. */
        is_curve_A = !is_curve_A;

        newVertexIntersection(curr_int_id);

        if (is_curve_A && curr_int_id == start_int_id) {
          PolygonClosed = true;
        }
      }

      /* Get the next unprocessed intersection point (in unsorted space). */
      inter_id = unprocessed_intersection_unsorted_ids.as_span().first_index_try(true);
    }

    /* Add one for the end. */
    newPolygon();

    BooleanResult result = copy_data_to_result();

    /**
     *  Holes are only create in the union of the shapes
     * (Because non-intersecting holes are already handled)
     */
    if (input_mode.boolean_mode == A_OR_B) {
      if (input_mode.hole_mode == WITHOUT_HOLES) {
        result = result_remove_holes(result, curve_a, curve_b);
      }
      else if (input_mode.hole_mode == WITH_ORDERED_HOLES) {
        result = result_sort_holes(result, curve_a, curve_b);
      }
    }

    return result;
  }

  bool cut_add_start_cap()
  {
    const int inter_id = A_inter_sorted_ids.first();
    const ExtendedIntersectionPoint &vertex_first = intersections[inter_id];

    if (vertex_first.A_entry_exit == ENTRY) {
      return false;
    }

    /* Start line. */
    newPolygon();

    for (const int i : IndexRange::from_begin_end_inclusive(0, vertex_first.point_a)) {
      newVertexID(i, true);
    }

    newVertexIntersection(inter_id);
    return true;
  }

  bool cut_add_end_cap()
  {
    const int inter_id = A_inter_sorted_ids.last();
    const ExtendedIntersectionPoint &vertex_last = intersections[inter_id];

    if (vertex_last.A_entry_exit == EXIT) {
      return false;
    }

    /* Start line. */
    newPolygon();

    newVertexIntersection(inter_id);

    for (const int i : IndexRange::from_begin_end(vertex_last.point_a + 1, len_a)) {
      newVertexID(i, true);
    }
    return true;
  }

  void cut_add_between_points(const int int_sorted)
  {
    const int i0 = A_inter_sorted_ids[int_sorted];
    const int i1 = A_inter_sorted_ids[int_sorted + 1];

    const ExtendedIntersectionPoint &vertex0 = intersections[i0];
    const ExtendedIntersectionPoint &vertex1 = intersections[i1];

    BLI_assert(vertex0.A_entry_exit == ENTRY);

    /* Start line. */
    newPolygon();

    newVertexIntersection(i0);

    if (vertex0.point_a != vertex1.point_a) {
      for (const int i :
           IndexRange::from_begin_end_inclusive(vertex0.point_a + 1, vertex1.point_a))
      {
        newVertexID(i, true);
      }
    }

    newVertexIntersection(i1);
  }

  bool cut_add_looping_cap()
  {
    const int inter_id_first = A_inter_sorted_ids.first();
    const int inter_id_last = A_inter_sorted_ids.last();
    const ExtendedIntersectionPoint &vertex_first = intersections[inter_id_first];
    const ExtendedIntersectionPoint &vertex_last = intersections[inter_id_last];

    if (vertex_first.A_entry_exit == ENTRY || vertex_last.A_entry_exit == EXIT) {
      return false;
    }

    /* Start line. */
    newPolygon();

    /* Start at the last intersection point. */
    newVertexIntersection(inter_id_last);

    /* Go through all point from last intersection point to the end. */
    for (const int i : IndexRange::from_begin_end(vertex_last.point_a + 1, len_a)) {
      newVertexID(i, true);
    }

    /* Go through all point from start point to the first intersection point. */
    for (const int i : IndexRange::from_begin_end_inclusive(0, vertex_first.point_a)) {
      newVertexID(i, true);
    }

    /* End at the first intersection point. */
    newVertexIntersection(inter_id_first);
    return true;
  }

  /* Curve `A` does not have fill. */
  BooleanResult execute_cut(const bool is_a_cyclic, Span<float2> curve_a, Span<float2> curve_b)
  {
    len_a = curve_a.size();
    len_b = curve_b.size();

    /* ---- ---- ---- Phase One ---- ---- ---- */

    const IndexRange a_range = is_a_cyclic ? IndexRange(len_a) : IndexRange(len_a - 1);

    for (const int i : a_range) {
      for (const int j : IndexRange(len_b)) {
        float alpha_a, alpha_b;
        const int val = intersect(curve_a[i],
                                  curve_a[(i + 1) % len_a],
                                  curve_b[j],
                                  curve_b[(j + 1) % len_b],
                                  &alpha_a,
                                  &alpha_b);
        if (val == ISECT_LINE_LINE_CROSS) {
          intersections.append(CreateIntersection(i, j, alpha_a, alpha_b));
        }
        else if (val == ISECT_LINE_LINE_EXACT) {
          return result_A(curve_a, curve_b);
        }
      }
    }

    if (intersections.is_empty()) {
      const bool is_a_in_b = inside(curve_a.first(), curve_b);
      if (is_a_in_b) {
        return result_None(curve_a, curve_b);
      }
      else {
        return result_A(curve_a, curve_b);
      }
    }

    num_intersects = intersections.size();

    sort_a_intersections();

    /* ---- ---- ---- Phase Two ---- ---- ---- */

    set_a_directions(curve_a, curve_b, true);

    /* ---- ---- ---- Phase Three ---- ---- ---- */

    bool is_looping = false;
    if (is_a_cyclic) {
      is_looping = cut_add_looping_cap();
    }

    bool is_start = is_looping;
    if (!is_looping) {
      is_start = cut_add_start_cap();
    }

    for (int i = is_start ? 1 : 0; i < num_intersects - 1; i += 2) {
      cut_add_between_points(i);
    }
    if (!is_looping) {
      cut_add_end_cap();
    }

    /* Add one for the end. */
    newPolygon();

    return copy_data_to_result();
  }
};

BooleanResult curve_boolean_calc(const InputMode input_mode,
                                 Span<float2> curve_a,
                                 Span<float2> curve_b)
{
  CurveBooleanExecutor executor;
  return executor.execute_boolean(input_mode, curve_a, curve_b);
}

BooleanResult curve_boolean_cut(const bool is_a_cyclic, Span<float2> curve_a, Span<float2> curve_b)
{
  CurveBooleanExecutor executor;
  return executor.execute_cut(is_a_cyclic, curve_a, curve_b);
}

}  // namespace blender::polygonboolean
