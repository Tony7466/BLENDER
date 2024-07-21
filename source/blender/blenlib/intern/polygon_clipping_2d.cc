/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include <algorithm>

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include "BLI_polygon_clipping_2d.hh"

namespace blender::polygonboolean {

int intersect(const float2 &P1,
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

bool inside(const float2 &point, Span<float2> poly)
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

ExtendedIntersectionPoint CreateIntersection(const int point_a,
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

BooleanResult result_None(Span<float2> /*curve_a*/, Span<float2> /*curve_b*/)
{
  BooleanResult result;

  const Array<Vertex> verts(0);
  const Array<int> offsets(1, 0);
  const Array<IntersectionPoint> intersections(0);

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = intersections;
  result.valid_geometry = true;

  return result;
}

BooleanResult result_A(Span<float2> curve_a, Span<float2> /*curve_b*/)
{
  BooleanResult result;

  const int len_a = curve_a.size();

  Array<Vertex> verts(len_a);
  Array<int> offsets(2);
  const Array<IntersectionPoint> intersections(0);

  for (const int i : IndexRange(len_a)) {
    verts[i] = {VertexType::PointA, i};
  }

  offsets[0] = 0;
  offsets[1] = len_a;

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = intersections;
  result.valid_geometry = true;

  return result;
}

BooleanResult result_B(Span<float2> /*curve_a*/, Span<float2> curve_b)
{
  BooleanResult result;

  const int len_b = curve_b.size();

  Array<Vertex> verts(len_b);
  Array<int> offsets(2);
  const Array<IntersectionPoint> intersections(0);

  for (const int i : IndexRange(len_b)) {
    verts[i] = {VertexType::PointB, i};
  }

  offsets[0] = 0;
  offsets[1] = len_b;

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = intersections;
  result.valid_geometry = true;

  return result;
}

BooleanResult result_AB(Span<float2> curve_a, Span<float2> curve_b)
{
  BooleanResult result;

  const int len_a = curve_a.size();
  const int len_b = curve_b.size();

  Array<Vertex> verts(len_a + len_b);
  Array<int> offsets(3);
  const Array<IntersectionPoint> intersections(0);

  for (const int i : IndexRange(len_a)) {
    verts[i] = {VertexType::PointA, i};
  }

  for (const int i : IndexRange(len_b)) {
    verts[i + len_a] = {VertexType::PointB, i};
  }

  offsets[0] = 0;
  offsets[1] = len_a;
  offsets[2] = len_a + len_b;

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = intersections;
  result.valid_geometry = true;

  return result;
}

BooleanResult result_BA(Span<float2> curve_a, Span<float2> curve_b)
{
  BooleanResult result;

  const int len_a = curve_a.size();
  const int len_b = curve_b.size();

  Array<Vertex> verts(len_b + len_a);
  Array<int> offsets(3);
  const Array<IntersectionPoint> intersections(0);

  for (const int i : IndexRange(len_b)) {
    verts[i] = {VertexType::PointB, i};
  }

  for (const int i : IndexRange(len_a)) {
    verts[i + len_b] = {VertexType::PointA, i};
  }

  offsets[0] = 0;
  offsets[1] = len_b;
  offsets[2] = len_b + len_a;

  result.verts = verts;
  result.offsets = offsets;
  result.intersections_data = intersections;
  result.valid_geometry = true;

  return result;
}

BooleanResult non_intersecting_result(const InputMode input_mode,
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

BooleanResult invalided_result(const BooleanMode mode, Span<float2> curve_a, Span<float2> curve_b)
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

std::pair<bool, bool> get_AB_mode(const BooleanMode mode)
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

    int start = i0;
    int end = i1;

    if (direction == ENTRY) {
      start = i0 + 1;
    }

    if (i0 > i1 && direction == ENTRY) {
      end = i1 + curve_len;
    }
    if (i0 < i1 && direction == EXIT) {
      start = i0 + curve_len;
    }

    if (i0 == i1 && direction == ENTRY) {
      start = i0 + 1;
      end = i1 + curve_len;
    }
    if (i0 == i1 && direction == EXIT) {
      start = i0 + curve_len;
      end = i1;
    }

    if (direction == ENTRY) {
      for (int i = start; i <= end; i++) {
        newVertexID(i % curve_len, is_curve_A);
      }
    }
    else { /* direction == EXIT */
      for (int i = start; i > end; i--) {
        newVertexID(i % curve_len, is_curve_A);
      }
    }
  }

  BooleanResult execute(const InputMode input_mode, Span<float2> curve_a, Span<float2> curve_b)
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

    num_intersects = intersections.size();

    /* ---- ---- ---- Phase Two ---- ---- ---- */

    if (intersections.is_empty()) {
      return non_intersecting_result(input_mode, curve_a, curve_b);
    }

    A_inter_sorted_ids = Array<int>(num_intersects);
    B_inter_sorted_ids = Array<int>(num_intersects);

    array_utils::fill_index_range<int>(A_inter_sorted_ids);
    array_utils::fill_index_range<int>(B_inter_sorted_ids);

    std::sort(A_inter_sorted_ids.begin(), A_inter_sorted_ids.end(), [&](int i1, int i2) {
      return intersections[i1].point_a + intersections[i1].alpha_a <
             intersections[i2].point_a + intersections[i2].alpha_a;
    });
    std::sort(B_inter_sorted_ids.begin(), B_inter_sorted_ids.end(), [&](int i1, int i2) {
      return intersections[i1].point_b + intersections[i1].alpha_b <
             intersections[i2].point_b + intersections[i2].alpha_b;
    });

    for (const int id : IndexRange(num_intersects)) {
      intersections[A_inter_sorted_ids[id]].sorted_id_a = id;
      intersections[B_inter_sorted_ids[id]].sorted_id_b = id;
    }

    /* ---- ---- ---- Phase Three ---- ---- ---- */

    const auto [A_mode, B_mode] = get_AB_mode(input_mode.boolean_mode);

    const bool is_a_in_b = inside(curve_a.first(), curve_b);
    const bool is_b_in_a = inside(curve_b.first(), curve_a);

    bool A_status = is_a_in_b ^ A_mode ? EXIT : ENTRY;
    bool B_status = is_b_in_a ^ B_mode ? EXIT : ENTRY;

    for (const int j : IndexRange(num_intersects)) {
      intersections[A_inter_sorted_ids[j]].A_entry_exit = A_status;
      intersections[B_inter_sorted_ids[j]].B_entry_exit = B_status;

      A_status = !A_status;
      B_status = !B_status;
    }

    /* ---- ---- ---- Phase Four ---- ---- ---- */

    Array<bool> unprocessed_intersection_unsorted_ids(num_intersects, true);
    int inter_id = 0;

    while (inter_id != -1) {
      bool is_curve_A = true;

      // in unsorted space
      int curr_int_id = inter_id;
      const int start_int_id = curr_int_id;

      newPolygon();

      bool PolygonClosed = false;
      while (!PolygonClosed) {
        Add_Between_Points(curr_int_id, is_curve_A);

        curr_int_id = get_next_intersection_id(curr_int_id, is_curve_A);
        unprocessed_intersection_unsorted_ids[curr_int_id] = false;

        // Switching to the other curve
        is_curve_A = !is_curve_A;

        newVertexIntersection(curr_int_id);

        if (is_curve_A && curr_int_id == start_int_id) {
          PolygonClosed = true;
        }
      }

      // get the next unprocessed intersection point (in unsorted space)
      inter_id = unprocessed_intersection_unsorted_ids.as_span().first_index_try(true);
    }

    /* Add one for the end. */
    newPolygon();

    /* ---- ---- ---- Phase Five ---- ---- ---- */

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

  /* Curve `A` does not loop. */
  BooleanResult curve_boolean_cut(Span<float2> curve_a, Span<float2> curve_b)
  {
    len_a = curve_a.size();
    len_b = curve_b.size();

    /* ---- ---- ---- Phase One ---- ---- ---- */

    for (const int i : IndexRange(len_a - 1)) { /* `A` does not loop. */
      for (const int j : IndexRange(len_b)) {
        float alpha_a, alpha_b;
        const int val = intersect(
            curve_a[i], curve_a[i + 1], curve_b[j], curve_b[(j + 1) % len_b], &alpha_a, &alpha_b);
        if (val == ISECT_LINE_LINE_CROSS) {
          intersections.append(CreateIntersection(i, j, alpha_a, alpha_b));
        }
        else if (val == ISECT_LINE_LINE_EXACT) {
          return result_A(curve_a, curve_b);
        }
      }
    }

    num_intersects = intersections.size();

    /* ---- ---- ---- Phase Two ---- ---- ---- */

    if (intersections.is_empty()) {
      const bool is_a_in_b = inside(curve_a.first(), curve_b);
      if (is_a_in_b) {
        return result_None(curve_a, curve_b);
      }
      else {
        return result_A(curve_a, curve_b);
      }
    }

    A_inter_sorted_ids = Array<int>(num_intersects);
    array_utils::fill_index_range<int>(A_inter_sorted_ids);
    std::sort(A_inter_sorted_ids.begin(), A_inter_sorted_ids.end(), [&](int i1, int i2) {
      return intersections[i1].point_a + intersections[i1].alpha_a <
             intersections[i2].point_a + intersections[i2].alpha_a;
    });

    for (const int id : IndexRange(num_intersects)) {
      intersections[A_inter_sorted_ids[id]].sorted_id_a = id;
    }

    /* ---- ---- ---- Phase Three ---- ---- ---- */

    const bool is_a_in_b = inside(curve_a.first(), curve_b);

    bool A_status = is_a_in_b ? ENTRY : EXIT;

    for (const int j : IndexRange(num_intersects)) {
      intersections[A_inter_sorted_ids[j]].A_entry_exit = A_status;

      A_status = !A_status;
    }

    /* ---- ---- ---- Phase Four ---- ---- ---- */

    int int_sorted = 0;

    const ExtendedIntersectionPoint &vertex_first = intersections[A_inter_sorted_ids[int_sorted]];

    if (vertex_first.A_entry_exit == EXIT) {
      /* Start line. */
      newPolygon();

      for (const int i : IndexRange::from_begin_end_inclusive(0, vertex_first.point_a)) {
        newVertexID(i, true);
      }

      newVertexIntersection(A_inter_sorted_ids[int_sorted]);

      int_sorted += 1;
    }

    if (num_intersects == 2 && int_sorted == 0) {
      const int i0 = A_inter_sorted_ids[int_sorted];
      const int i1 = A_inter_sorted_ids[int_sorted + 1];

      const ExtendedIntersectionPoint &vertex0 = intersections[i0];
      const ExtendedIntersectionPoint &vertex1 = intersections[i1];

      int_sorted += 2;

      if (vertex0.A_entry_exit == ENTRY) {
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
    }

    for (const int j : IndexRange(int(num_intersects / 2))) {
      if (num_intersects > (j + 1) * 2) {
        const int i0 = A_inter_sorted_ids[int_sorted];
        const int i1 = A_inter_sorted_ids[int_sorted + 1];

        const ExtendedIntersectionPoint &vertex0 = intersections[i0];
        const ExtendedIntersectionPoint &vertex1 = intersections[i1];

        int_sorted += 2;

        if (vertex0.A_entry_exit == ENTRY) {
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
      }
    }

    int_sorted = num_intersects - 1;

    const ExtendedIntersectionPoint &vertex_last = intersections[A_inter_sorted_ids[int_sorted]];

    if (vertex_last.A_entry_exit == ENTRY) {
      /* Start line. */
      newPolygon();

      newVertexIntersection(A_inter_sorted_ids[int_sorted]);

      for (const int i : IndexRange::from_begin_end(vertex_last.point_a + 1, len_a)) {
        newVertexID(i, true);
      }
    }

    /* Add one for the end. */
    newPolygon();

    /* ---- ---- ---- Phase Five ---- ---- ---- */

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
};

BooleanResult curve_boolean_calc(const InputMode input_mode,
                                 Span<float2> curve_a,
                                 Span<float2> curve_b)
{
  CurveBooleanExecutor executor;
  return executor.execute(input_mode, curve_a, curve_b);
}

BooleanResult curve_boolean_cut(Span<float2> curve_a, Span<float2> curve_b)
{
  CurveBooleanExecutor executor;
  return executor.curve_boolean_cut(curve_a, curve_b);
}

}  // namespace blender::polygonboolean
