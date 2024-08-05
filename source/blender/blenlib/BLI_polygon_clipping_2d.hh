/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_array.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"
#include "BLI_virtual_array.hh"

/** \file
 * \ingroup bli
 *
 * This header file contains a C++ interface to the 2D Greiner-Hormann clipping algorithm.
 */

/**
 * Interface for Polygon Clipping in 2D use the Greiner-Hormann clipping algorithm.
 *
 * The input is two lists of positions describing the point in each polygon.
 *
 * The output is the following:
 * 	1: Whether the algorithm can generate valid polygons, (the input geometry will be outputted if
 * not valid)
 *  2: List of Vertex describing how to interpolate any attribute.
 *  3: Offsets to determent the start and end of each polygon of the output.
 *  4: List of Intersection points.
 *
 */

namespace blender::polygonboolean {

enum BooleanMode {
  /* (A*B) Intersection of A and B. */
  A_AND_B,
  /* (A+B) Union of A and B. */
  A_OR_B,
  /* (A-B) Differences of A with B. */
  A_NOT_B,
  /* (B-C) Differences of B with A. */
  B_NOT_A,
};

enum HoleMode {
  /* Generates the base polygons and holes in an arbitrary order. */
  WITH_HOLES,
  /* Generates holes with the base polygon being the first and all others being holes. */
  WITH_ORDERED_HOLES,
  /* Generates only the base polygon without any holes.*/
  WITHOUT_HOLES,
};

struct InputMode {
  BooleanMode boolean_mode;
  HoleMode hole_mode;
};

enum VertexType {
  PointA,
  PointB,
  Intersection,
};

/**
 * `type` determent which array the `point_id` refers to.
 */
struct Vertex {
  VertexType type;
  int point_id;
};

struct IntersectionPoint {
  int point_a;
  int point_b;
  /**
   * `alpha_a` is the factor between point_a and point_a + 1 (i.e. the next point)
   * And the same is true for `B`
   */
  float alpha_a;
  float alpha_b;
};

struct BooleanResult {
  bool valid_geometry;
  Array<Vertex> verts;
  Array<int> offsets;
  Array<IntersectionPoint> intersections_data;
};

template<typename T>
void interpolate_data_from_ab_result(const VArray<T> data_a,
                                     const VArray<T> data_b,
                                     const BooleanResult &result,
                                     MutableSpan<T> dst_attr);
template<typename T>
void interpolate_data_from_a_result(const VArray<T> data_a,
                                    const BooleanResult &result,
                                    MutableSpan<T> dst_attr);
template<typename T>
void interpolate_data_from_b_result(const VArray<T> data_b,
                                    const BooleanResult &result,
                                    MutableSpan<T> dst_attr);

template<typename T>
Array<T> interpolate_data_from_a_result(const Span<T> data_a, const BooleanResult &result);
template<typename T>
Array<T> interpolate_data_from_b_result(const Span<T> data_b, const BooleanResult &result);
template<typename T>
Array<T> interpolate_data_from_ab_result(const Span<T> data_a,
                                         const Span<T> data_b,
                                         const BooleanResult &result);

BooleanResult curve_boolean_calc(const InputMode input_mode,
                                 Span<float2> curve_a,
                                 Span<float2> curve_b);
/**
 * `Cut` behaves like `A_NOT_B` but with `A` not having any fill, and so `A` is cut into separate
 * parts without any segments of `B` is left in the result.
 */
BooleanResult curve_boolean_cut(const bool is_a_cyclic,
                                Span<float2> curve_a,
                                Span<float2> curve_b);

template<typename T>
static T interpolate_data_of_a_intersection_point(const VArray<T> data_a,
                                                  const IntersectionPoint &inter_point)
{
  const T a0 = data_a[inter_point.point_a];
  const T a1 = data_a[(inter_point.point_a + 1) % data_a.size()];
  const float alpha_a = inter_point.alpha_a;

  return math::interpolate(a0, a1, alpha_a);
}

template<typename T>
static T interpolate_data_of_b_intersection_point(const VArray<T> data_b,
                                                  const IntersectionPoint &inter_point)
{
  const T b0 = data_b[inter_point.point_b];
  const T b1 = data_b[(inter_point.point_b + 1) % data_b.size()];
  const float alpha_b = inter_point.alpha_b;

  return math::interpolate(b0, b1, alpha_b);
}

static Array<int2> calculate_segments(const BooleanResult &result)
{
  const OffsetIndices<int> points_by_polygon = OffsetIndices<int>(result.offsets);
  Array<int2> segment_indices(result.verts.size(), int2(-1, -1));

  for (const int polygon_id : points_by_polygon.index_range()) {
    const IndexRange vert_ids = points_by_polygon[polygon_id];

    int first_i = -1;
    int last_i = -1;

    for (const int i : vert_ids.index_range()) {
      const Vertex &vert = result.verts[vert_ids[i]];
      if (vert.type == VertexType::Intersection) {
        first_i = vert.point_id;
        break;
      }
    }

    for (const int i : vert_ids.index_range()) {
      const Vertex &vert = result.verts[vert_ids[vert_ids.size() - 1 - i]];
      if (vert.type == VertexType::Intersection) {
        last_i = vert.point_id;
        break;
      }
    }

    int inter_point = last_i;

    for (const int i : vert_ids.index_range()) {
      const int j = vert_ids[i];
      const Vertex &vert = result.verts[j];

      if (vert.type == VertexType::Intersection) {
        inter_point = vert.point_id;
      }

      segment_indices[j][0] = inter_point;
    }

    inter_point = first_i;

    for (const int i : vert_ids.index_range()) {
      const int j = vert_ids[vert_ids.size() - 1 - i];
      const Vertex &vert = result.verts[j];

      if (vert.type == VertexType::Intersection) {
        inter_point = vert.point_id;
      }

      segment_indices[j][1] = inter_point;
    }
  }

  return segment_indices;
}

template<typename T>
void interpolate_data_from_a_result(const VArray<T> data_a,
                                    const BooleanResult &result,
                                    MutableSpan<T> dst_attr)
{
  const Array<int2> segment_indices = calculate_segments(result);

  for (const int i : result.verts.index_range()) {
    const Vertex &vert = result.verts[i];
    const VertexType &type = vert.type;

    if (type == VertexType::PointA) {
      dst_attr[i] = data_a[vert.point_id];
    }
    else if (type == VertexType::PointB) {
      const int2 &segment = segment_indices[i];

      const IntersectionPoint &i0 = result.intersections_data[segment[0]];
      const IntersectionPoint &i1 = result.intersections_data[segment[1]];

      const T a0 = interpolate_data_of_a_intersection_point<T>(data_a, i0);
      const T a1 = interpolate_data_of_a_intersection_point<T>(data_a, i1);

      const float min_b = i0.point_b + i0.alpha_b;
      const float max_b = i1.point_b + i1.alpha_b;
      const float id_b = vert.point_id;

      const float alpha = (id_b - min_b) / (max_b - min_b);

      dst_attr[i] = math::interpolate(a0, a1, alpha);
    }
    else if (type == VertexType::Intersection) {
      const IntersectionPoint &inter_point = result.intersections_data[vert.point_id];
      dst_attr[i] = interpolate_data_of_a_intersection_point<T>(data_a, inter_point);
    }
  }
}

template<typename T>
void interpolate_data_from_b_result(const VArray<T> data_b,
                                    const BooleanResult &result,
                                    MutableSpan<T> dst_attr)
{
  const Array<int2> segment_indices = calculate_segments(result);

  for (const int i : result.verts.index_range()) {
    const Vertex &vert = result.verts[i];
    const VertexType &type = vert.type;

    if (type == VertexType::PointA) {
      const int2 &segment = segment_indices[i];

      const IntersectionPoint &i0 = result.intersections_data[segment[0]];
      const IntersectionPoint &i1 = result.intersections_data[segment[1]];

      const T b0 = interpolate_data_of_b_intersection_point<T>(data_b, i0);
      const T b1 = interpolate_data_of_b_intersection_point<T>(data_b, i1);

      const float min_a = i0.point_a + i0.alpha_a;
      const float max_a = i1.point_a + i1.alpha_a;
      const float id_a = vert.point_id;

      const float alpha = (id_a - min_a) / (max_a - min_a);

      dst_attr[i] = math::interpolate(b0, b1, alpha);
    }
    else if (type == VertexType::PointB) {
      dst_attr[i] = data_b[vert.point_id];
    }
    else if (type == VertexType::Intersection) {
      const IntersectionPoint &inter_point = result.intersections_data[vert.point_id];
      dst_attr[i] = interpolate_data_of_b_intersection_point<T>(data_b, inter_point);
    }
  }
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

      const T a = interpolate_data_of_a_intersection_point<T>(data_a, inter_point);
      const T b = interpolate_data_of_b_intersection_point<T>(data_b, inter_point);

      dst_attr[i] = math::interpolate(a, b, 0.5f);
    }
  }
}

template<typename T>
Array<T> interpolate_data_from_a_result(const Span<T> data_a, const BooleanResult &result)
{
  Array<T> attribute_out(result.verts.size());

  interpolate_data_from_a_result(
      VArray<T>::ForSpan(data_a), result, attribute_out.as_mutable_span());

  return attribute_out;
}

template<typename T>
Array<T> interpolate_data_from_b_result(const Span<T> data_b, const BooleanResult &result)
{
  Array<T> attribute_out(result.verts.size());

  interpolate_data_from_b_result(
      VArray<T>::ForSpan(data_b), result, attribute_out.as_mutable_span());

  return attribute_out;
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

}  // namespace blender::polygonboolean
