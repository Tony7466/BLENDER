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
Array<T> interpolate_data_from_ab_result(const Span<T> data_a,
                                         const Span<T> data_b,
                                         const BooleanResult &result);

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
void interpolate_data_from_a_result(const VArray<T> data_a,
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
      /* TODO: Interpolate between start and end of the segment. */
      dst_attr[i] = data_a.first();
    }
    else if (type == VertexType::Intersection) {
      const IntersectionPoint &inter_point = result.intersections_data[vert.point_id];

      const T a0 = data_a[inter_point.point_a];
      const T a1 = data_a[(inter_point.point_a + 1) % data_a.size()];
      const float alpha_a = inter_point.alpha_a;

      dst_attr[i] = math::interpolate(a0, a1, alpha_a);
    }
  }
}

template<typename T>
void interpolate_data_from_b_result(const VArray<T> data_b,
                                    const BooleanResult &result,
                                    MutableSpan<T> dst_attr)
{
  for (const int i : result.verts.index_range()) {
    const Vertex &vert = result.verts[i];
    const VertexType &type = vert.type;

    if (type == VertexType::PointA) {
      /* TODO: Interpolate between start and end of the segment. */
      dst_attr[i] = data_b.first();
    }
    else if (type == VertexType::PointB) {
      dst_attr[i] = data_b[vert.point_id];
    }
    else if (type == VertexType::Intersection) {
      const IntersectionPoint &inter_point = result.intersections_data[vert.point_id];

      const T b0 = data_b[inter_point.point_b];
      const T b1 = data_b[(inter_point.point_b + 1) % data_b.size()];
      const float alpha_b = inter_point.alpha_b;

      dst_attr[i] = math::interpolate(b0, b1, alpha_b);
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

}  // namespace blender::polygonboolean
