/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_array.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

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
Array<T> interpolate_attribute_from_ab_result(const Span<T> curve_a,
                                              const Span<T> curve_b,
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

}  // namespace blender::polygonboolean
