/* SPDX-FileCopyrightText: 2023 Blender Authors
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
 * Greiner-Hormann clipping algorithm 2D.
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

struct Vertex {
  VertexType type;
  int point_id;
};

struct IntersectionPoint {
  int point_a;
  int point_b;
  float alpha_a;
  float alpha_b;
};

struct BooleanResult {
  bool valid_geometry;
  Array<Vertex> verts;
  Array<int> offsets;
  Array<IntersectionPoint> intersections_data;
};

BooleanResult curve_boolean_calc(const InputMode input_mode,
                                 Span<float2> curve_a,
                                 Span<float2> curve_b);
BooleanResult curve_boolean_cut(Span<float2> curve_a, Span<float2> curve_b);

}  // namespace blender::polygonboolean
