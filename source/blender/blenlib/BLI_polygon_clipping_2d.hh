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

BooleanResult curve_boolean_calc(const BooleanMode mode,
                                 Span<float2> curve_a,
                                 Span<float2> curve_b);
BooleanResult curve_boolean_cut(Span<float2> curve_a, Span<float2> curve_b);

}  // namespace blender::polygonboolean
