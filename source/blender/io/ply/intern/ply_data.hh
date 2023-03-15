/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup ply
 */

#pragma once

#include "BLI_array.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

namespace blender::io::ply {

enum PlyDataTypes { NONE, CHAR, UCHAR, SHORT, USHORT, INT, UINT, FLOAT, DOUBLE, PLY_TYPE_COUNT };

struct PlyData {
  Vector<float3> vertices;
  Vector<float3> vertex_normals;
  /* Value between 0 and 1. */
  Vector<float4> vertex_colors;
  Vector<std::pair<int, int>> edges;
  Vector<float3> edge_colors;
  Vector<Array<uint32_t>> faces;
  Vector<float2> uv_coordinates;
};

enum PlyFormatType { ASCII, BINARY_LE, BINARY_BE };

struct PlyProperty {
  std::string name;
  PlyDataTypes type = PlyDataTypes::NONE;
  PlyDataTypes count_type = PlyDataTypes::NONE; /* NONE means it's not a list property */
};

struct PlyElement {
  std::string name;
  int count = 0;
  Vector<PlyProperty> properties;
  int stride = 0;

  void calc_stride();
};

struct PlyHeader {
  int vertex_count = 0;
  int edge_count = 0;
  int face_count = 0;
  int header_size = 0;

  Vector<PlyElement> elements;

  PlyFormatType type;
};

}  // namespace blender::io::ply
