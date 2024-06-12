/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>

#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "BLI_array.hh"
#include "BLI_index_range.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_offset_indices.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

namespace blender::geometry::dyntopo {

template<typename T, int num>
std::ostream &operator<<(std::ostream &stream, const std::array<T, num> &data)
{
  stream << "{";
  for (const int64_t i : IndexRange(num)) {
    stream << data[i] << (num - 1 == i ? "" : ", ");
  }
  stream << "}";
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, const Span<T> span)
{
  for (const int64_t i : span.index_range()) {
    stream << span[i] << (span.size() - 1 == i ? "" : ", ");
  }
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, MutableSpan<T> span)
{
  stream << span.as_span();
  return stream;
}

template<typename In, typename Out, typename Func>
static void parallel_transform(const Span<In> src,
                               const int64_t grain_size,
                               MutableSpan<Out> dst,
                               const Func func)
{
  BLI_assert(src.size() == dst.size());
  threading::parallel_for(src.index_range(), grain_size, [&](const IndexRange range) {
    const Span<In> src_slice = src.slice(range);
    MutableSpan<Out> dst_slice = dst.slice(range);
    std::transform(src_slice.begin(), src_slice.end(), dst_slice.begin(), func);
  });
}

static float len_squared_to_tris(const std::array<float3, 3> tri, const float3 pos)
{
  float3 result;
  closest_on_tri_to_point_v3(result, pos, tri[0], tri[1], tri[2]);
  return math::distance_squared(result, pos);
}

static int3 gather_tri(const IndexRange range, const Span<int> indices)
{
  BLI_assert(range.size() == 3);
  return int3(indices[range[0]], indices[range[1]], indices[range[2]]);
}

Mesh *subdivide(const Mesh &src_mesh,
                const Span<float2> projection,
                const float2 position,
                const float radius,
                const float max_length)
{
  BLI_assert([&]() -> bool {
    const OffsetIndices<int> faces = src_mesh.faces();
    for (const int face_i : faces.index_range()) {
      if (faces[face_i].size() != 3) {
        return false;
      }
    }
    return true;
  }());

  const OffsetIndices<int> faces = src_mesh.faces();
  const Span<int> corner_edges = src_mesh.corner_edges();
  const Span<int> corner_verts = src_mesh.corner_verts();

  Array<int> edge_to_face_offset_data;
  Array<int> edge_to_face_indices;
  const GroupedSpan<int> edge_to_face_map = bke::mesh::build_edge_to_face_map(
      faces, corner_edges, src_mesh.edges_num, edge_to_face_offset_data, edge_to_face_indices);

  Array<int> face_total_points(src_mesh.faces_num + 1);

  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    const int3 edges = gather_tri(face, corner_edges);
    const int3 verts = gather_tri(face, corner_verts);

    const Span<int> a_faces = edge_to_face_map[edges[0]];
    const Span<int> b_faces = edge_to_face_map[edges[1]];
    const Span<int> c_faces = edge_to_face_map[edges[2]];

    const int3 a_verts = gather_tri(faces[a_faces[0] == face_i ? a_faces[1] : a_faces[0]],
                                    corner_verts);
    const int3 b_verts = gather_tri(faces[b_faces[0] == face_i ? b_faces[1] : b_faces[0]],
                                    corner_verts);
    const int3 c_verts = gather_tri(faces[c_faces[0] == face_i ? c_faces[1] : c_faces[0]],
                                    corner_verts);
  }

  std::cout << face_total_points.as_span() << ";\n";

  return nullptr;
}

}  // namespace blender::geometry::dyntopo
