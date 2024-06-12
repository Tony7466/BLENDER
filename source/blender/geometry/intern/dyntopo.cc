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

static void build_face_to_face_by_edge_map(const OffsetIndices<int> faces,
                                           const Span<int> corner_edges,
                                           const int edges_num,
                                           Array<int> &r_offsets,
                                           Array<int> &r_indices)
{
  Array<int> edge_to_face_offset_data;
  Array<int> edge_to_face_indices;
  const GroupedSpan<int> edge_to_face_map = bke::mesh::build_edge_to_face_map(
      faces, corner_edges, edges_num, edge_to_face_offset_data, edge_to_face_indices);
  const OffsetIndices<int> edge_to_face_offsets(edge_to_face_offset_data);

  r_offsets = Array<int>(faces.size() + 1, 0);
  threading::parallel_for(faces.index_range(), 4096, [&](const IndexRange range) {
    for (const int face_i : range) {
      for (const int edge : corner_edges.slice(faces[face_i])) {
        /* Subtract face itself from the number of faces connected to the edge. */
        r_offsets[face_i] += edge_to_face_offsets[edge].size() - 1;
      }
    }
  });
  const OffsetIndices<int> offsets = offset_indices::accumulate_counts_to_offsets(r_offsets);
  r_indices.reinitialize(offsets.total_size());

  threading::parallel_for(faces.index_range(), 1024, [&](IndexRange range) {
    for (const int face_i : range) {
      MutableSpan<int> neighbors = r_indices.as_mutable_span().slice(offsets[face_i]);
      if (neighbors.is_empty()) {
        continue;
      }
      int count = 0;
      for (const int edge : corner_edges.slice(faces[face_i])) {
        for (const int neighbor : edge_to_face_map[edge]) {
          if (neighbor != face_i) {
            neighbors[count] = neighbor;
            count++;
          }
        }
      }
    }
  });
}

static float len_squared_to_tris(const std::array<float3, 3> tri, const float3 pos)
{
  float3 result;
  closest_on_tri_to_point_v3(result, pos, tri[0], tri[1], tri[2]);
  return math::distance_squared(result, pos);
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
  Array<int> face_to_face_map_offsets;
  Array<int> face_to_face_map_indices;
  build_face_to_face_by_edge_map(src_mesh.faces(),
                                 src_mesh.corner_edges(),
                                 src_mesh.edges_num,
                                 face_to_face_map_offsets,
                                 face_to_face_map_indices);
  GroupedSpan<int>(OffsetIndices<int>(face_to_face_map_offsets), face_to_face_map_indices);

  Array<int> face_total_points(src_mesh.faces_num + 1);

  std::cout << face_total_points.as_span() << ";\n";

  return nullptr;
}

}  // namespace blender::geometry::dyntopo
