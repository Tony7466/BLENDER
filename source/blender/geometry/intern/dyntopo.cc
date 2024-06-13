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

static bool elem(const int2 items, const int to_check)
{
  return ELEM(to_check, items[0], items[1]);
}

static bool elem(const int3 items, const int to_check)
{
  return ELEM(to_check, items[0], items[1], items[2]);
}

static int exclusive_one(const int3 set, const int2 part)
{
  BLI_assert(elem(set, part[0]));
  BLI_assert(elem(set, part[1]));
  BLI_assert(set[0] >= 0 && set[1] >= 0 && set[2] >= 0);
  const int exclusive_elem = (set[0] - part[0]) + (set[1] - part[1]) + set[2];
  BLI_assert(elem(set, exclusive_elem));
  BLI_assert(!elem(part, exclusive_elem));
  BLI_assert(exclusive_elem >= 0);
  return exclusive_elem;
}

static int exclusive_one(const int2 set, const int part)
{
  BLI_assert(elem(set, part));
  BLI_assert(set[0] >= 0 && set[1] >= 0);
  const int exclusive_elem = (set[0] - part) + set[1];
  BLI_assert(elem(set, exclusive_elem));
  BLI_assert(exclusive_elem >= 0);
  return exclusive_elem;
}

static void insert_verts_is(const float2 a_vert,
                            const float2 b_vert,
                            const float2 c_vert,
                            const float2 ab_vert,
                            const float2 bc_vert const float2 ca_vert const float2 position,
                            const float radius,
                            const float max_length,
                            int &r_total_verts_in)
{
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
  const Span<int2> edges = src_mesh.edges();
  const Span<int> corner_edges = src_mesh.corner_edges();
  const Span<int> corner_verts = src_mesh.corner_verts();

  Array<int> edge_to_face_offset_data;
  Array<int> edge_to_face_indices;
  const GroupedSpan<int> edge_to_face_map = bke::mesh::build_edge_to_face_map(
      faces, corner_edges, src_mesh.edges_num, edge_to_face_offset_data, edge_to_face_indices);

  Array<int> face_total_points(src_mesh.faces_num + 1);

  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    const int3 face_edges = gather_tri(face, corner_edges);
    const int3 face_verts = gather_tri(face, corner_verts);

    const int2 edge_a = edges[face_edges[0]];
    const int2 edge_b = edges[face_edges[1]];
    const int2 edge_c = edges[face_edges[2]];

    const Span<int> a_faces = edge_to_face_map[face_edges[0]];
    const Span<int> b_faces = edge_to_face_map[face_edges[1]];
    const Span<int> c_faces = edge_to_face_map[face_edges[2]];

    const int3 a_verts = gather_tri(faces[a_faces[0] == face_i ? a_faces[1] : a_faces[0]],
                                    corner_verts);
    const int3 b_verts = gather_tri(faces[b_faces[0] == face_i ? b_faces[1] : b_faces[0]],
                                    corner_verts);
    const int3 c_verts = gather_tri(faces[c_faces[0] == face_i ? c_faces[1] : c_faces[0]],
                                    corner_verts);

    BLI_assert(elem(a_verts, edge_a[0]) && elem(a_verts, edge_a[1]));
    BLI_assert(elem(b_verts, edge_b[0]) && elem(b_verts, edge_b[1]));
    BLI_assert(elem(c_verts, edge_c[0]) && elem(c_verts, edge_c[1]));

    const int a_vert = exclusive_one(a_verts, edge_a);
    const int b_vert = exclusive_one(b_verts, edge_b);
    const int c_vert = exclusive_one(c_verts, edge_c);

    const int ab_vert = exclusive_one(face_verts, edge_a);
    const int bc_vert = exclusive_one(face_verts, edge_b);
    const int ca_vert = exclusive_one(face_verts, edge_c);

    BLI_assert(!elem(face_verts, a_vert));
    BLI_assert(!elem(face_verts, b_vert));
    BLI_assert(!elem(face_verts, c_vert));

    int total_verts_in = 0;
    insert_verts_is(projection[bc_vert],
                    projection[ca_vert],
                    projection[ab_vert],
                    projection[b_vert],
                    projection[c_vert],
                    projection[a_vert],
                    position,
                    radius,
                    max_length,
                    total_verts_in);
    face_total_points[face_i] = total_verts_in;
  }

  std::cout << face_total_points.as_span() << ";\n";

  return nullptr;
}

}  // namespace blender::geometry::dyntopo
