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

static float len_squared_to_tris(const std::array<float2, 3> tri, const float2 pos)
{
  float3 result;
  std::array<float3, 3> tri_3d;
  tri_3d[0] = float3(tri[0], 0.0f);
  tri_3d[1] = float3(tri[1], 0.0f);
  tri_3d[2] = float3(tri[2], 0.0f);
  const float3 pos_3d(pos, 0.0f);
  closest_on_tri_to_point_v3(result, pos_3d, tri_3d[0], tri_3d[1], tri_3d[2]);
  return math::distance_squared(result, pos_3d);
}

static int dominant_axis(const float3 a)
{
  return ((a.x > a.y) ? ((a.x > a.z) ? 0 : 2) : ((a.y > a.z) ? 1 : 2));
}

struct Triangle {
  float2 a;
  float2 b;
  float2 c;

  /* Neighboards of triangle: abd, bce, caf. */
  float2 d;
  float2 e;
  float2 f;

  bool ab_is_real_edge;
  bool bc_is_real_edge;
  bool ca_is_real_edge;
};

[[nodiscard]] static int edge_count_segments(const float2 a_vert,
                                             const float2 b_vert,
                                             const float2 c_vert,
                                             const float2 d_vert,
                                             const float2 centre,
                                             const float radius,
                                             const float max_length,
                                             int &r_total_verts_in)
{
  int segments_num;
  Vector<std::array<float2, 4>> stack = {{a_vert, b_vert, c_vert, d_vert}};
  while (!stack.is_empty()) {
    const std::array<float2, 4> edge_and_tris = stack.pop_last();

    const float segment_length = math::distance_squared(edge_and_tris[0], edge_and_tris[1]);
    if (segment_length < max_length) {
      segments_num++;
      continue;
    }

    const bool left_is_affected = len_squared_to_tris(
                                      {edge_and_tris[0], edge_and_tris[1], edge_and_tris[2]},
                                      centre) <= radius;
    const bool right_is_affected = len_squared_to_tris(
                                       {edge_and_tris[0], edge_and_tris[1], edge_and_tris[3]},
                                       centre) <= radius;
    if (!(left_is_affected || right_is_affected)) {
      segments_num++;
      continue;
    }

    const float ac_length = math::distance_squared(edge_and_tris[0], edge_and_tris[2]);
    const float bc_length = math::distance_squared(edge_and_tris[1], edge_and_tris[2]);
    const float abc_max = math::max(ac_length, bc_length);

    const float ad_length = math::distance_squared(edge_and_tris[0], edge_and_tris[3]);
    const float bd_length = math::distance_squared(edge_and_tris[1], edge_and_tris[3]);
    const float abd_max = math::max(ad_length, bd_length);

    if (math::max(abc_max, abd_max) < radius) {
      continue;
    }

    if (abc_max > abd_max) {
      if (ac_length > bc_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
      }
    }
    else {
      if (ad_length > bd_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
      }
    }
  }
  return segments_num;
}

static void edge_subdivide_count(const float2 a_vert,
                                 const float2 b_vert,
                                 const float2 c_vert,
                                 const float2 d_vert,
                                 const float2 centre,
                                 const float radius,
                                 const float max_length,
                                 int &r_total_verts_in)
{
  // std::cout << "Face Size:\n";
  Vector<std::array<float2, 4>> stack = {{a_vert, b_vert, c_vert, d_vert}};
  while (!stack.is_empty()) {
    const std::array<float2, 4> edge_and_tris = stack.pop_last();

    const bool left_is_affected = len_squared_to_tris(
                                      {edge_and_tris[0], edge_and_tris[1], edge_and_tris[2]},
                                      centre) <= radius;
    const bool right_is_affected = len_squared_to_tris(
                                       {edge_and_tris[0], edge_and_tris[1], edge_and_tris[3]},
                                       centre) <= radius;
    if (!(left_is_affected || right_is_affected)) {
      continue;
    }

    const float ab_length = math::distance_squared(edge_and_tris[0], edge_and_tris[1]);

    const float ac_length = math::distance_squared(edge_and_tris[0], edge_and_tris[2]);
    const float bc_length = math::distance_squared(edge_and_tris[1], edge_and_tris[2]);
    const float abc_max = math::max(ac_length, bc_length);

    const float ad_length = math::distance_squared(edge_and_tris[0], edge_and_tris[3]);
    const float bd_length = math::distance_squared(edge_and_tris[1], edge_and_tris[3]);
    const float abd_max = math::max(ad_length, bd_length);

    if (math::max(math::max(ab_length, abc_max), abd_max) < radius) {
      continue;
    }

    if (ab_length > abc_max && ab_length > abd_max) {
      const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[1]);
      stack.append({edge_and_tris[0], mid, edge_and_tris[2], edge_and_tris[3]});
      stack.append({mid, edge_and_tris[1], edge_and_tris[2], edge_and_tris[3]});
      r_total_verts_in++;
      continue;
    }

    if (abc_max > abd_max) {
      if (ac_length > bc_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
      }
    }
    else {
      if (ad_length > bd_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
      }
    }
  }
}

static void face_subdivide_count(const float2 a_vert,
                                 const float2 b_vert,
                                 const float2 c_vert,
                                 const float2 d_vert,
                                 const float2 e_vert,
                                 const float2 f_vert,
                                 const float2 centre,
                                 const float radius,
                                 const float max_length,
                                 int &r_total_verts_in)
{
  // std::cout << "Face Size:\n";
  Vector<Triangle> stack = {{a_vert, b_vert, c_vert, d_vert, e_vert, f_vert, true, true, true}};
  while (!stack.is_empty()) {
    const Triangle triangle = stack.pop_last();

    if (len_squared_to_tris({triangle.a, triangle.b, triangle.c}, centre) > radius) {
      continue;
    }

    const float ab_length = math::distance_squared(triangle.a, triangle.b);
    const float bc_length = math::distance_squared(triangle.b, triangle.c);
    const float ca_length = math::distance_squared(triangle.c, triangle.a);
    const float3 triangle_sides(ab_length, bc_length, ca_length);
    const int largest_side = dominant_axis(triangle_sides);
    if (triangle_sides[largest_side] < max_length) {
      continue;
    }

    bool sibdivide_real_edge = false;
    switch (largest_side) {
      case 0: {
        sibdivide_real_edge = triangle.ab_is_real_edge;
        const float2 mid = math::midpoint(triangle.a, triangle.b);
        stack.append({triangle.a,
                      mid,
                      triangle.c,
                      triangle.d,
                      triangle.b,
                      triangle.f,
                      triangle.ab_is_real_edge,
                      false,
                      triangle.ca_is_real_edge});
        stack.append({mid,
                      triangle.b,
                      triangle.c,
                      triangle.d,
                      triangle.e,
                      triangle.a,
                      triangle.ab_is_real_edge,
                      triangle.bc_is_real_edge,
                      false});
        break;
      }
      case 1: {
        sibdivide_real_edge = triangle.bc_is_real_edge;
        const float2 mid = math::midpoint(triangle.b, triangle.c);
        stack.append({triangle.a,
                      triangle.b,
                      mid,
                      triangle.d,
                      triangle.e,
                      triangle.c,
                      triangle.ab_is_real_edge,
                      triangle.bc_is_real_edge,
                      false});
        stack.append({triangle.a,
                      mid,
                      triangle.c,
                      triangle.b,
                      triangle.e,
                      triangle.f,
                      false,
                      triangle.bc_is_real_edge,
                      triangle.ca_is_real_edge});
        break;
      }
      case 2: {
        sibdivide_real_edge = triangle.ca_is_real_edge;
        const float2 mid = math::midpoint(triangle.c, triangle.a);
        stack.append({triangle.a,
                      triangle.b,
                      mid,
                      triangle.d,
                      triangle.c,
                      triangle.f,
                      triangle.ab_is_real_edge,
                      false,
                      triangle.ca_is_real_edge});
        stack.append({mid,
                      triangle.b,
                      triangle.c,
                      triangle.a,
                      triangle.e,
                      triangle.f,
                      false,
                      triangle.bc_is_real_edge,
                      triangle.ca_is_real_edge});
        break;
      }
    }

    if (!sibdivide_real_edge) {
      r_total_verts_in++;
    }
  }
}

static void edge_subdivide_uv(const float2 a_vert,
                              const float2 b_vert,
                              const float2 c_vert,
                              const float2 d_vert,
                              const float2 centre,
                              const float radius,
                              const float max_length,
                              MutableSpan<float2> r_positions)
{
  int r_total_verts_in = 0;
  // std::cout << "Face Size:\n";
  Vector<std::array<float2, 4>> stack = {{a_vert, b_vert, c_vert, d_vert}};
  while (!stack.is_empty()) {
    const std::array<float2, 4> edge_and_tris = stack.pop_last();

    const bool left_is_affected = len_squared_to_tris(
                                      {edge_and_tris[0], edge_and_tris[1], edge_and_tris[2]},
                                      centre) <= radius;
    const bool right_is_affected = len_squared_to_tris(
                                       {edge_and_tris[0], edge_and_tris[1], edge_and_tris[3]},
                                       centre) <= radius;
    if (!(left_is_affected || right_is_affected)) {
      continue;
    }

    const float ab_length = math::distance_squared(edge_and_tris[0], edge_and_tris[1]);

    const float ac_length = math::distance_squared(edge_and_tris[0], edge_and_tris[2]);
    const float bc_length = math::distance_squared(edge_and_tris[1], edge_and_tris[2]);
    const float abc_max = math::max(ac_length, bc_length);

    const float ad_length = math::distance_squared(edge_and_tris[0], edge_and_tris[3]);
    const float bd_length = math::distance_squared(edge_and_tris[1], edge_and_tris[3]);
    const float abd_max = math::max(ad_length, bd_length);

    if (math::max(math::max(ab_length, abc_max), abd_max) < radius) {
      continue;
    }

    if (ab_length > abc_max && ab_length > abd_max) {
      const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[1]);
      stack.append({edge_and_tris[0], mid, edge_and_tris[2], edge_and_tris[3]});
      stack.append({mid, edge_and_tris[1], edge_and_tris[2], edge_and_tris[3]});
      r_positions[r_total_verts_in] = mid;
      r_total_verts_in++;
      continue;
    }

    if (abc_max > abd_max) {
      if (ac_length > bc_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
      }
    }
    else {
      if (ad_length > bd_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
      }
    }
  }
}

static void edge_subdivide_verts(const float2 a_vert,
                                 const float2 b_vert,
                                 const float2 c_vert,
                                 const float2 d_vert,
                                 const float2 centre,
                                 const float radius,
                                 const float max_length,
                                 const int2 edge,
                                 const IndexRange verts_range,
                                 MutableSpan<int2> r_verts)
{
  if (verts_range.is_empty()) {
    r_verts[0] = edge;
    return;
  }
}

static void face_subdivide_uv(const float2 a_vert,
                              const float2 b_vert,
                              const float2 c_vert,
                              const float2 d_vert,
                              const float2 e_vert,
                              const float2 f_vert,
                              const float2 centre,
                              const float radius,
                              const float max_length,
                              MutableSpan<float2> r_positions)
{
  // std::cout << "Face Size:\n";
  Vector<Triangle> stack = {{a_vert, b_vert, c_vert, d_vert, e_vert, f_vert, true, true, true}};
  int r_total_verts_in = 0;
  while (!stack.is_empty()) {
    const Triangle triangle = stack.pop_last();

    if (len_squared_to_tris({triangle.a, triangle.b, triangle.c}, centre) > radius) {
      continue;
    }

    const float ab_length = math::distance_squared(triangle.a, triangle.b);
    const float bc_length = math::distance_squared(triangle.b, triangle.c);
    const float ca_length = math::distance_squared(triangle.c, triangle.a);
    const float3 triangle_sides(ab_length, bc_length, ca_length);
    const int largest_side = dominant_axis(triangle_sides);
    if (triangle_sides[largest_side] < max_length) {
      continue;
    }

    bool sibdivide_real_edge = false;
    switch (largest_side) {
      case 0: {
        sibdivide_real_edge = triangle.ab_is_real_edge;
        const float2 mid = math::midpoint(triangle.a, triangle.b);
        stack.append({triangle.a,
                      mid,
                      triangle.c,
                      triangle.d,
                      triangle.b,
                      triangle.f,
                      triangle.ab_is_real_edge,
                      false,
                      triangle.ca_is_real_edge});
        stack.append({mid,
                      triangle.b,
                      triangle.c,
                      triangle.d,
                      triangle.e,
                      triangle.a,
                      triangle.ab_is_real_edge,
                      triangle.bc_is_real_edge,
                      false});
        if (!sibdivide_real_edge) {
          r_positions[r_total_verts_in] = mid;
        }
        break;
      }
      case 1: {
        sibdivide_real_edge = triangle.bc_is_real_edge;
        const float2 mid = math::midpoint(triangle.b, triangle.c);
        stack.append({triangle.a,
                      triangle.b,
                      mid,
                      triangle.d,
                      triangle.e,
                      triangle.c,
                      triangle.ab_is_real_edge,
                      triangle.bc_is_real_edge,
                      false});
        stack.append({triangle.a,
                      mid,
                      triangle.c,
                      triangle.b,
                      triangle.e,
                      triangle.f,
                      false,
                      triangle.bc_is_real_edge,
                      triangle.ca_is_real_edge});
        if (!sibdivide_real_edge) {
          r_positions[r_total_verts_in] = mid;
        }
        break;
      }
      case 2: {
        sibdivide_real_edge = triangle.ca_is_real_edge;
        const float2 mid = math::midpoint(triangle.c, triangle.a);
        stack.append({triangle.a,
                      triangle.b,
                      mid,
                      triangle.d,
                      triangle.c,
                      triangle.f,
                      triangle.ab_is_real_edge,
                      false,
                      triangle.ca_is_real_edge});
        stack.append({mid,
                      triangle.b,
                      triangle.c,
                      triangle.a,
                      triangle.e,
                      triangle.f,
                      false,
                      triangle.bc_is_real_edge,
                      triangle.ca_is_real_edge});
        if (!sibdivide_real_edge) {
          r_positions[r_total_verts_in] = mid;
        }
        break;
      }
    }

    if (!sibdivide_real_edge) {
      r_total_verts_in++;
    }
  }
}

Mesh *subdivide(const Mesh &src_mesh,
                const Span<float2> projection,
                const float2 centre,
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

  const float squared_radius = radius * radius;

  const OffsetIndices<int> faces = src_mesh.faces();
  const Span<int2> edges = src_mesh.edges();
  const Span<int> corner_edges = src_mesh.corner_edges();
  const Span<int> corner_verts = src_mesh.corner_verts();

  Array<int> edge_to_face_offset_data;
  Array<int> edge_to_face_indices;
  const GroupedSpan<int> edge_to_face_map = bke::mesh::build_edge_to_face_map(
      faces, corner_edges, src_mesh.edges_num, edge_to_face_offset_data, edge_to_face_indices);

  Array<int> edge_total_verts(src_mesh.edges_num + 1);
  Array<int> face_total_verts(src_mesh.faces_num + 1);

  for (const int edge_i : IndexRange(src_mesh.edges_num)) {
    const int2 edge = edges[edge_i];

    const Span<int> edge_faces = edge_to_face_map[edge_i];

    const int3 left_verts = gather_tri(faces[edge_faces[0]], corner_verts);
    const int3 right_verts = gather_tri(faces[edge_faces[0]], corner_verts);

    BLI_assert(elem(left_verts, edge[0]) && elem(left_verts, edge[1]));
    BLI_assert(elem(right_verts, edge[0]) && elem(right_verts, edge[1]));

    const int a_vert = exclusive_one(left_verts, edge);
    const int b_vert = exclusive_one(right_verts, edge);

    int total_verts_in = 0;
    edge_subdivide_count(projection[edge[0]],
                         projection[edge[1]],
                         projection[a_vert],
                         projection[b_vert],
                         centre,
                         squared_radius,
                         max_length,
                         total_verts_in);
    edge_total_verts[edge_i] = total_verts_in;
  }

  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    const int3 face_edges = gather_tri(face, corner_edges);
    const int3 face_verts = gather_tri(face, corner_verts);

    const int2 edge_ab = edges[face_edges[0]];
    const int2 edge_bc = edges[face_edges[1]];
    const int2 edge_ca = edges[face_edges[2]];

    const Span<int> ab_faces = edge_to_face_map[face_edges[0]];
    const Span<int> bc_faces = edge_to_face_map[face_edges[1]];
    const Span<int> ca_faces = edge_to_face_map[face_edges[2]];

    const int3 abd_verts = gather_tri(faces[ab_faces[0] == face_i ? ab_faces[1] : ab_faces[0]],
                                      corner_verts);
    const int3 bce_verts = gather_tri(faces[bc_faces[0] == face_i ? bc_faces[1] : bc_faces[0]],
                                      corner_verts);
    const int3 caf_verts = gather_tri(faces[ca_faces[0] == face_i ? ca_faces[1] : ca_faces[0]],
                                      corner_verts);

    BLI_assert(elem(abd_verts, edge_ab[0]) && elem(abd_verts, edge_ab[1]));
    BLI_assert(elem(bce_verts, edge_bc[0]) && elem(bce_verts, edge_bc[1]));
    BLI_assert(elem(caf_verts, edge_ca[0]) && elem(caf_verts, edge_ca[1]));

    const int a_vert = exclusive_one(face_verts, edge_bc);
    const int b_vert = exclusive_one(face_verts, edge_ca);
    const int c_vert = exclusive_one(face_verts, edge_ab);

    const int d_vert = exclusive_one(abd_verts, edge_ab);
    const int e_vert = exclusive_one(bce_verts, edge_bc);
    const int f_vert = exclusive_one(caf_verts, edge_ca);

    BLI_assert(!elem(face_verts, d_vert));
    BLI_assert(!elem(face_verts, e_vert));
    BLI_assert(!elem(face_verts, f_vert));

    int total_verts_in = 0;
    face_subdivide_count(projection[a_vert],
                         projection[b_vert],
                         projection[c_vert],
                         projection[d_vert],
                         projection[e_vert],
                         projection[f_vert],
                         centre,
                         squared_radius,
                         max_length,
                         total_verts_in);
    face_total_verts[face_i] = total_verts_in;
  }

  std::cout << ">> Edges: " << edge_total_verts.as_span() << ";\n";
  std::cout << std::endl;
  std::cout << ">> Faces: " << face_total_verts.as_span() << ";\n";

  const OffsetIndices<int> subdive_edge_verts = offset_indices::accumulate_counts_to_offsets(
      edge_total_verts);
  const OffsetIndices<int> subdive_face_verts = offset_indices::accumulate_counts_to_offsets(
      face_total_verts);

  Mesh *dst_mesh = BKE_mesh_new_nomain(src_mesh.verts_num + subdive_face_verts.total_size() +
                                           subdive_edge_verts.total_size(),
                                       src_mesh.edges_num + subdive_edge_verts.total_size(),
                                       0,
                                       0);
  MutableSpan<float3> positions = dst_mesh->vert_positions_for_write();

  positions.take_front(src_mesh.verts_num).copy_from(src_mesh.vert_positions());

  for (const int edge_i : IndexRange(src_mesh.edges_num)) {
    const int2 edge = edges[edge_i];

    const Span<int> edge_faces = edge_to_face_map[edge_i];

    const int3 left_verts = gather_tri(faces[edge_faces[0]], corner_verts);
    const int3 right_verts = gather_tri(faces[edge_faces[0]], corner_verts);

    BLI_assert(elem(left_verts, edge[0]) && elem(left_verts, edge[1]));
    BLI_assert(elem(right_verts, edge[0]) && elem(right_verts, edge[1]));

    const int a_vert = exclusive_one(left_verts, edge);
    const int b_vert = exclusive_one(right_verts, edge);

    IndexRange points_range = subdive_edge_verts[edge_i].shift(src_mesh.verts_num +
                                                               subdive_face_verts.total_size());
    Array<float2> uv_positions(points_range.size());

    edge_subdivide_uv(projection[edge[0]],
                      projection[edge[1]],
                      projection[a_vert],
                      projection[b_vert],
                      centre,
                      squared_radius,
                      max_length,
                      uv_positions);

    parallel_transform(uv_positions.as_span(),
                       4096,
                       positions.slice(points_range),
                       [&](const float2 uv) { return float3(uv, 0.0f); });
  }

  MutableSpan<int2> dst_edges = dst_mesh->edges_for_write();
  dst_edges.fill({0, 1});

  for (const int edge_i : IndexRange(src_mesh.edges_num)) {
    const int2 edge = edges[edge_i];

    const Span<int> edge_faces = edge_to_face_map[edge_i];

    const int3 left_verts = gather_tri(faces[edge_faces[0]], corner_verts);
    const int3 right_verts = gather_tri(faces[edge_faces[0]], corner_verts);

    BLI_assert(elem(left_verts, edge[0]) && elem(left_verts, edge[1]));
    BLI_assert(elem(right_verts, edge[0]) && elem(right_verts, edge[1]));

    const int a_vert = exclusive_one(left_verts, edge);
    const int b_vert = exclusive_one(right_verts, edge);

    const IndexRange points_range = subdive_edge_verts[edge_i];
    const IndexRange edges_range = IndexRange::from_begin_size(points_range.start() + edge_i,
                                                               points_range.size() + 1);

    edge_subdivide_verts(projection[edge[0]],
                         projection[edge[1]],
                         projection[a_vert],
                         projection[b_vert],
                         centre,
                         squared_radius,
                         max_length,
                         edge,
                         points_range,
                         dst_edges.slice(edges_range));
  }

  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    const int3 face_edges = gather_tri(face, corner_edges);
    const int3 face_verts = gather_tri(face, corner_verts);

    const int2 edge_ab = edges[face_edges[0]];
    const int2 edge_bc = edges[face_edges[1]];
    const int2 edge_ca = edges[face_edges[2]];

    const Span<int> ab_faces = edge_to_face_map[face_edges[0]];
    const Span<int> bc_faces = edge_to_face_map[face_edges[1]];
    const Span<int> ca_faces = edge_to_face_map[face_edges[2]];

    const int3 abd_verts = gather_tri(faces[ab_faces[0] == face_i ? ab_faces[1] : ab_faces[0]],
                                      corner_verts);
    const int3 bce_verts = gather_tri(faces[bc_faces[0] == face_i ? bc_faces[1] : bc_faces[0]],
                                      corner_verts);
    const int3 caf_verts = gather_tri(faces[ca_faces[0] == face_i ? ca_faces[1] : ca_faces[0]],
                                      corner_verts);

    BLI_assert(elem(abd_verts, edge_ab[0]) && elem(abd_verts, edge_ab[1]));
    BLI_assert(elem(bce_verts, edge_bc[0]) && elem(bce_verts, edge_bc[1]));
    BLI_assert(elem(caf_verts, edge_ca[0]) && elem(caf_verts, edge_ca[1]));

    const int a_vert = exclusive_one(face_verts, edge_bc);
    const int b_vert = exclusive_one(face_verts, edge_ca);
    const int c_vert = exclusive_one(face_verts, edge_ab);

    const int d_vert = exclusive_one(abd_verts, edge_ab);
    const int e_vert = exclusive_one(bce_verts, edge_bc);
    const int f_vert = exclusive_one(caf_verts, edge_ca);

    BLI_assert(!elem(face_verts, d_vert));
    BLI_assert(!elem(face_verts, e_vert));
    BLI_assert(!elem(face_verts, f_vert));

    IndexRange points_range = subdive_face_verts[face_i].shift(src_mesh.verts_num);
    Array<float2> uv_positions(points_range.size());

    face_subdivide_uv(projection[a_vert],
                      projection[b_vert],
                      projection[c_vert],
                      projection[d_vert],
                      projection[e_vert],
                      projection[f_vert],
                      centre,
                      squared_radius,
                      max_length,
                      uv_positions);

    parallel_transform(uv_positions.as_span(),
                       4096,
                       positions.slice(points_range),
                       [&](const float2 uv) { return float3(uv, 0.0f); });
  }

  return dst_mesh;
}

}  // namespace blender::geometry::dyntopo
