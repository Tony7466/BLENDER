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

static void insert_verts_count(const float2 a_vert,
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
  Vector<Triangle> stack = {{a_vert, b_vert, c_vert, d_vert, e_vert, f_vert}};
  while (!stack.is_empty()) {
    const Triangle triangle_and_neighboards = stack.pop_last();
    const float2 vert_a = triangle_and_neighboards.a;
    const float2 vert_b = triangle_and_neighboards.b;
    const float2 vert_c = triangle_and_neighboards.c;

    const float2 vert_d = triangle_and_neighboards.d;
    const float2 vert_e = triangle_and_neighboards.e;
    const float2 vert_f = triangle_and_neighboards.f;
    // std::cout << "  " << triangle_and_neighboards << ";\n";

    if (len_squared_to_tris({vert_a, vert_b, vert_c}, centre) > radius) {
      const bool a_is_affected = len_squared_to_tris({vert_a, vert_b, vert_d}, centre) < radius;
      const bool b_is_affected = len_squared_to_tris({vert_b, vert_c, vert_e}, centre) < radius;
      const bool c_is_affected = len_squared_to_tris({vert_c, vert_a, vert_f}, centre) < radius;
      if (!(a_is_affected || b_is_affected || c_is_affected)) {
        continue;
      }

      float abd_max = std::numeric_limits<float>::lowest();
      float bce_max = std::numeric_limits<float>::lowest();
      float caf_max = std::numeric_limits<float>::lowest();
      int largest_abd_side = -1;
      int largest_bce_side = -1;
      int largest_caf_side = -1;

      if (a_is_affected) {
        const float ab_length = math::distance_squared(vert_a, vert_b);
        const float bd_length = math::distance_squared(vert_b, vert_d);
        const float da_length = math::distance_squared(vert_d, vert_a);
        const float3 abd(ab_length, bd_length, da_length);
        largest_abd_side = dominant_axis(abd);
        abd_max = abd[largest_abd_side];
      }

      if (b_is_affected) {
        const float bc_length = math::distance_squared(vert_b, vert_c);
        const float ce_length = math::distance_squared(vert_c, vert_e);
        const float eb_length = math::distance_squared(vert_e, vert_b);
        const float3 bce(bc_length, ce_length, eb_length);
        largest_bce_side = dominant_axis(bce);
        bce_max = bce[largest_bce_side];
      }

      if (c_is_affected) {
        const float ca_length = math::distance_squared(vert_c, vert_a);
        const float af_length = math::distance_squared(vert_a, vert_f);
        const float fc_length = math::distance_squared(vert_f, vert_c);
        const float3 caf(ca_length, af_length, fc_length);
        largest_caf_side = dominant_axis(caf);
        caf_max = caf[largest_caf_side];
      }

      const float3 larges_side_in(abd_max, bce_max, caf_max);
      const int larges_side = dominant_axis(larges_side_in);

      if (larges_side_in[larges_side] < max_length) {
        continue;
      }

      switch (larges_side) {
        case 0: {
          switch (largest_abd_side) {
            case 0: {
              const float2 mid = math::midpoint(vert_a, vert_b);
              stack.append({vert_a, mid, vert_c, vert_d, vert_b, vert_f});
              stack.append({mid, vert_b, vert_c, vert_d, vert_e, vert_a});
              r_total_verts_in++;
              break;
            }
            case 1: {
              const float2 mid = math::midpoint(vert_d, vert_b);
              stack.append({vert_a, vert_b, vert_c, mid, vert_e, vert_f});
              break;
            }
            case 2: {
              const float2 mid = math::midpoint(vert_a, vert_d);
              stack.append({vert_a, vert_b, vert_c, mid, vert_e, vert_f});
              break;
            }
          }
          break;
        }
        case 1: {
          switch (largest_bce_side) {
            case 0: {
              const float2 mid = math::midpoint(vert_b, vert_c);
              stack.append({vert_a, vert_b, mid, vert_d, vert_e, vert_c});
              stack.append({vert_a, mid, vert_c, vert_b, vert_e, vert_f});
              r_total_verts_in++;
              break;
            }
            case 1: {
              const float2 mid = math::midpoint(vert_c, vert_e);
              stack.append({vert_a, vert_b, vert_c, vert_d, mid, vert_f});
              break;
            }
            case 2: {
              const float2 mid = math::midpoint(vert_e, vert_b);
              stack.append({vert_a, vert_b, vert_c, vert_d, mid, vert_f});
              break;
            }
          }
          break;
        }
        case 2: {
          switch (largest_caf_side) {
            case 0: {
              const float2 mid = math::midpoint(vert_c, vert_a);
              stack.append({vert_a, vert_b, mid, vert_d, vert_c, vert_f});
              stack.append({mid, vert_b, vert_c, vert_a, vert_e, vert_f});
              r_total_verts_in++;
              break;
            }
            case 1: {
              const float2 mid = math::midpoint(vert_a, vert_f);
              stack.append({vert_a, vert_b, vert_c, vert_d, vert_e, mid});
              break;
            }
            case 2: {
              const float2 mid = math::midpoint(vert_f, vert_c);
              stack.append({vert_a, vert_b, vert_c, vert_d, vert_e, mid});
              break;
            }
          }
          break;
        }
      }

      continue;
    }

    const float ab_length = math::distance_squared(vert_a, vert_b);
    const float bc_length = math::distance_squared(vert_b, vert_c);
    const float ca_length = math::distance_squared(vert_c, vert_a);
    const float3 triangle_sides(ab_length, bc_length, ca_length);
    const int largest_side = dominant_axis(triangle_sides);
    if (triangle_sides[largest_side] < max_length) {
      continue;
    }

    switch (largest_side) {
      case 0: {
        const float2 mid = math::midpoint(vert_a, vert_b);
        stack.append({vert_a, mid, vert_c, vert_d, vert_b, vert_f});
        stack.append({mid, vert_b, vert_c, vert_d, vert_e, vert_a});
        break;
      }
      case 1: {
        const float2 mid = math::midpoint(vert_b, vert_c);
        stack.append({vert_a, vert_b, mid, vert_d, vert_e, vert_c});
        stack.append({vert_a, mid, vert_c, vert_b, vert_e, vert_f});
        break;
      }
      case 2: {
        const float2 mid = math::midpoint(vert_c, vert_a);
        stack.append({vert_a, vert_b, mid, vert_d, vert_c, vert_f});
        stack.append({mid, vert_b, vert_c, vert_a, vert_e, vert_f});
        break;
      }
    }

    r_total_verts_in++;
  }
}

static void insert_verts_positions(const float2 a_vert,
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
  Vector<std::array<float2, 6>> stack = {{a_vert, b_vert, c_vert, d_vert, e_vert, f_vert}};
  int r_total_verts_in = 0;
  while (!stack.is_empty()) {
    const std::array<float2, 6> triangle_and_neighboards = stack.pop_last();
    const float2 vert_a = triangle_and_neighboards[0];
    const float2 vert_b = triangle_and_neighboards[1];
    const float2 vert_c = triangle_and_neighboards[2];

    const float2 vert_d = triangle_and_neighboards[3];
    const float2 vert_e = triangle_and_neighboards[4];
    const float2 vert_f = triangle_and_neighboards[5];
    // std::cout << "  " << triangle_and_neighboards << ";\n";

    if (len_squared_to_tris({vert_a, vert_b, vert_c}, centre) > radius) {
      const bool a_is_affected = len_squared_to_tris({vert_a, vert_b, vert_d}, centre) < radius;
      const bool b_is_affected = len_squared_to_tris({vert_b, vert_c, vert_e}, centre) < radius;
      const bool c_is_affected = len_squared_to_tris({vert_c, vert_a, vert_f}, centre) < radius;
      if (!(a_is_affected || b_is_affected || c_is_affected)) {
        continue;
      }

      float abd_max = std::numeric_limits<float>::lowest();
      float bce_max = std::numeric_limits<float>::lowest();
      float caf_max = std::numeric_limits<float>::lowest();
      int largest_abd_side = -1;
      int largest_bce_side = -1;
      int largest_caf_side = -1;

      if (a_is_affected) {
        const float ab_length = math::distance_squared(vert_a, vert_b);
        const float bd_length = math::distance_squared(vert_b, vert_d);
        const float da_length = math::distance_squared(vert_d, vert_a);
        const float3 abd(ab_length, bd_length, da_length);
        largest_abd_side = dominant_axis(abd);
        abd_max = abd[largest_abd_side];
      }

      if (b_is_affected) {
        const float bc_length = math::distance_squared(vert_b, vert_c);
        const float ce_length = math::distance_squared(vert_c, vert_e);
        const float eb_length = math::distance_squared(vert_e, vert_b);
        const float3 bce(bc_length, ce_length, eb_length);
        largest_bce_side = dominant_axis(bce);
        bce_max = bce[largest_bce_side];
      }

      if (c_is_affected) {
        const float ca_length = math::distance_squared(vert_c, vert_a);
        const float af_length = math::distance_squared(vert_a, vert_f);
        const float fc_length = math::distance_squared(vert_f, vert_c);
        const float3 caf(ca_length, af_length, fc_length);
        largest_caf_side = dominant_axis(caf);
        caf_max = caf[largest_caf_side];
      }

      const float3 larges_side_in(abd_max, bce_max, caf_max);
      const int larges_side = dominant_axis(larges_side_in);

      if (larges_side_in[larges_side] < max_length) {
        continue;
      }

      switch (larges_side) {
        case 0: {
          switch (largest_abd_side) {
            case 0: {
              const float2 mid = math::midpoint(vert_a, vert_b);
              stack.append({vert_a, mid, vert_c, vert_d, vert_b, vert_f});
              stack.append({mid, vert_b, vert_c, vert_d, vert_e, vert_a});
              r_positions[r_total_verts_in] = mid;
              r_total_verts_in++;
              break;
            }
            case 1: {
              const float2 mid = math::midpoint(vert_d, vert_b);
              stack.append({vert_a, vert_b, vert_c, mid, vert_e, vert_f});
              break;
            }
            case 2: {
              const float2 mid = math::midpoint(vert_a, vert_d);
              stack.append({vert_a, vert_b, vert_c, mid, vert_e, vert_f});
              break;
            }
          }
          break;
        }
        case 1: {
          switch (largest_bce_side) {
            case 0: {
              const float2 mid = math::midpoint(vert_b, vert_c);
              stack.append({vert_a, vert_b, mid, vert_d, vert_e, vert_c});
              stack.append({vert_a, mid, vert_c, vert_b, vert_e, vert_f});
              r_positions[r_total_verts_in] = mid;
              r_total_verts_in++;
              break;
            }
            case 1: {
              const float2 mid = math::midpoint(vert_c, vert_e);
              stack.append({vert_a, vert_b, vert_c, vert_d, mid, vert_f});
              break;
            }
            case 2: {
              const float2 mid = math::midpoint(vert_e, vert_b);
              stack.append({vert_a, vert_b, vert_c, vert_d, mid, vert_f});
              break;
            }
          }
          break;
        }
        case 2: {
          switch (largest_caf_side) {
            case 0: {
              const float2 mid = math::midpoint(vert_c, vert_a);
              stack.append({vert_a, vert_b, mid, vert_d, vert_c, vert_f});
              stack.append({mid, vert_b, vert_c, vert_a, vert_e, vert_f});
              r_positions[r_total_verts_in] = mid;
              r_total_verts_in++;
              break;
            }
            case 1: {
              const float2 mid = math::midpoint(vert_a, vert_f);
              stack.append({vert_a, vert_b, vert_c, vert_d, vert_e, mid});
              break;
            }
            case 2: {
              const float2 mid = math::midpoint(vert_f, vert_c);
              stack.append({vert_a, vert_b, vert_c, vert_d, vert_e, mid});
              break;
            }
          }
          break;
        }
      }

      continue;
    }

    const float ab_length = math::distance_squared(vert_a, vert_b);
    const float bc_length = math::distance_squared(vert_b, vert_c);
    const float ca_length = math::distance_squared(vert_c, vert_a);
    const float3 triangle_sides(ab_length, bc_length, ca_length);
    const int largest_side = dominant_axis(triangle_sides);
    if (triangle_sides[largest_side] < max_length) {
      continue;
    }

    switch (largest_side) {
      case 0: {
        const float2 mid = math::midpoint(vert_a, vert_b);
        stack.append({vert_a, mid, vert_c, vert_d, vert_b, vert_f});
        stack.append({mid, vert_b, vert_c, vert_d, vert_e, vert_a});
        r_positions[r_total_verts_in] = mid;
        break;
      }
      case 1: {
        const float2 mid = math::midpoint(vert_b, vert_c);
        stack.append({vert_a, vert_b, mid, vert_d, vert_e, vert_c});
        stack.append({vert_a, mid, vert_c, vert_b, vert_e, vert_f});
        r_positions[r_total_verts_in] = mid;
        break;
      }
      case 2: {
        const float2 mid = math::midpoint(vert_c, vert_a);
        stack.append({vert_a, vert_b, mid, vert_d, vert_c, vert_f});
        stack.append({mid, vert_b, vert_c, vert_a, vert_e, vert_f});
        r_positions[r_total_verts_in] = mid;
        break;
      }
    }

    r_total_verts_in++;
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

  Array<int> face_total_points(src_mesh.faces_num + 1);

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
    insert_verts_count(projection[a_vert],
                       projection[b_vert],
                       projection[c_vert],
                       projection[d_vert],
                       projection[e_vert],
                       projection[f_vert],
                       centre,
                       squared_radius,
                       max_length,
                       total_verts_in);
    face_total_points[face_i] = total_verts_in;
  }

  std::cout << ">> " << face_total_points.as_span() << ";\n";

  const OffsetIndices<int> subdive_verts = offset_indices::accumulate_counts_to_offsets(
      face_total_points);

  Mesh *dst_mesh = BKE_mesh_new_nomain(subdive_verts.total_size(), 0, 0, 0);
  MutableSpan<float3> positions = dst_mesh->vert_positions_for_write();

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

    IndexRange points_range = subdive_verts[face_i];
    Array<float2> uv_positions(points_range.size());

    insert_verts_positions(projection[a_vert],
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
