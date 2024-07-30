/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_offset_indices.hh"
#include "BLI_ordered_edge.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

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

namespace topo_set {

static bool part_of(const int3 all, const int part)
{
  return (part == all.a || part == all.b || part == all.c);
}

static bool equals(const int3 left, const int3 right)
{
  return part_of(left, right.a) && part_of(left, right.b) && part_of(left, right.b);
}

static bool part_of(const int3 all, const int2 part)
{
  return part_of(all, part.a) && part_of(all, part.b);
}

static bool part_of(const int2 all, const int part)
{
  return part == all.a || part == all.b;
}

static bool equals(const int2 left, const int2 right)
{
  return part_of(left, right.a) && part_of(left, right.b);
}

static int index_of(const int3 all, const int part)
{
  BLI_assert(part_of(all, part));
  return int(all.b == part) * 1 + int(all.c == part) * 2;
}

static int index_of(const int2 all, const int part)
{
  BLI_assert(part_of(all, part));
  return int(all.b == part) * 1;
}

static int unordered_index_of(const int3 all, const int2 part)
{
  BLI_assert(part_of(all, part));
  return int(equals(int2{all.b, all.c}, part)) * 1 + int(equals(int2{all.c, all.a}, part)) * 2;
}

static int diff(const int3 all, const int2 part)
{
  BLI_assert(part_of(all, part));
  BLI_assert(all.a >= 0 && all.b >= 0 && all.c >= 0);
  return (all.a - part.a) + (all.b - part.b) + all.c;
}

static int diff(const int2 all, const int part)
{
  BLI_assert(part_of(all, part));
  BLI_assert(all.a >= 0 && all.b >= 0);
  return (all.a - part) + all.b;
}

static int2 sample(const int3 all, const int2 indices)
{
  return int2{all[indices.a], all[indices.b]};
}

static const std::array<int2, 3> edges{int2{0, 1}, int2{1, 2}, int2{2, 0}};
static const int3 face{0, 1, 2};

static const std::array<int, 3> shift_front{1, 2, 0};
static const std::array<int, 3> shift_back{2, 0, 1};

constexpr int vert_a = 0;
constexpr int vert_b = 1;
constexpr int vert_c = 2;

constexpr int edge_ab = 0;
constexpr int edge_bc = 1;
constexpr int edge_ca = 2;

}  // namespace topo_set

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

static float3 bary_weight_for_tris_point(const float2 a,
                                         const float2 b,
                                         const float2 c,
                                         const float2 p)
{
  float3 result;
  barycentric_weights_v2(a, b, c, p, result);
  return result;
}

static bool triangle_is_in_range(
    const float2 &a, const float2 &b, const float2 &c, const float2 &centre, const float range)
{
  return len_squared_to_tris({a, b, c}, centre) < range;
}

namespace FaceVerts {
static constexpr const int a = 0;
static constexpr const int b = 1;
static constexpr const int c = 2;
static const int3 abc(0, 1, 2);
}  // namespace FaceVerts

namespace EdgeState {
static constexpr const int8_t ab_is_real_edge = 1 << 0;
static constexpr const int8_t bc_is_real_edge = 1 << 1;
static constexpr const int8_t ca_is_real_edge = 1 << 2;

static constexpr const int8_t ab_is_owned_edge = 1 << 3;
static constexpr const int8_t bc_is_owned_edge = 1 << 4;
static constexpr const int8_t ca_is_owned_edge = 1 << 5;

static const std::array<int8_t, 3> edges_real = {
    ab_is_real_edge, bc_is_real_edge, ca_is_real_edge};
static const std::array<int8_t, 3> edges_owned = {
    ab_is_owned_edge, bc_is_owned_edge, ca_is_owned_edge};
}  // namespace EdgeState

static int dominant_axis(const float3 a)
{
  return ((a.x > a.y) ? ((a.x > a.z) ? 0 : 2) : ((a.y > a.z) ? 1 : 2));
}

static void connected_tris_ensure_short_edges(const float3 a_point_3d,
                                              const float3 b_point_3d,
                                              const float2 a_point_2d,
                                              const float2 b_point_2d,
                                              const float max_length_squared,
                                              MutableSpan<float3> points_3d,
                                              MutableSpan<float2> points_2d)
{
  BLI_assert(points_3d.size() == points_2d.size());

  for (const int point_i : points_2d.index_range()) {
    while (true) {
      const float ac_length_squared = math::distance_squared(a_point_3d, points_3d[point_i]);
      const float bc_length_squared = math::distance_squared(b_point_3d, points_3d[point_i]);
      if (math::max(ac_length_squared, bc_length_squared) < max_length_squared) {
        break;
      }
      /* TODO. */
      BLI_assert(ac_length_squared != bc_length_squared);
      if (ac_length_squared < bc_length_squared) {
        points_3d[point_i] = math::midpoint(b_point_3d, points_3d[point_i]);
        points_2d[point_i] = math::midpoint(b_point_2d, points_2d[point_i]);
      }
      else {
        points_3d[point_i] = math::midpoint(a_point_3d, points_3d[point_i]);
        points_2d[point_i] = math::midpoint(a_point_2d, points_2d[point_i]);
      }
    }
  }
}

static void connected_tris_drop_short_edges(const float3 a_point_3d,
                                            const float3 b_point_3d,
                                            const float max_length_squared,
                                            Vector<float3> &r_points_3d,
                                            Vector<float2> &r_points_2d)
{
  BLI_assert(points_3d.size() == points_2d.size());

  const IndexRange range = points_2d.index_range();
  for (const int point_i : range) {
    const int r_point_i = range.last(point_i);
    const float ac_length_squared = math::distance_squared(a_point_3d, points_3d[r_point_i]);
    const float bc_length_squared = math::distance_squared(b_point_3d, points_3d[r_point_i]);
    if (math::max(ac_length_squared, bc_length_squared) < max_length_squared) {
      r_points_3d.remove_and_reorder(r_point_i);
      r_points_2d.remove_and_reorder(r_point_i);
    }
  }
}

template<typename VertFunc, typename FaceFunc>
static void face_subdivide(std::array<Vector<float2>, 3> connected_2d_points,
                           std::array<Vector<float3>, 3> connected_3d_points,
                           const std::array<float2, 3> tri_2d_points,
                           const std::array<float3, 3> tri_3d_points,
                           const float2 centre,
                           const float radius,
                           const float max_length,
                           const int3 tri_verts,
                           VertFunc vert_func,
                           FaceFunc face_func)
{
  Vector<std::array<Vector<float2>, 3>> connected_2d_points_stack = {
      std::move(connected_2d_points)};
  Vector<std::array<Vector<float3>, 3>> connected_3d_points_stack = {
      std::move(connected_3d_points)};
  Vector<std::array<float2, 3>> tri_2d_points_stack = {tri_2d_points};
  Vector<std::array<float3, 3>> tri_3d_points_stack = {tri_3d_points};

  Vector<int3> tri_verts_stack = {tri_verts};

  VectorSet<OrderedEdge> split_of_edges;
  VectorSet<OrderedEdge> split_in_face;

  constexpr int8_t edges_are_real = EdgeState::ab_is_real_edge | EdgeState::bc_is_real_edge |
                                    EdgeState::ca_is_real_edge;
  constexpr int8_t edges_are_owned = EdgeState::ab_is_owned_edge | EdgeState::bc_is_owned_edge |
                                     EdgeState::ca_is_owned_edge;
  Vector<int8_t> edges_states = {edges_are_real | (~edges_are_owned)};

  while (!edges_states.is_empty()) {
    std::array < Vector<float2>, 3 >> connected_2d_points = connected_2d_points_stack.pop_last();
    std::array < Vector<float3>, 3 >> connected_3d_points = connected_3d_points_stack.pop_last();
    std::array < float2, 3 >> tri_2d_points = tri_2d_points_stack.pop_last();
    std::array < float3, 3 >> tri_3d_points = tri_3d_points_stack.pop_last();

    const int3 tri_verts = tri_verts_stack.pop_last();

    const int8_t edges_state = edges_states.pop_last();

    std::array<float, 3> lengths_squared;
    lengths_squared[0] = math::distance_squared(tri_3d_points[0], tri_3d_points[1]);
    lengths_squared[1] = math::distance_squared(tri_3d_points[1], tri_3d_points[2]);
    lengths_squared[2] = math::distance_squared(tri_3d_points[2], tri_3d_points[0]);
    /* TODO. */
    BLI_assert(!(ELEM(lengths_squared[0], lengths_squared[1], lengths_squared[2]) ||
                 ELEM(lengths_squared[1], lengths_squared[0], lengths_squared[2])));
    const int largest_side_to_split = dominant_axis(
        {lengths_squared[0], lengths_squared[1], lengths_squared[2]});
    if (lengths_squared[largest_side_to_split] <= max_length) {
      face_func(tri_verts);
      continue;
    }

    connected_tris_ensure_short_edges(tri_3d_points[0],
                                      tri_3d_points[1],
                                      tri_2d_points[0],
                                      tri_2d_points[1],
                                      lengths_squared[largest_side_to_split],
                                      connected_3d_points[0],
                                      connected_2d_points[0]);
    connected_tris_ensure_short_edges(tri_3d_points[1],
                                      tri_3d_points[2],
                                      tri_2d_points[1],
                                      tri_2d_points[2],
                                      lengths_squared[largest_side_to_split],
                                      connected_3d_points[1],
                                      connected_2d_points[1]);
    connected_tris_ensure_short_edges(tri_3d_points[2],
                                      tri_3d_points[0],
                                      tri_2d_points[2],
                                      tri_2d_points[0],
                                      lengths_squared[largest_side_to_split],
                                      connected_3d_points[2],
                                      connected_2d_points[2]);

    connected_tris_drop_short_edges(tri_3d_points[0],
                                    tri_3d_points[1],
                                    max_length,
                                    connected_3d_points[0],
                                    connected_2d_points[0]);
    connected_tris_drop_short_edges(tri_3d_points[1],
                                    tri_3d_points[2],
                                    max_length,
                                    connected_3d_points[1],
                                    connected_2d_points[1]);
    connected_tris_drop_short_edges(tri_3d_points[2],
                                    tri_3d_points[0],
                                    max_length,
                                    connected_3d_points[2],
                                    connected_2d_points[2]);

    if (UNLIKELY(!triangle_is_in_range(
            tri_2d_points[0], tri_2d_points[1], tri_2d_points[2], centre, radius)))
    {
      const float2 a_point = tri_2d_points[largest_side_to_split];
      const float2 b_point = tri_2d_points[topo_set::shift_front[largest_side_to_split]];
      const Span<float2> side_points = connected_2d_points[largest_side_to_split];
      const bool has_connection_to_split = std::any_of(
          side_points.begin(), side_points.end(), [&](const float2 point) {
            return triangle_is_in_range(a_point, b_point, point, centre, radius);
          });
      if (!has_connection_to_split) {
        face_func(tri_verts);
        continue;
      }
    }

    const int2 split_edge = topo_set::edges[largest_side_to_split];

    const int2 face_verts_to_split = topo_set::sample(tri_verts, split_edge);
    const int new_vert_i = [&]() {
      if (edges_state & EdgeState::edges_real(largest_side_to_split)) {
        return split_in_face.index_of_or_add(face_verts_to_split);
      }
      return std::numeric_limits<int>::min() + split_of_edges.index_of_or_add(face_verts_to_split);
    }();

    const float2 mid_2d = math::midpoint(tri_2d_points[split_edge[0]],
                                         tri_2d_points[split_edge[1]]);
    const float3 mid_3d = math::midpoint(tri_3d_points[split_edge[0]],
                                         tri_3d_points[split_edge[1]]);

    vert_func(mid_2d, mid_3d, edges_state, new_vert_i);

    std::array < Vector<float2>, 3 >> left_connected_2d_points;
    std::array < Vector<float3>, 3 >> left_connected_3d_points;
    std::array < float2, 3 >> left_tri_2d_points = tri_2d_points;
    std::array < float3, 3 >> left_tri_3d_points = tri_3d_points;

    left_connected_2d_points[largest_side_to_split] = connected_2d_points[largest_side_to_split];
    left_connected_3d_points[largest_side_to_split] = connected_3d_points[largest_side_to_split];

    left_connected_2d_points[topo_set::shift_front[largest_side_to_split]] = {
        tri_2d_points[topo_set::shift_front[largest_side_to_split]]};
    left_connected_3d_points[topo_set::shift_front[largest_side_to_split]] = {
        tri_3d_points[topo_set::shift_front[largest_side_to_split]]};

    left_connected_2d_points[topo_set::shift_back[largest_side_to_split]] = std::move(
        connected_2d_points[topo_set::shift_back[largest_side_to_split]]);
    left_connected_3d_points[topo_set::shift_back[largest_side_to_split]] = std::move(
        connected_3d_points[topo_set::shift_back[largest_side_to_split]]);

    left_tri_2d_points[topo_set::shift_front[largest_side_to_split]] = mid_2d;
    left_tri_3d_points[topo_set::shift_front[largest_side_to_split]] = mid_3d;

    const int8_t left_edges_state =
        edges_state & (~EdgeState::edges_real[topo_set::shift_front[largest_side_to_split]]) |
        EdgeState::edges_owned[topo_set::shift_front[largest_side_to_split]];

    int3 left_tri_verts = tri_verts;
    left_tri_verts[topo_set::shift_front[largest_side_to_split]] = new_vert_i;

    std::array < Vector<float2>, 3 >> right_connected_2d_points;
    std::array < Vector<float3>, 3 >> right_connected_3d_points;
    std::array < float2, 3 >> right_tri_2d_points = tri_2d_points;
    std::array < float3, 3 >> right_tri_3d_points = tri_3d_points;

    right_connected_2d_points[largest_side_to_split] = std::move(
        connected_2d_points[largest_side_to_split]);
    right_connected_3d_points[largest_side_to_split] = std::move(
        connected_3d_points[largest_side_to_split]);

    right_connected_2d_points[topo_set::shift_front[largest_side_to_split]] = std::move(
        connected_2d_points[topo_set::shift_front[largest_side_to_split]]);
    right_connected_3d_points[topo_set::shift_front[largest_side_to_split]] = std::move(
        connected_3d_points[topo_set::shift_front[largest_side_to_split]]);

    right_connected_2d_points[topo_set::shift_back[largest_side_to_split]] = {
        tri_2d_points[largest_side_to_split]};
    right_connected_3d_points[topo_set::shift_back[largest_side_to_split]] = {
        tri_3d_points[largest_side_to_split]};

    right_tri_2d_points[largest_side_to_split] = mid_2d;
    right_tri_3d_points[largest_side_to_split] = mid_3d;

    const int8_t right_edges_state =
        edges_state & (~EdgeState::edges_real[topo_set::shift_back[largest_side_to_split]]);

    int3 right_tri_verts = tri_verts;
    right_tri_verts[largest_side_to_split] = new_vert_i;

    connected_2d_points_stack.append(std::move(left_connected_2d_points));
    connected_3d_points_stack.append(std::move(left_connected_3d_points));
    tri_2d_points_stack.append(std::move(left_tri_2d_points));
    tri_3d_points_stack.append(std::move(left_tri_3d_points));
    edges_states.append(left_edges_state);
    tri_verts_stack.append(left_tri_verts);

    connected_2d_points_stack.append(std::move(right_connected_2d_points));
    connected_3d_points_stack.append(std::move(right_connected_3d_points));
    tri_2d_points_stack.append(std::move(right_tri_2d_points));
    tri_3d_points_stack.append(std::move(right_tri_3d_points));
    edges_states.append(right_edges_state);
    tri_verts_stack.append(right_tri_verts);
  }
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
    // std::cout << "  " << edge_and_tris << ";\n";

    const float ab_length = math::distance_squared(edge_and_tris[0], edge_and_tris[1]);
    if (ab_length < max_length) {
      continue;
    }
    // std::cout << "  is enought large" << ";\n";

    const bool left_is_affected = len_squared_to_tris(
                                      {edge_and_tris[0], edge_and_tris[1], edge_and_tris[2]},
                                      centre) <= radius;
    const bool right_is_affected = len_squared_to_tris(
                                       {edge_and_tris[0], edge_and_tris[1], edge_and_tris[3]},
                                       centre) <= radius;
    if (!(left_is_affected || right_is_affected)) {
      continue;
    }
    // std::cout << "  is_affected" << ";\n";

    const float ac_length = math::distance_squared(edge_and_tris[0], edge_and_tris[2]);
    const float bc_length = math::distance_squared(edge_and_tris[1], edge_and_tris[2]);
    const float abc_max = math::max(ac_length, bc_length);

    const float ad_length = math::distance_squared(edge_and_tris[0], edge_and_tris[3]);
    const float bd_length = math::distance_squared(edge_and_tris[1], edge_and_tris[3]);
    const float abd_max = math::max(ad_length, bd_length);

    if (math::max(abd_max, abc_max) < max_length) {
      continue;
    }
    // std::cout << "  Is in range" << ";\n";

    if (ab_length > abc_max && ab_length > abd_max) {
      const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[1]);
      stack.append({edge_and_tris[0], mid, edge_and_tris[2], edge_and_tris[3]});
      stack.append({mid, edge_and_tris[1], edge_and_tris[2], edge_and_tris[3]});
      r_total_verts_in++;
      // std::cout << "  Is AB Split" << ";\n";
      continue;
    }

    if (abc_max > abd_max) {
      // std::cout << "  Is ABC split" << ";\n";
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
      // std::cout << "  Is ABD split" << ";\n";
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

static void edge_subdivide_uv(const float2 a_vert,
                              const float2 b_vert,
                              const float2 c_vert,
                              const float2 d_vert,
                              const float2 centre,
                              const float radius,
                              const float max_length,
                              MutableSpan<float> r_factors)
{
  int r_total_verts_in = 0;
  // std::cout << "Face Size:\n";
  Vector<float2> factor_range_stack = {{0.0f, 1.0f}};
  Vector<std::array<float2, 4>> stack = {{a_vert, b_vert, c_vert, d_vert}};
  while (!stack.is_empty()) {
    BLI_assert(factor_range_stack.size() == stack.size());
    const std::array<float2, 4> edge_and_tris = stack.pop_last();
    const float2 edge_factor = factor_range_stack.pop_last();
    // std::cout << "  " << edge_and_tris << ";\n";

    const float ab_length = math::distance_squared(edge_and_tris[0], edge_and_tris[1]);
    if (ab_length < max_length) {
      continue;
    }
    // std::cout << "  is enought large" << ";\n";

    const bool left_is_affected = len_squared_to_tris(
                                      {edge_and_tris[0], edge_and_tris[1], edge_and_tris[2]},
                                      centre) <= radius;
    const bool right_is_affected = len_squared_to_tris(
                                       {edge_and_tris[0], edge_and_tris[1], edge_and_tris[3]},
                                       centre) <= radius;
    if (!(left_is_affected || right_is_affected)) {
      continue;
    }
    // std::cout << "  is_affected" << ";\n";

    const float ac_length = math::distance_squared(edge_and_tris[0], edge_and_tris[2]);
    const float bc_length = math::distance_squared(edge_and_tris[1], edge_and_tris[2]);
    const float abc_max = math::max(ac_length, bc_length);

    const float ad_length = math::distance_squared(edge_and_tris[0], edge_and_tris[3]);
    const float bd_length = math::distance_squared(edge_and_tris[1], edge_and_tris[3]);
    const float abd_max = math::max(ad_length, bd_length);

    if (math::max(abc_max, abd_max) < max_length) {
      continue;
    }
    // std::cout << "  Is in range" << ";\n";

    if (ab_length > abc_max && ab_length > abd_max) {
      const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[1]);
      stack.append({edge_and_tris[0], mid, edge_and_tris[2], edge_and_tris[3]});
      stack.append({mid, edge_and_tris[1], edge_and_tris[2], edge_and_tris[3]});
      const float factor_mid = math::midpoint(edge_factor[0], edge_factor[1]);
      factor_range_stack.append({edge_factor[0], factor_mid});
      factor_range_stack.append({factor_mid, edge_factor[1]});
      r_factors[r_total_verts_in] = factor_mid;
      r_total_verts_in++;
      // std::cout << "  Is AB Split" << ";\n";
      continue;
    }

    if (abc_max > abd_max) {
      // std::cout << "  Is ABC split" << ";\n";
      if (ac_length > bc_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
        factor_range_stack.append(edge_factor);
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[2]);
        stack.append({edge_and_tris[0], edge_and_tris[1], mid, edge_and_tris[3]});
        factor_range_stack.append(edge_factor);
      }
    }
    else {
      // std::cout << "  Is ABD split" << ";\n";
      if (ad_length > bd_length) {
        const float2 mid = math::midpoint(edge_and_tris[0], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
        factor_range_stack.append(edge_factor);
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris[1], edge_and_tris[3]);
        stack.append({edge_and_tris[0], edge_and_tris[1], edge_and_tris[2], mid});
        factor_range_stack.append(edge_factor);
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
  BLI_assert(!verts_range.is_empty());

  int r_total_verts_in = 0;
  int edge_iter = 0;
  Vector<TrisEdge> stack = {{a_vert, b_vert, c_vert, d_vert, edge[0], edge[1]}};
  while (!stack.is_empty()) {
    const TrisEdge edge_and_tris = stack.pop_last();
    // std::cout << "  " << edge_and_tris << ";\n";

    const float ab_length = math::distance_squared(edge_and_tris.a, edge_and_tris.b);
    if (ab_length < max_length) {
      r_verts[edge_iter] = int2(edge_and_tris.vert_a, edge_and_tris.vert_b);
      edge_iter++;
      continue;
    }
    // std::cout << "  is enought large" << ";\n";

    const bool left_is_affected = len_squared_to_tris(
                                      {edge_and_tris.a, edge_and_tris.b, edge_and_tris.c},
                                      centre) <= radius;
    const bool right_is_affected = len_squared_to_tris(
                                       {edge_and_tris.a, edge_and_tris.b, edge_and_tris.d},
                                       centre) <= radius;
    if (!(left_is_affected || right_is_affected)) {
      r_verts[edge_iter] = int2(edge_and_tris.vert_a, edge_and_tris.vert_b);
      edge_iter++;
      continue;
    }
    // std::cout << "  is_affected" << ";\n";

    const float ac_length = math::distance_squared(edge_and_tris.a, edge_and_tris.c);
    const float bc_length = math::distance_squared(edge_and_tris.b, edge_and_tris.c);
    const float abc_max = math::max(ac_length, bc_length);

    const float ad_length = math::distance_squared(edge_and_tris.a, edge_and_tris.d);
    const float bd_length = math::distance_squared(edge_and_tris.b, edge_and_tris.d);
    const float abd_max = math::max(ad_length, bd_length);

    if (math::max(abc_max, abd_max) < max_length) {
      r_verts[edge_iter] = int2(edge_and_tris.vert_a, edge_and_tris.vert_b);
      edge_iter++;
      continue;
    }
    // std::cout << "  Is in range" << ";\n";

    if (ab_length > abc_max && ab_length > abd_max) {
      const float2 mid = math::midpoint(edge_and_tris.a, edge_and_tris.b);
      const int mid_index = verts_range[r_total_verts_in];
      r_total_verts_in++;
      stack.append({edge_and_tris.a,
                    mid,
                    edge_and_tris.c,
                    edge_and_tris.d,
                    edge_and_tris.vert_a,
                    mid_index});
      stack.append({mid,
                    edge_and_tris.b,
                    edge_and_tris.c,
                    edge_and_tris.d,
                    mid_index,
                    edge_and_tris.vert_b});
      // std::cout << "  Is AB Split" << ";\n";
      continue;
    }

    if (abc_max > abd_max) {
      // std::cout << "  Is ABC split" << ";\n";
      if (ac_length > bc_length) {
        const float2 mid = math::midpoint(edge_and_tris.a, edge_and_tris.c);
        stack.append({edge_and_tris.a,
                      edge_and_tris.b,
                      mid,
                      edge_and_tris.d,
                      edge_and_tris.vert_a,
                      edge_and_tris.vert_b});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris.b, edge_and_tris.c);
        stack.append({edge_and_tris.a,
                      edge_and_tris.b,
                      mid,
                      edge_and_tris.d,
                      edge_and_tris.vert_a,
                      edge_and_tris.vert_b});
      }
    }
    else {
      // std::cout << "  Is ABD split" << ";\n";
      if (ad_length > bd_length) {
        const float2 mid = math::midpoint(edge_and_tris.a, edge_and_tris.d);
        stack.append({edge_and_tris.a,
                      edge_and_tris.b,
                      edge_and_tris.c,
                      mid,
                      edge_and_tris.vert_a,
                      edge_and_tris.vert_b});
      }
      else {
        const float2 mid = math::midpoint(edge_and_tris.b, edge_and_tris.d);
        stack.append({edge_and_tris.a,
                      edge_and_tris.b,
                      edge_and_tris.c,
                      mid,
                      edge_and_tris.vert_a,
                      edge_and_tris.vert_b});
      }
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
  const float squared_max_length = max_length * max_length;

  const OffsetIndices<int> faces = src_mesh.faces();
  const Span<int2> edges = src_mesh.edges();
  const Span<int> corner_edges = src_mesh.corner_edges();
  const Span<int> corner_verts = src_mesh.corner_verts();

  const Span<float3> src_positions = src_mesh.vert_positions();

  Array<int> edge_to_face_offset_data;
  Array<int> edge_to_face_indices;
  const GroupedSpan<int> edge_to_face_map = bke::mesh::build_edge_to_face_map(
      faces, corner_edges, src_mesh.edges_num, edge_to_face_offset_data, edge_to_face_indices);

  Array<int> edge_total_verts(src_mesh.edges_num + 1);
  Array<int> face_total_verts(src_mesh.faces_num + 1);
  Array<int> face_total_edges(src_mesh.faces_num + 1);

  for (const int edge_i : IndexRange(src_mesh.edges_num)) {
    // const int2 edge = edges[edge_i];

    // const Span<int> edge_faces = edge_to_face_map[edge_i];

    //  const int3 left_verts = gather_tri(faces[edge_faces[0]], corner_verts);
    // const int3 right_verts = gather_tri(faces[edge_faces[1]], corner_verts);

    // BLI_assert(elem(left_verts, edge[0]) && elem(left_verts, edge[1]));
    // BLI_assert(elem(right_verts, edge[0]) && elem(right_verts, edge[1]));

    // const int a_vert = exclusive_one(left_verts, edge);
    // const int b_vert = exclusive_one(right_verts, edge);

    // std::cout << "Edge: " << edge_i << ";\n";

    int total_verts_in = 0; /*
     edge_subdivide_count(projection[edge[0]],
                          projection[edge[1]],
                          projection[a_vert],
                          projection[b_vert],
                          centre,
                          squared_radius,
                          squared_max_length,
                          total_verts_in);*/
    edge_total_verts[edge_i] = total_verts_in;
  }

  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    const int3 face_edges = gather_tri(face, corner_edges);
    const int3 face_verts = gather_tri(face, corner_verts);

    const int2 edge_ab = edges[face_edges[0]];
    const int2 edge_bc = edges[face_edges[1]];
    const int2 edge_ca = edges[face_edges[2]];

    BLI_assert(OrderedEdge(face_verts[0], face_verts[1]) == OrderedEdge(edge_ab));
    BLI_assert(OrderedEdge(face_verts[1], face_verts[2]) == OrderedEdge(edge_bc));
    BLI_assert(OrderedEdge(face_verts[2], face_verts[0]) == OrderedEdge(edge_ca));

    const Span<int> ab_faces = edge_to_face_map[face_edges[0]];
    const Span<int> bc_faces = edge_to_face_map[face_edges[1]];
    const Span<int> ca_faces = edge_to_face_map[face_edges[2]];

    std::array<int, 5> vert_offsets;
    vert_offsets[0] = 3;
    vert_offsets[1] = ab_faces.size() - 1;
    vert_offsets[2] = bc_faces.size() - 1;
    vert_offsets[3] = ca_faces.size() - 1;
    offset_indices::accumulate_counts_to_offsets(vert_offsets);

    Vector<float2> uv_verts;
    uv_verts.append(projection[face_verts[0]]);
    uv_verts.append(projection[face_verts[1]]);
    uv_verts.append(projection[face_verts[2]]);
    Vector<float3> verts;
    verts.append(src_positions[face_verts[0]]);
    verts.append(src_positions[face_verts[1]]);
    verts.append(src_positions[face_verts[2]]);
    for (const int other_face_i : ab_faces) {
      if (other_face_i != face_i) {
        const IndexRange other_face = faces[other_face_i];
        const int3 other_face_verts = gather_tri(other_face, corner_verts);
        const int d_vert = exclusive_one(other_face_verts, edge_ab);
        BLI_assert(!elem(face_verts, d_vert));
        uv_verts.append(projection[d_vert]);
        verts.append(src_positions[d_vert]);
      }
    }
    for (const int other_face_i : bc_faces) {
      if (other_face_i != face_i) {
        const IndexRange other_face = faces[other_face_i];
        const int3 other_face_verts = gather_tri(other_face, corner_verts);
        const int e_vert = exclusive_one(other_face_verts, edge_bc);
        BLI_assert(!elem(face_verts, e_vert));
        uv_verts.append(projection[e_vert]);
        verts.append(src_positions[e_vert]);
      }
    }
    for (const int other_face_i : ca_faces) {
      if (other_face_i != face_i) {
        const IndexRange other_face = faces[other_face_i];
        const int3 other_face_verts = gather_tri(other_face, corner_verts);
        const int f_vert = exclusive_one(other_face_verts, edge_ca);
        BLI_assert(!elem(face_verts, f_vert));
        uv_verts.append(projection[f_vert]);
        verts.append(src_positions[f_vert]);
      }
    }

    int total_verts_in = 0;
    int total_edges_in = 0;
    int total_faces_in = 0;

    VectorSet<OrderedEdge> face_edges_set;
    VectorSet<OrderedEdge> result_face_edges_set;
    face_subdivide(
        vert_offsets,
        uv_verts,
        verts,
        centre,
        squared_radius,
        squared_max_length,
        face_verts,
        [&](const float2 pos,
            const float3 real_pos,
            const bool is_edge_vert,
            const bool is_this_tris_vert) {
          total_edges_in++;
          std::cout << "edge++\n";
          if ((!is_edge_vert) && (!is_this_tris_vert)) {
            total_verts_in++;
            std::cout << "vert++\n";
            total_edges_in++;
            std::cout << "edge++\n";
          }
        },
        [&](const int3 verts_of) { total_faces_in++; },
        face_edges_set,
        result_face_edges_set);

    std::cout << " Result:\n";
    std::cout << "\t total_verts_in: " << total_verts_in << ";\n";
    std::cout << "\t total_edges_in: " << total_edges_in << ";\n";
    std::cout << "\t total_faces_in: " << total_faces_in << ";\n";

    face_total_verts[face_i] = 0;
    face_total_edges[face_i] = 0;
  }

  // std::cout << ">> Edges: " << edge_total_verts.as_span() << ";\n";
  // std::cout << std::endl;
  // std::cout << ">> Faces: " << face_total_verts.as_span() << ";\n";

  IndexMaskMemory memory;
  const IndexMask changed_edges = IndexMask::from_predicate(
      IndexMask(src_mesh.edges_num), GrainSize(8192), memory, [&](const int i) {
        return edge_total_verts[i] > 0;
      });

  const IndexMask changed_faces = IndexMask::from_predicate(
      IndexMask(src_mesh.faces_num), GrainSize(8192), memory, [&](const int i) {
        return face_total_edges[i] > 0;
      });

  const IndexMask keeped_edges = changed_edges.complement(IndexMask(src_mesh.edges_num), memory);
  const IndexMask keeped_faces = changed_faces.complement(IndexMask(src_mesh.edges_num), memory);

  const OffsetIndices<int> subdive_edge_verts = offset_indices::accumulate_counts_to_offsets(
      edge_total_verts);
  const OffsetIndices<int> subdive_face_verts = offset_indices::accumulate_counts_to_offsets(
      face_total_verts);
  const OffsetIndices<int> subdive_face_edges = offset_indices::accumulate_counts_to_offsets(
      face_total_edges);

  const IndexRange verts_range(src_mesh.verts_num);
  const IndexRange edges_verts_range = verts_range.after(subdive_edge_verts.total_size());
  const IndexRange faces_verts_range = edges_verts_range.after(subdive_face_verts.total_size());

  const IndexRange edges_range(keeped_edges.size());
  const IndexRange edges_edges_range = edges_range.after(changed_edges.size() +
                                                         subdive_edge_verts.total_size());
  const IndexRange faces_edges_range = edges_edges_range.after(subdive_face_edges.total_size());

  const int total_verts = faces_verts_range.one_after_last();
  const int total_edges = faces_edges_range.one_after_last();

  Mesh *dst_mesh = BKE_mesh_new_nomain(total_verts, total_edges, 0, 0);
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  MutableSpan<float3> dst_positions = dst_mesh->vert_positions_for_write();
  MutableSpan<int2> dst_edges = dst_mesh->edges_for_write();
  dst_edges.fill({0, 1});

  // bke::gather_attributes(src_attributes, bke::AttrDomain::Point, {}, {}, verts_range,
  // dst_attributes);
  dst_positions.take_front(src_mesh.verts_num).copy_from(src_positions);

  keeped_edges.foreach_index_optimized<int>(GrainSize(8092), [&](const int i, const int pos) {
    dst_edges[edge_total_verts[i] + i] = edges[i];
  });

  Array<float> edge_vertices_factor_weight(edges_verts_range.size());
  Array<float3> face_vertices_bary_weight(faces_verts_range.size());

  changed_edges.foreach_index([&](const int edge_i) {
    const int2 edge = edges[edge_i];

    const Span<int> edge_faces = edge_to_face_map[edge_i];

    const int3 left_verts = gather_tri(faces[edge_faces[0]], corner_verts);
    const int3 right_verts = gather_tri(faces[edge_faces[1]], corner_verts);

    BLI_assert(elem(left_verts, edge[0]) && elem(left_verts, edge[1]));
    BLI_assert(elem(right_verts, edge[0]) && elem(right_verts, edge[1]));

    const int a_vert = exclusive_one(left_verts, edge);
    const int b_vert = exclusive_one(right_verts, edge);

    // std::cout << "Edge: " << edge_i << ";\n";
    edge_subdivide_uv(
        projection[edge[0]],
        projection[edge[1]],
        projection[a_vert],
        projection[b_vert],
        centre,
        squared_radius,
        squared_max_length,
        edge_vertices_factor_weight.as_mutable_span().slice(subdive_edge_verts[edge_i]));
  });

  changed_edges.foreach_index([&](const int edge_i) {
    const int2 edge = edges[edge_i];
    const Span<int> edge_faces = edge_to_face_map[edge_i];

    const int3 left_verts = gather_tri(faces[edge_faces[0]], corner_verts);
    const int3 right_verts = gather_tri(faces[edge_faces[1]], corner_verts);

    BLI_assert(elem(left_verts, edge[0]) && elem(left_verts, edge[1]));
    BLI_assert(elem(right_verts, edge[0]) && elem(right_verts, edge[1]));

    const int a_vert = exclusive_one(left_verts, edge);
    const int b_vert = exclusive_one(right_verts, edge);

    const IndexRange edge_verts_range = subdive_edge_verts[edge_i];
    const IndexRange edge_verts_indices_range = edge_verts_range.shift(edges_verts_range.start());
    const IndexRange edge_edges_range = IndexRange::from_begin_size(
        edge_verts_range.start() + edge_i, edge_verts_range.size() + 1);

    edge_subdivide_verts(projection[edge[0]],
                         projection[edge[1]],
                         projection[a_vert],
                         projection[b_vert],
                         centre,
                         squared_radius,
                         squared_max_length,
                         edge,
                         edge_verts_indices_range,
                         dst_edges.slice(edge_edges_range));
  });

  changed_faces.foreach_index([&](const int face_i) {
    const IndexRange face = faces[face_i];
    const int3 face_edges = gather_tri(face, corner_edges);
    const int3 face_verts = gather_tri(face, corner_verts);

    const int2 edge_ab = edges[face_edges[0]];
    const int2 edge_bc = edges[face_edges[1]];
    const int2 edge_ca = edges[face_edges[2]];

    const Span<int> ab_faces = edge_to_face_map[face_edges[0]];
    const Span<int> bc_faces = edge_to_face_map[face_edges[1]];
    const Span<int> ca_faces = edge_to_face_map[face_edges[2]];

    const int face_ab = ab_faces[0] == face_i ? ab_faces[1] : ab_faces[0];
    const int face_bc = bc_faces[0] == face_i ? bc_faces[1] : bc_faces[0];
    const int face_ca = ca_faces[0] == face_i ? ca_faces[1] : ca_faces[0];

    const int3 abd_verts = gather_tri(faces[face_ab], corner_verts);
    const int3 bce_verts = gather_tri(faces[face_bc], corner_verts);
    const int3 caf_verts = gather_tri(faces[face_ca], corner_verts);

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

    const IndexRange verts_range = subdive_face_verts[face_i].shift(faces_verts_range.start());
    const IndexRange edges_range = subdive_face_edges[face_i].shift(faces_edges_range.start());

    const IndexRange ab_points_range = subdive_edge_verts[face_edges[0]].shift(
        edges_verts_range.start());
    const IndexRange bc_points_range = subdive_edge_verts[face_edges[1]].shift(
        edges_verts_range.start());
    const IndexRange ca_points_range = subdive_edge_verts[face_edges[2]].shift(
        edges_verts_range.start());

    const int3 face_abd_edge = gather_tri(faces[face_ab], corner_edges);
    const int3 face_bce_edge = gather_tri(faces[face_bc], corner_edges);
    const int3 face_caf_edge = gather_tri(faces[face_ca], corner_edges);

    Map<OrderedEdge, int, 9> edge_indices;
    edge_indices.add(edges[face_edges[0]], face_edges[0]);
    edge_indices.add(edges[face_edges[1]], face_edges[1]);
    edge_indices.add(edges[face_edges[2]], face_edges[2]);

    edge_indices.add(edges[face_abd_edge[0]], face_abd_edge[0]);
    edge_indices.add(edges[face_abd_edge[1]], face_abd_edge[1]);
    edge_indices.add(edges[face_abd_edge[2]], face_abd_edge[2]);

    edge_indices.add(edges[face_bce_edge[0]], face_bce_edge[0]);
    edge_indices.add(edges[face_bce_edge[1]], face_bce_edge[1]);
    edge_indices.add(edges[face_bce_edge[2]], face_bce_edge[2]);

    edge_indices.add(edges[face_caf_edge[0]], face_caf_edge[0]);
    edge_indices.add(edges[face_caf_edge[1]], face_caf_edge[1]);
    edge_indices.add(edges[face_caf_edge[2]], face_caf_edge[2]);

    const int2 edge_ad(face_verts[0], d_vert);
    const int2 edge_bd(face_verts[1], d_vert);

    const int2 edge_be(face_verts[1], e_vert);
    const int2 edge_ce(face_verts[2], e_vert);

    const int2 edge_cf(face_verts[2], f_vert);
    const int2 edge_af(face_verts[0], f_vert);

    const std::array<float2, 6> verts_list = {projection[a_vert],
                                              projection[b_vert],
                                              projection[c_vert],
                                              projection[d_vert],
                                              projection[e_vert],
                                              projection[f_vert]};
    const std::array<int, 9> edge_indices_list = {edge_indices.lookup(edge_ab),
                                                  edge_indices.lookup(edge_bc),
                                                  edge_indices.lookup(edge_ca),
                                                  edge_indices.lookup(edge_ad),
                                                  edge_indices.lookup(edge_bd),
                                                  edge_indices.lookup(edge_be),
                                                  edge_indices.lookup(edge_ce),
                                                  edge_indices.lookup(edge_cf),
                                                  edge_indices.lookup(edge_af)};

    const IndexRange points_range = subdive_face_verts[face_i].shift(
        src_mesh.verts_num + subdive_edge_verts.total_size());
    MutableSpan<float3> face_verts_bary_weight = face_vertices_bary_weight.as_mutable_span().slice(
        subdive_face_verts[face_i]);
  });

  changed_faces.foreach_index([&](const int face_i) {
    const IndexRange face = faces[face_i];
    const int3 face_edges = gather_tri(face, corner_edges);
    const int3 face_verts = gather_tri(face, corner_verts);

    const int2 edge_ab = edges[face_edges[0]];
    const int2 edge_bc = edges[face_edges[1]];
    const int2 edge_ca = edges[face_edges[2]];

    const Span<int> ab_faces = edge_to_face_map[face_edges[0]];
    const Span<int> bc_faces = edge_to_face_map[face_edges[1]];
    const Span<int> ca_faces = edge_to_face_map[face_edges[2]];

    const int face_ab = ab_faces[0] == face_i ? ab_faces[1] : ab_faces[0];
    const int face_bc = bc_faces[0] == face_i ? bc_faces[1] : bc_faces[0];
    const int face_ca = ca_faces[0] == face_i ? ca_faces[1] : ca_faces[0];

    const int3 abd_verts = gather_tri(faces[face_ab], corner_verts);
    const int3 bce_verts = gather_tri(faces[face_bc], corner_verts);
    const int3 caf_verts = gather_tri(faces[face_ca], corner_verts);

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

    const IndexRange verts_range = subdive_face_verts[face_i].shift(faces_verts_range.start());
    const IndexRange edges_range = subdive_face_edges[face_i].shift(faces_edges_range.start());

    const IndexRange ab_points_range = subdive_edge_verts[face_edges[0]].shift(
        edges_verts_range.start());
    const IndexRange bc_points_range = subdive_edge_verts[face_edges[1]].shift(
        edges_verts_range.start());
    const IndexRange ca_points_range = subdive_edge_verts[face_edges[2]].shift(
        edges_verts_range.start());

    const int3 face_abd_edge = gather_tri(faces[face_ab], corner_edges);
    const int3 face_bce_edge = gather_tri(faces[face_bc], corner_edges);
    const int3 face_caf_edge = gather_tri(faces[face_ca], corner_edges);

    Map<OrderedEdge, int, 9> edge_indices;
    edge_indices.add(edges[face_edges[0]], face_edges[0]);
    edge_indices.add(edges[face_edges[1]], face_edges[1]);
    edge_indices.add(edges[face_edges[2]], face_edges[2]);

    edge_indices.add(edges[face_abd_edge[0]], face_abd_edge[0]);
    edge_indices.add(edges[face_abd_edge[1]], face_abd_edge[1]);
    edge_indices.add(edges[face_abd_edge[2]], face_abd_edge[2]);

    edge_indices.add(edges[face_bce_edge[0]], face_bce_edge[0]);
    edge_indices.add(edges[face_bce_edge[1]], face_bce_edge[1]);
    edge_indices.add(edges[face_bce_edge[2]], face_bce_edge[2]);

    edge_indices.add(edges[face_caf_edge[0]], face_caf_edge[0]);
    edge_indices.add(edges[face_caf_edge[1]], face_caf_edge[1]);
    edge_indices.add(edges[face_caf_edge[2]], face_caf_edge[2]);

    const int2 edge_ad(face_verts[0], d_vert);
    const int2 edge_bd(face_verts[1], d_vert);

    const int2 edge_be(face_verts[1], e_vert);
    const int2 edge_ce(face_verts[2], e_vert);

    const int2 edge_cf(face_verts[2], f_vert);
    const int2 edge_af(face_verts[0], f_vert);

    const std::array<float2, 6> verts_list = {projection[a_vert],
                                              projection[b_vert],
                                              projection[c_vert],
                                              projection[d_vert],
                                              projection[e_vert],
                                              projection[f_vert]};
    const std::array<int, 9> edge_indices_list = {edge_indices.lookup(edge_ab),
                                                  edge_indices.lookup(edge_bc),
                                                  edge_indices.lookup(edge_ca),
                                                  edge_indices.lookup(edge_ad),
                                                  edge_indices.lookup(edge_bd),
                                                  edge_indices.lookup(edge_be),
                                                  edge_indices.lookup(edge_ce),
                                                  edge_indices.lookup(edge_cf),
                                                  edge_indices.lookup(edge_af)};
  });

  changed_edges.foreach_index([&](const int edge_i) {
    const int2 edge = edges[edge_i];
    const IndexRange edge_verts_range = subdive_edge_verts[edge_i];
    const Span<float> factors = edge_vertices_factor_weight.as_span().slice(edge_verts_range);
    MutableSpan<float3> dst = dst_positions.slice(
        edge_verts_range.shift(edges_verts_range.start()));
    const float3 a = dst_positions[edge[0]];
    const float3 b = dst_positions[edge[1]];
    parallel_transform(factors, 2048, dst, [&](const float factor) {
      return bke::attribute_math::mix2(factor, a, b);
    });
  });

  changed_faces.foreach_index([&](const int face_i) {
    const IndexRange face = faces[face_i];
    const int3 face_verts = gather_tri(face, corner_verts);

    const IndexRange face_verts_range = subdive_face_verts[face_i];
    const Span<float3> bary_weights = face_vertices_bary_weight.as_span().slice(face_verts_range);
    MutableSpan<float3> dst = dst_positions.slice(
        face_verts_range.shift(faces_verts_range.start()));
    const float3 a = dst_positions[face_verts[0]];
    const float3 b = dst_positions[face_verts[1]];
    const float3 c = dst_positions[face_verts[2]];
    parallel_transform(bary_weights, 2048, dst, [&](const float3 weight) {
      return bke::attribute_math::mix3(weight, a, b, c);
    });
  });

  return dst_mesh;
}

}  // namespace blender::geometry::dyntopo
