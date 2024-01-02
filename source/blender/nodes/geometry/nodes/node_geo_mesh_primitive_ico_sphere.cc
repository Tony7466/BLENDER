/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>

#include <array>

#include "DNA_mesh_types.h"

#include "BKE_lib_id.h"
#include "BKE_material.h"
#include "BKE_mesh.hh"

#include "BLI_array_utils.hh"
#include "BLI_map.hh"
#include "BLI_math_base.h"
#include "BLI_math_base.hh"
#include "BLI_math_euler.hh"
#include "BLI_math_quaternion.hh"
#include "BLI_math_rotation.hh"
#include "BLI_math_rotation_types.hh"
#include "BLI_ordered_edge.hh"
#include "BLI_span.hh"

#include "GEO_randomize.hh"

#include "bmesh.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_mesh_primitive_ico_sphere_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("Radius")
      .default_value(1.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Distance from the generated points to the origin");
  b.add_input<decl::Int>("Subdivisions")
      .default_value(1)
      .min(1)
      .max(7)
      .description("Number of subdivisions on top of the basic icosahedron");

  b.add_input<decl::Bool>("New");

  b.add_output<decl::Geometry>("Mesh");
  b.add_output<decl::Vector>("UV Map").field_on_all();
}

static Bounds<float3> calculate_bounds_ico_sphere(const float radius, const int subdivisions)
{
  const float delta_phi = (2.0f * M_PI) / 5.0f;
  const float theta = std::cos(std::atan(0.5f));
  const float ro = radius * std::sin(delta_phi);

  float x_max = radius;
  float x_min = -radius;
  float y_max = radius;
  float y_min = -radius;

  if (subdivisions == 1) {
    x_max = radius * theta;
    x_min = -x_max;
    y_max = ro * theta;
    y_min = -y_max;
  }
  else if (subdivisions == 2) {
    x_max = ro;
    x_min = -x_max;
  }

  const float3 bounds_min(x_min, y_min, -radius);
  const float3 bounds_max(x_max, y_max, radius);

  return {bounds_min, bounds_max};
}

static Mesh *create_ico_sphere_mesh(const int subdivisions,
                                    const float radius,
                                    const AttributeIDRef &uv_map_id)
{
  if (subdivisions >= 3) {
    /* Most nodes don't need this because they internally use multi-threading which triggers
     * lazy-threading without any extra code. */
    lazy_threading::send_hint();
  }

  const float4x4 transform = float4x4::identity();

  const bool create_uv_map = bool(uv_map_id);

  BMeshCreateParams bmesh_create_params{};
  bmesh_create_params.use_toolflags = true;
  const BMAllocTemplate allocsize = {0, 0, 0, 0};
  BMesh *bm = BM_mesh_create(&allocsize, &bmesh_create_params);
  BM_data_layer_add_named(bm, &bm->ldata, CD_PROP_FLOAT2, "UVMap");
  /* Make sure the associated boolean layers exists as well. Normally this would be done when
   * adding a UV layer via python or when copying from Mesh, but when we 'manually' create the UV
   * layer we need to make sure the boolean layers exist as well. */
  BM_uv_map_ensure_select_and_pin_attrs(bm);

  BMO_op_callf(bm,
               BMO_FLAG_DEFAULTS,
               "create_icosphere subdivisions=%i radius=%f matrix=%m4 calc_uvs=%b",
               subdivisions,
               std::abs(radius),
               transform.ptr(),
               create_uv_map);

  BMeshToMeshParams params{};
  params.calc_object_remap = false;
  Mesh *mesh = reinterpret_cast<Mesh *>(BKE_id_new_nomain(ID_ME, nullptr));
  BKE_id_material_eval_ensure_default_slot(&mesh->id);
  BM_mesh_bm_to_me(nullptr, bm, mesh, &params);
  BM_mesh_free(bm);

  /* The code above generates a "UVMap" attribute. The code below renames that attribute, we don't
   * have a simple utility for that yet though so there is some overhead right now. */
  MutableAttributeAccessor attributes = mesh->attributes_for_write();
  if (create_uv_map) {
    const VArraySpan orig_uv_map = *attributes.lookup<float2>("UVMap");
    SpanAttributeWriter<float2> uv_map = attributes.lookup_or_add_for_write_only_span<float2>(
        uv_map_id, AttrDomain::Corner);
    uv_map.span.copy_from(orig_uv_map);
    uv_map.finish();
  }
  attributes.remove("UVMap");

  geometry::debug_randomize_mesh_order(mesh);

  mesh->bounds_set_eager(calculate_bounds_ico_sphere(radius, subdivisions));
  return mesh;
}

static constexpr int base_verts_num = 12;
static constexpr int latitude_num = (base_verts_num - 2) / 2;
static constexpr int base_edges_num = 30;
static constexpr int base_faces_num = 20;
static constexpr int base_face_quads = base_faces_num / 2;

/* Sum of pyramid of elements with floor of /p floor.
 * In other words: /return = floor + floor - 1 + floor - 2 + ... 0. */
static constexpr int pyramid_sum(const int floor)
{
  return floor * (floor + 1) / 2;
}

static constexpr IndexRange pyramid_slice(const int floor, const int i)
{
  const int total_size = pyramid_sum(floor);
  const int begin = pyramid_sum(floor - i);
  const int end = pyramid_sum(floor - 1 - i);
  return IndexRange(total_size).take_back(begin).drop_back(end);
}

static void base_ico_sphere_positions(const float radius, MutableSpan<float3> positions)
{
  positions.first() = float3(0.0f, 0.0f, 1.0f);

  const float latitude = math::atan(0.5f);

  MutableSpan<float3> top_latitude = positions.drop_front(1).take_front(latitude_num);
  MutableSpan<float3> bottom_latitude = positions.drop_back(1).take_back(latitude_num);

  const constexpr float step = M_PI / latitude_num * 2.0f;
  const float3 base_vector(1.0f, 0.0f, 0.0f);
  for (const int i : IndexRange(latitude_num)) {
    const math::EulerXYZ rotation(0.0f, -latitude, i * step);
    top_latitude[i] = math::transform_point(math::to_quaternion(rotation), base_vector);
  }
  for (const int i : IndexRange(latitude_num)) {
    const math::EulerXYZ rotation(0.0f, latitude, (i - 0.5f) * step);
    bottom_latitude[i] = math::transform_point(math::to_quaternion(rotation), base_vector);
  }

  positions.last() = float3(0.0f, 0.0f, -1.0f);

  for (float3 &position : positions) {
    position *= radius;
  }
}

static Span<int2> base_edge_point_indices()
{
  static const auto edge_points = []() -> std::array<int2, base_edges_num> {
    std::array<int2, base_edges_num> edge_points;

    const constexpr int first_point = 0;
    const constexpr IndexRange top_latitude_points(1, latitude_num);
    const constexpr IndexRange bottom_latitude_points(1 + latitude_num, latitude_num);
    const constexpr int last_point = base_verts_num - 1;

    for (const int i : IndexRange(latitude_num)) {
      edge_points[latitude_num * 0 + i] = int2(first_point, top_latitude_points[i]);
      edge_points[latitude_num * 1 + i] = int2(last_point, bottom_latitude_points[i]);

      const int wrap_i = math::mod(i + 1, latitude_num);
      edge_points[latitude_num * 2 + i] = int2(top_latitude_points[i],
                                               top_latitude_points[wrap_i]);
      edge_points[latitude_num * 3 + i] = int2(bottom_latitude_points[i],
                                               bottom_latitude_points[wrap_i]);

      edge_points[latitude_num * 4 + i] = int2(top_latitude_points[i], bottom_latitude_points[i]);
      edge_points[latitude_num * 5 + i] = int2(top_latitude_points[i],
                                               bottom_latitude_points[wrap_i]);
    }

    return edge_points;
  }();

  return edge_points;
}

static Span<int3> base_face_point_indices()
{
  static const auto face_points = []() -> std::array<int3, base_faces_num> {
    std::array<int3, base_faces_num> face_points;

    const constexpr int first_point = 0;
    const constexpr IndexRange top_latitude_points(1, latitude_num);
    const constexpr IndexRange bottom_latitude_points(1 + latitude_num, latitude_num);
    const constexpr int last_point = base_verts_num - 1;

    for (const int i : IndexRange(latitude_num)) {
      const int wrap_i = math::mod(i + 1, latitude_num);
      face_points[latitude_num * 0 + i] = int3(
          top_latitude_points[i], top_latitude_points[wrap_i], first_point);
      face_points[latitude_num * 1 + i] = int3(
          bottom_latitude_points[i], bottom_latitude_points[wrap_i], last_point);

      face_points[latitude_num * 2 + i] = int3(
          top_latitude_points[i], top_latitude_points[wrap_i], bottom_latitude_points[wrap_i]);
      face_points[latitude_num * 3 + i] = int3(
          bottom_latitude_points[i], bottom_latitude_points[wrap_i], top_latitude_points[i]);
    }

    return face_points;
  }();

  return face_points;
}

static Span<int3> base_face_edge_indices()
{
  static const auto face_edges = []() -> std::array<int3, base_faces_num> {
    std::array<int3, base_faces_num> face_edges;

    Span<int2> edge_points = base_edge_point_indices();
    Span<int3> face_points = base_face_point_indices();

    Map<OrderedEdge, int> edge_points_map;
    for (const int i : IndexRange(base_edges_num)) {
      edge_points_map.add(edge_points[i], i);
    }

    for (const int face_i : IndexRange(base_faces_num)) {
      face_edges[face_i][0] = edge_points_map.lookup(face_points[face_i].xy());
      face_edges[face_i][1] = edge_points_map.lookup(face_points[face_i].yz());
      face_edges[face_i][2] = edge_points_map.lookup(
          int2(face_points[face_i][2], face_points[face_i][0]));
    }

    return face_edges;
  }();

  return face_edges;
}

static float3 polar_lert(const float3 &a, const float3 &b, const float factor)
{
  const float3 normalized_a = math::normalize(a);
  const float3 normalized_b = math::normalize(b);

  const float3 normal = math::normalize(math::cross_tri(float3(0.0f), normalized_a, normalized_b));
  const math::AngleRadian rotation = math::angle_between<float>(normalized_a, normalized_b);

  const math::AxisAngle axis(normal, rotation * factor);
  return math::transform_point(math::to_quaternion(axis), a);
}

static void interpolate_edge_points(const int subdiv_verts_num,
                                    const Span<float3> base_points,
                                    MutableSpan<float3> edge_points)
{
  const Span<int2> base_edge_points = base_edge_point_indices();

  for (const int edge_i : IndexRange(base_edges_num)) {
    MutableSpan<float3> points = edge_points.slice(edge_i * subdiv_verts_num, subdiv_verts_num);

    const int2 edge = base_edge_points[edge_i];
    const float3 point = base_points[edge[0]];

    const float3 point_a = base_points[edge[0]];
    const float3 point_b = base_points[edge[1]];

    for (const int i : IndexRange(subdiv_verts_num)) {
      points[i] = polar_lert(point_a, point_b, float(i + 1) / (subdiv_verts_num + 1));
    }
  }
}

static void interpolate_face_points(const int face_points_num,
                                    const int edge_points_num,
                                    const Span<float3> base_points,
                                    MutableSpan<float3> face_points)
{
  if (face_points_num == 0) {
    return;
  }
  const Span<int3> base_face_points = base_face_point_indices();

  for (const int face_i : IndexRange(base_faces_num)) {
    MutableSpan<float3> points = face_points.slice(face_i * face_points_num, face_points_num);
    const int3 face = base_face_points[face_i];

    const float3 point_a = base_points[face[0]];
    const float3 point_b = base_points[face[1]];
    const float3 point_c = base_points[face[2]];

    const int points_num = edge_points_num - 1;
    for (const int y_index : IndexRange(points_num)) {
      const float3 edge_point_a = polar_lert(
          point_a, point_c, float(y_index + 1) / (edge_points_num + 1));
      const float3 edge_point_b = polar_lert(
          point_b, point_c, float(y_index + 1) / (edge_points_num + 1));

      MutableSpan<float3> line_points = points.slice(pyramid_slice(points_num, y_index));
      for (const int x_index : IndexRange(points_num - y_index)) {
        line_points[x_index] = polar_lert(
            edge_point_a, edge_point_b, float(x_index + 1) / (edge_points_num - y_index));
      }
    }
  }
}

static void fill_edge_line(const IndexRange verts, const int2 ends, MutableSpan<int2> edges)
{
  BLI_assert(verts.size() == edges.size() - 1);
  if (UNLIKELY(verts.is_empty())) {
    edges.first() = ends;
    return;
  }

  const IndexRange left_verts = verts.drop_back(1);
  const IndexRange right_verts = verts.drop_front(1);

  edges.first()[0] = ends[0];
  edges.first()[1] = verts.first();

  const IndexRange range = edges.index_range().drop_front(1).drop_back(1);
  for (const int i : range.index_range()) {
    edges[range[i]] = int2(left_verts[i], right_verts[i]);
  }

  edges.last()[0] = verts.last();
  edges.last()[1] = ends[1];
}

static void vert_edge_topology(const int edge_edges_num,
                               const int edge_verts_num,
                               MutableSpan<int2> edge_edges)
{
  const Span<int2> base_edges = base_edge_point_indices();
  if (edge_edges_num == 1) {
    edge_edges.copy_from(base_edges);
    return;
  }

  const IndexRange edges_points(base_verts_num, base_edges_num * edge_verts_num);
  for (const int edge_i : IndexRange(base_edges_num)) {
    const int2 base_edge = base_edges[edge_i];
    MutableSpan<int2> edges = edge_edges.slice(edge_i * edge_edges_num, edge_edges_num);
    const IndexRange points = edges_points.slice(edge_i * edge_verts_num, edge_verts_num);
    fill_edge_line(points, base_edge, edges);
  }
}

static void face_edge_topology(const int edge_edges_num,
                               const int face_verts_num,
                               const int edge_verts_num,
                               const IndexRange faces_verts,
                               MutableSpan<int2> face_edges)
{
  const Span<int2> base_edge_points = base_edge_point_indices();
  const Span<int3> base_face_points = base_face_point_indices();
  const Span<int3> base_faces_edges = base_face_edge_indices();

  for (const int face_i : IndexRange(base_faces_num)) {
    const int3 face_vert_indices = base_face_points[face_i];
    const int3 face_edge_indices = base_faces_edges[face_i];
    const IndexRange faces_vert = faces_verts.slice(face_i * face_verts_num, face_verts_num);

    /* Edges from A to B, poduct to C. */

    /* Edge C verts: {C, A}. */
    const IndexRange edge_a_verts(base_verts_num + face_edge_indices[2] * edge_verts_num,
                                  edge_verts_num);
    /* Edge B verts: {B, C}. */
    const IndexRange edge_b_verts(base_verts_num + face_edge_indices[1] * edge_verts_num,
                                  edge_verts_num);

    const bool edge_a_order = int2(face_vert_indices[0], face_vert_indices[2]) ==
                              base_edge_points[face_edge_indices[2]];
    const bool edge_b_order = int2(face_vert_indices[1], face_vert_indices[2]) ==
                              base_edge_points[face_edge_indices[1]];

    MutableSpan<int2> edges = face_edges.slice(face_i * pyramid_sum(edge_edges_num - 1),
                                               pyramid_sum(edge_edges_num - 1));
    for (const int line_i : IndexRange(edge_edges_num - 1)) {
      const int begin_vert = edge_a_order ? edge_a_verts[line_i] :
                                            edge_a_verts.one_after_last() - 1 - line_i;
      const int end_vert = edge_b_order ? edge_b_verts[line_i] :
                                          edge_b_verts.one_after_last() - 1 - line_i;

      const IndexRange line_verts = faces_vert.slice(pyramid_slice(edge_edges_num - 2, line_i));
      fill_edge_line(line_verts,
                     int2(begin_vert, end_vert),
                     edges.slice(pyramid_slice(edge_edges_num - 1, line_i)));
    }
  }

  for (const int face_i : IndexRange(base_faces_num)) {
    const int3 face_vert_indices = base_face_points[face_i];
    const int3 face_edge_indices = base_faces_edges[face_i];
    const IndexRange faces_vert = faces_verts.slice(face_i * face_verts_num, face_verts_num);

    /* Edges from C to B, poduct to A. */

    /* Edge B verts: {A, B}. */
    const IndexRange edge_c_verts(base_verts_num + face_edge_indices[0] * edge_verts_num,
                                  edge_verts_num);
    /* Edge C verts: {C, A}. */
    const IndexRange edge_b_verts(base_verts_num + face_edge_indices[2] * edge_verts_num,
                                  edge_verts_num);

    const bool edge_c_order = int2(face_vert_indices[0], face_vert_indices[1]) ==
                              base_edge_points[face_edge_indices[0]];
    const bool edge_b_order = int2(face_vert_indices[0], face_vert_indices[2]) ==
                              base_edge_points[face_edge_indices[2]];

    MutableSpan<int2> edges = face_edges.slice((base_faces_num + face_i) *
                                                   pyramid_sum(edge_edges_num - 1),
                                               pyramid_sum(edge_edges_num - 1));
    for (const int line_i : IndexRange(edge_edges_num - 1)) {
      const int r_line_i = edge_edges_num - 2 - line_i;
      const int begin_vert = edge_c_order ? edge_c_verts[line_i] :
                                            edge_c_verts.one_after_last() - 1 - line_i;
      const int end_vert = edge_b_order ? edge_b_verts[line_i] :
                                          edge_b_verts.one_after_last() - 1 - line_i;

      MutableSpan<int2> line_edges = edges.slice(pyramid_slice(edge_edges_num - 1, r_line_i));
      if (line_edges.size() == 1) {
        line_edges.first() = int2(begin_vert, end_vert);
        continue;
      }

      line_edges.first()[0] = begin_vert;
      line_edges.first()[1] =
          faces_vert.slice(pyramid_slice(edge_edges_num - 2, 0)).drop_back(r_line_i).last();

      const IndexRange range = line_edges.index_range().drop_front(1).drop_back(1);
      for (const int i : range.index_range()) {
        line_edges[range[i]] = int2(
            faces_vert.slice(pyramid_slice(edge_edges_num - 2, i)).drop_back(r_line_i).last(),
            faces_vert.slice(pyramid_slice(edge_edges_num - 2, i + 1)).drop_back(r_line_i).last());
      }

      line_edges.last()[0] = faces_vert.slice(pyramid_slice(edge_edges_num - 2, range.size()))
                                 .drop_back(r_line_i)
                                 .last();
      line_edges.last()[1] = end_vert;
    }
  }

  for (const int face_i : IndexRange(base_faces_num)) {
    const int3 face_vert_indices = base_face_points[face_i];
    const int3 face_edge_indices = base_faces_edges[face_i];
    const IndexRange faces_vert = faces_verts.slice(face_i * face_verts_num, face_verts_num);

    /* Edges from C to A, poduct to B. */

    /* Edge B verts: {A, B}. */
    const IndexRange edge_a_verts(base_verts_num + face_edge_indices[0] * edge_verts_num,
                                  edge_verts_num);
    /* Edge C verts: {B, C}. */
    const IndexRange edge_c_verts(base_verts_num + face_edge_indices[1] * edge_verts_num,
                                  edge_verts_num);

    const bool edge_a_order = int2(face_vert_indices[0], face_vert_indices[1]) ==
                              base_edge_points[face_edge_indices[0]];
    const bool edge_c_order = int2(face_vert_indices[2], face_vert_indices[1]) ==
                              base_edge_points[face_edge_indices[1]];

    MutableSpan<int2> edges = face_edges.slice((base_faces_num * 2 + face_i) *
                                                   pyramid_sum(edge_edges_num - 1),
                                               pyramid_sum(edge_edges_num - 1));
    for (const int line_i : IndexRange(edge_edges_num - 1)) {
      const int begin_vert = edge_a_order ? edge_a_verts[line_i] :
                                            edge_a_verts.one_after_last() - 1 - line_i;
      const int end_vert = edge_c_order ? edge_c_verts[line_i] :
                                          edge_c_verts.one_after_last() - 1 - line_i;

      MutableSpan<int2> line_edges = edges.slice(pyramid_slice(edge_edges_num - 1, line_i));
      if (line_edges.size() == 1) {
        line_edges.first() = int2(begin_vert, end_vert);
        continue;
      }

      line_edges.first()[0] = begin_vert;
      line_edges.first()[1] =
          faces_vert.slice(pyramid_slice(edge_edges_num - 2, 0)).drop_front(line_i).start();

      const IndexRange range = line_edges.index_range().drop_front(1).drop_back(1);
      for (const int i : range.index_range()) {
        line_edges[range[i]] = int2(
            faces_vert.slice(pyramid_slice(edge_edges_num - 2, i)).drop_front(line_i).start(),
            faces_vert.slice(pyramid_slice(edge_edges_num - 2, i + 1)).drop_front(line_i).start());
      }

      line_edges.last()[0] = faces_vert.slice(pyramid_slice(edge_edges_num - 2, range.size()))
                                 .drop_front(line_i)
                                 .start();
      line_edges.last()[1] = end_vert;
    }
  }
}

static bool elem_of(int elem, const int2 elems)
{
  return ELEM(elem, elems[0], elems[1]);
}

static void corner_edges_topology(const int edge_edges_num,
                                  const int face_faces_num,
                                  MutableSpan<int> corner_edges,
                                  const Span<int2> edges)
{
  const Span<int2> base_edge_points = base_edge_point_indices();
  const Span<int3> base_face_points = base_face_point_indices();
  const Span<int3> base_faces_edges = base_face_edge_indices();
  if (edge_edges_num == 1) {
    corner_edges.copy_from(base_faces_edges.cast<int>());
    return;
  }
  for (const int face_i : IndexRange(base_faces_num)) {
    const int3 face_vert_indices = base_face_points[face_i];
    const int3 face_edge_indices = base_faces_edges[face_i];
    // MutableSpan<int> face_cornder_edges = corner_edges.slice(face_i * face_faces_num * 3,
    // face_faces_num * 3);
    MutableSpan<int> face_cornder_edges = corner_edges.slice(
        face_i * (3 * (3 + (edge_edges_num - 2) * 1)), 3 * (3 + (edge_edges_num - 2) * 1));

    const IndexRange edge_a_edges(face_edge_indices[0] * edge_edges_num, edge_edges_num);
    const IndexRange edge_b_edges(face_edge_indices[1] * edge_edges_num, edge_edges_num);
    const IndexRange edge_c_edges(face_edge_indices[2] * edge_edges_num, edge_edges_num);

    const IndexRange face_edges_a(base_edges_num * edge_edges_num +
                                      (base_faces_num * 0 + face_i) *
                                          pyramid_sum(edge_edges_num - 1),
                                  pyramid_sum(edge_edges_num - 1));
    const IndexRange face_edges_b(base_edges_num * edge_edges_num +
                                      (base_faces_num * 1 + face_i) *
                                          pyramid_sum(edge_edges_num - 1),
                                  pyramid_sum(edge_edges_num - 1));
    const IndexRange face_edges_c(base_edges_num * edge_edges_num +
                                      (base_faces_num * 2 + face_i) *
                                          pyramid_sum(edge_edges_num - 1),
                                  pyramid_sum(edge_edges_num - 1));

    face_cornder_edges[0] = elem_of(base_edge_points[face_edge_indices[2]][0],
                                    base_edge_points[face_edge_indices[1]]) ?
                                edge_c_edges.first() :
                                edge_c_edges.last();
    face_cornder_edges[1] = face_edges_a.last();
    face_cornder_edges[2] = elem_of(base_edge_points[face_edge_indices[1]][0],
                                    base_edge_points[face_edge_indices[2]]) ?
                                edge_b_edges.first() :
                                edge_b_edges.last();

    face_cornder_edges[3] = elem_of(base_edge_points[face_edge_indices[2]][0],
                                    base_edge_points[face_edge_indices[0]]) ?
                                edge_c_edges.first() :
                                edge_c_edges.last();
    face_cornder_edges[4] = face_edges_b.last();
    face_cornder_edges[5] = elem_of(base_edge_points[face_edge_indices[0]][0],
                                    base_edge_points[face_edge_indices[2]]) ?
                                edge_a_edges.first() :
                                edge_a_edges.last();

    face_cornder_edges[6] = elem_of(base_edge_points[face_edge_indices[0]][0],
                                    base_edge_points[face_edge_indices[1]]) ?
                                edge_a_edges.first() :
                                edge_a_edges.last();
    face_cornder_edges[7] = face_edges_c.last();
    face_cornder_edges[8] = elem_of(base_edge_points[face_edge_indices[1]][0],
                                    base_edge_points[face_edge_indices[0]]) ?
                                edge_b_edges.first() :
                                edge_b_edges.last();

    face_cornder_edges = face_cornder_edges.drop_front(3 * 3);

    for (const int i : IndexRange(edge_edges_num - 2)) {
      face_cornder_edges[i * 3 + 0] = edge_a_edges[i + 1];

      const IndexRange ab_line = face_edges_c.slice(pyramid_slice(edge_edges_num - 1, i));
      const IndexRange ac_line = face_edges_b.slice(
          pyramid_slice(edge_edges_num - 1, edge_edges_num - 2 - 1 - i));

      face_cornder_edges[i * 3 + 1] = ab_line.first();
      face_cornder_edges[i * 3 + 2] = ac_line.first();
    }

    /*
    for (const int line_i : IndexRange(edge_edges_num)) {
      const IndexRange range = pyramid_slice(edge_edges_num, line_i);
      MutableSpan<int> face_line = face_cornder_edges.slice(range.first() * 3, range.size() * 3);
      if (line_i == 0) {
        for (const int i : range.index_range()) {
          face_line[i * 3 + 0] = edge_a_edges[i];
          face_line[i * 3 + 1] = face_edges_c.slice()
        }
      }
    }

    for (const int line_i : IndexRange(edge_edges_num - 1)) {
      const IndexRange range = pyramid_slice(edge_edges_num - 1, line_i);
      MutableSpan<int> face_line = face_cornder_edges.drop_front(pyramid_sum(edge_edges_num) *
    3).slice(range.first() * 3, range.size() * 3);

    }
    */
  }
}

static void corner_verts_from_edges(const Span<int> corner_edges,
                                    const Span<int2> edges,
                                    const int faces_num,
                                    MutableSpan<int> corner_verts)
{
  for (const int i : IndexRange(faces_num)) {
    const int2 edge_a = edges[corner_edges[i * 3 + 0]];
    const int2 edge_b = edges[corner_edges[i * 3 + 1]];
    const int2 edge_c = edges[corner_edges[i * 3 + 2]];

    BLI_assert(elem_of(edge_a[0], edge_b) != elem_of(edge_a[0], edge_c));
    BLI_assert(elem_of(edge_b[0], edge_a) != elem_of(edge_b[0], edge_c));
    BLI_assert(elem_of(edge_c[0], edge_b) != elem_of(edge_c[0], edge_a));

    const int vert_a = elem_of(edge_a[0], edge_b) ? edge_a[0] : edge_a[1];
    const int vert_b = bke::mesh::edge_other_vert(edge_b, vert_a);
    const int vert_c = bke::mesh::edge_other_vert(edge_c, vert_b);

    corner_verts[i * 3 + 0] = vert_a;
    corner_verts[i * 3 + 1] = vert_b;
    corner_verts[i * 3 + 2] = vert_c;
  }
}

static Mesh *ico_sphere(const int subdivisions, const float radius)
{
  BLI_assert(subdivisions > 0);
  const int line_subdiv = math::pow<int>(2, subdivisions - 1) + 1;

  const int edge_verts_num = math::max<int>(0, line_subdiv - 2);
  const int face_verts_num = math::max<int>(0,
                                            pyramid_sum(line_subdiv) - (edge_verts_num * 3 + 3));

  const int edge_edges_num = edge_verts_num + 1;
  const int face_edges_num = pyramid_sum(edge_edges_num - 1);

  const int face_faces_num = pyramid_sum(edge_verts_num + 1) * 2 - (edge_verts_num + 1);

  const int verts_num = base_verts_num + (base_edges_num - base_face_quads) * edge_verts_num +
                        base_face_quads * edge_verts_num * edge_verts_num;
  const int edges_num = base_edges_num * edge_edges_num + face_edges_num * base_faces_num * 3;
  // const int faces_num = base_faces_num * face_faces_num;
  const int faces_num = base_faces_num * (3 + (edge_edges_num - 2) * 1);
  const int corners_num = faces_num * 3;

  Mesh *mesh = BKE_mesh_new_nomain(verts_num, edges_num, faces_num, corners_num);

  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  MutableSpan<int2> edges = mesh->edges_for_write();
  MutableSpan<int> corner_edges = mesh->corner_edges_for_write();
  MutableSpan<int> corner_verts = mesh->corner_verts_for_write();

  offset_indices::fill_constant_group_size(3, 0, mesh->face_offsets_for_write());

  const IndexRange vert_points(base_verts_num);
  const IndexRange edge_points(vert_points.one_after_last(), base_edges_num * edge_verts_num);
  const IndexRange face_points(edge_points.one_after_last(), face_verts_num * base_faces_num);

  base_ico_sphere_positions(radius, positions.take_front(base_verts_num));

  interpolate_edge_points(
      edge_verts_num, positions.slice(vert_points), positions.slice(edge_points));
  interpolate_face_points(
      face_verts_num, edge_verts_num, positions.slice(vert_points), positions.slice(face_points));

  const IndexRange edge_edges(base_edges_num * edge_edges_num);
  const IndexRange face_edges(edge_edges.one_after_last(), face_edges_num * base_faces_num * 3);

  vert_edge_topology(edge_edges_num, edge_verts_num, edges.slice(edge_edges));
  face_edge_topology(
      edge_edges_num, face_verts_num, edge_verts_num, face_points, edges.slice(face_edges));

  corner_edges_topology(edge_edges_num, face_faces_num, corner_edges, edges);
  corner_verts_from_edges(corner_edges, edges, faces_num, corner_verts);

  BLI_assert(std::all_of(edges.cast<int>().begin(),
                         edges.cast<int>().end(),
                         [=](const int i) -> bool { return IndexRange(verts_num).contains(i); }));

  // mesh->tag_loose_verts_none();
  // mesh->tag_overlapping_none();
  BKE_id_material_eval_ensure_default_slot(&mesh->id);
  mesh->bounds_set_eager(calculate_bounds_ico_sphere(radius, subdivisions));

  bke::mesh_smooth_set(*mesh, false);

  geometry::debug_randomize_mesh_order(mesh);
  return mesh;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const int subdivisions = math::max<int>(1, params.extract_input<int>("Subdivisions"));
  const float radius = params.extract_input<float>("Radius");

  const bool new_type = params.extract_input<bool>("New");

  if (new_type) {
    Mesh *mesh = ico_sphere(subdivisions, radius);
    params.set_output("Mesh", GeometrySet::from_mesh(mesh));
    return;
  }

  AnonymousAttributeIDPtr uv_map_id = params.get_output_anonymous_attribute_id_if_needed("UV Map");

  Mesh *mesh = create_ico_sphere_mesh(subdivisions, radius, uv_map_id.get());
  params.set_output("Mesh", GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_MESH_PRIMITIVE_ICO_SPHERE, "Ico Sphere", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_mesh_primitive_ico_sphere_cc
