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

#include "BLI_timeit.hh"

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

namespace FaceEdge {
static constexpr int8_t AB = 0;
static constexpr int8_t BC = 1;
static constexpr int8_t CA = 2;
}  // namespace FaceEdge

namespace FaceVert {
static constexpr int8_t A = 0;
static constexpr int8_t B = 1;
static constexpr int8_t C = 2;
}  // namespace FaceVert

namespace EdgeVert {
static constexpr int8_t A = 0;
static constexpr int8_t B = 1;
}  // namespace EdgeVert

/* Sum of pyramid of elements with floor of /p floor.
 * In other words: /return = floor + floor - 1 + floor - 2 + ... 0. */
static constexpr int pyramid_sum(const int floor)
{
  /* Zero level have zero length which one is known from prefix sum from -1 level. */
  BLI_assert(floor >= -1);
  return floor * (floor + 1) / 2;
}

static constexpr IndexRange pyramid_slice(const int floor, const int i)
{
  const int total_size = pyramid_sum(floor);
  const int begin = pyramid_sum(floor - i);
  const int end = pyramid_sum(floor - 1 - i);
  return IndexRange(total_size).take_back(begin).drop_back(end);
}

class TriangleRange {
 private:
  int base_;
  int total_;

 public:
  TriangleRange(const int base) : base_(base), total_(pyramid_sum(base)) {}

  IndexRange slice_at(const int level_i) const
  {
    const int begin = pyramid_sum(base_ - level_i);
    const int end = pyramid_sum(base_ - 1 - level_i);
    return IndexRange(total_ - begin, begin - end);
  }

  int start_of(const int level_i) const
  {
    const int begin = pyramid_sum(base_ - level_i);
    return total_ - begin;
  }

  int end_of(const int level_i) const
  {
    const int end = pyramid_sum(base_ - 1 - level_i);
    return total_ - end;
  }

  int first_of(const int level_i) const
  {
    const int begin = pyramid_sum(base_ - level_i);
    return total_ - begin;
  }

  int last_of(const int level_i) const
  {
    const int end = pyramid_sum(base_ - 1 - level_i);
    return total_ - end - 1;
  }

  int total() const
  {
    return total_;
  }

  int hight() const
  {
    return base_;
  }

  TriangleRange drop_bottom(const int levels_num) const
  {
    return TriangleRange(math::max<int>(0, base_ - levels_num));
  }
};

static void base_ico_sphere_positions(const float radius, MutableSpan<float3> positions)
{
  SCOPED_TIMER_AVERAGED(__func__);
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
  SCOPED_TIMER_AVERAGED(__func__);
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
  SCOPED_TIMER_AVERAGED(__func__);
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
  SCOPED_TIMER_AVERAGED(__func__);
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

static void interpolate_edge_points(const int edge_verts_num,
                                    const Span<float3> base_verts,
                                    MutableSpan<float3> edge_verts)
{
  SCOPED_TIMER_AVERAGED(__func__);
  const Span<int2> base_edge_points = base_edge_point_indices();

  const float lerp_factor = 1.0f / (edge_verts_num + 1);
  for (const int edge_i : IndexRange(base_edges_num)) {
    MutableSpan<float3> verts = edge_verts.slice(edge_i * edge_verts_num, edge_verts_num);

    const int2 edge = base_edge_points[edge_i];

    const float3 &point_a = base_verts[edge[0]];
    const float3 &point_b = base_verts[edge[1]];

    const float3 normalized_a = math::normalize(point_a);
    const float3 normalized_b = math::normalize(point_b);

    const float3 normal = math::normalize(
        math::cross_tri(float3(0.0f), normalized_a, normalized_b));
    const math::AngleRadian rotation = math::angle_between<float>(normalized_a, normalized_b);

    math::AngleRadian steps(0.0f);
    for (float3 &vert : verts) {
      steps += rotation * lerp_factor;
      const math::AxisAngle axis(normal, steps);
      vert = math::transform_point(math::to_quaternion(axis), point_a);
    }
  }
}

static void interpolate_face_points(const int line_subdiv,
                                    const Span<float3> base_verts,
                                    MutableSpan<float3> faces_verts)
{
  SCOPED_TIMER_AVERAGED(__func__);
  const Span<int3> base_face_points = base_face_point_indices();

  const constexpr int left_righr_points = 2;
  const TriangleRange inner_face_verts =
      TriangleRange(line_subdiv - left_righr_points).drop_bottom(1);
  const float edge_lerp_factor = 1.0f / (inner_face_verts.hight() + left_righr_points);

  for (const int face_i : IndexRange(base_faces_num)) {
    const int3 face = base_face_points[face_i];

    const float3 &point_a = base_verts[face[0]];
    const float3 &point_b = base_verts[face[1]];
    const float3 &point_c = base_verts[face[2]];

    const float3 normalized_a = math::normalize(point_a);
    const float3 normalized_b = math::normalize(point_b);
    const float3 normalized_c = math::normalize(point_c);

    const float3 normal_ac = math::normalize(
        math::cross_tri(float3(0.0f), normalized_a, normalized_c));
    const math::AngleRadian rotation_ac = math::angle_between<float>(normalized_a, normalized_c);

    const float3 normal_bc = math::normalize(
        math::cross_tri(float3(0.0f), normalized_b, normalized_c));
    const math::AngleRadian rotation_bc = math::angle_between<float>(normalized_b, normalized_c);

    math::AngleRadian steps_ac(0.0f);
    math::AngleRadian steps_bc(0.0f);

    MutableSpan<float3> face_verts = faces_verts.slice(face_i * inner_face_verts.total(),
                                                       inner_face_verts.total());
    for (const int y_index : IndexRange(inner_face_verts.hight())) {
      const int r_y_index = inner_face_verts.hight() - 1 - y_index;
      const float face_inner_factor = 1.0f / (r_y_index + left_righr_points);
      MutableSpan<float3> level_verts = face_verts.slice(inner_face_verts.slice_at(y_index));

      steps_ac += rotation_ac * edge_lerp_factor;
      steps_bc += rotation_bc * edge_lerp_factor;

      const math::AxisAngle axis_ac(normal_ac, steps_ac);
      const math::AxisAngle axis_bc(normal_bc, steps_bc);

      const float3 edge_point_a = math::transform_point(math::to_quaternion(axis_ac), point_a);
      const float3 edge_point_b = math::transform_point(math::to_quaternion(axis_bc), point_b);

      const float3 normalized_edge_a = math::normalize(edge_point_a);
      const float3 normalized_edge_b = math::normalize(edge_point_b);

      const float3 normal_c = math::normalize(
          math::cross_tri(float3(0.0f), normalized_edge_a, normalized_edge_b));
      const math::AngleRadian rotation_c = math::angle_between<float>(normalized_edge_a,
                                                                      normalized_edge_b);

      math::AngleRadian steps_c(0.0f);
      for (float3 &vert : level_verts) {
        steps_c += rotation_c * face_inner_factor;
        const math::AxisAngle axis_ac(normal_c, steps_c);
        vert = math::transform_point(math::to_quaternion(axis_ac), edge_point_a);
      }
    }
  }
}

template<typename Func>
static void edge_line_points(const int2 ends, MutableSpan<int2> edges, Func &&func)
{
  MutableSpan<int> edge_verts = edges.cast<int>();
  edge_verts.first() = ends[0];
  edge_verts.last() = ends[1];
  edge_verts = edge_verts.drop_back(1).drop_front(1);
  if (UNLIKELY(edge_verts.is_empty())) {
    return;
  }
  for (const int i : IndexRange(edges.size() - 1)) {
    const int vert_i = func(i);
    edge_verts[i * 2 + 0] = vert_i;
    edge_verts[i * 2 + 1] = vert_i;
  }
}

static void vert_edge_topology(const int edge_edges_num,
                               const int edge_verts_num,
                               MutableSpan<int2> edge_edges)
{
  SCOPED_TIMER_AVERAGED(__func__);
  const Span<int2> base_edges = base_edge_point_indices();
  if (edge_edges.size() == base_edges.size()) {
    edge_edges.copy_from(base_edges);
    return;
  }

  const IndexRange edges_verts(base_verts_num, base_edges_num * edge_verts_num);
  for (const int edge_i : IndexRange(base_edges_num)) {
    const int2 base_edge = base_edges[edge_i];
    MutableSpan<int2> edges = edge_edges.slice(edge_i * edge_edges_num, edge_edges_num);
    const IndexRange points = edges_verts.slice(edge_i * edge_verts_num, edge_verts_num);
    edge_line_points(base_edge, edges, [=](const int edge_i) -> int { return points[edge_i]; });
  }
}

static void face_edge_topology(const int edge_edges_num,
                               const int face_verts_num,
                               const int edge_verts_num,
                               const IndexRange faces_verts,
                               MutableSpan<int2> face_edges)
{
  SCOPED_TIMER_AVERAGED(__func__);
  const Span<int2> base_edge_points = base_edge_point_indices();
  const Span<int3> base_face_points = base_face_point_indices();
  const Span<int3> base_faces_edges = base_face_edge_indices();

  const TriangleRange inner_face_edges = TriangleRange(edge_edges_num).drop_bottom(1);
  const TriangleRange inner_face_verts = TriangleRange(edge_edges_num).drop_bottom(2);

  {
    SCOPED_TIMER_AVERAGED("face_edge_topology: 1");
    for (const int face_i : IndexRange(base_faces_num)) {
      const int3 face_vert_indices = base_face_points[face_i];
      const int3 face_edge_indices = base_faces_edges[face_i];
      const IndexRange faces_vert = faces_verts.slice(face_i * face_verts_num, face_verts_num);

      /* Edges from A to B, poduct to C. */
      const IndexRange edge_a_verts(
          base_verts_num + face_edge_indices[FaceEdge::CA] * edge_verts_num, edge_verts_num);
      const IndexRange edge_b_verts(
          base_verts_num + face_edge_indices[FaceEdge::BC] * edge_verts_num, edge_verts_num);

      /* Check if order of vertices in face side (edge) is the same as in edge (real edge). */
      const bool edge_a_order = int2(face_vert_indices[FaceVert::A],
                                     face_vert_indices[FaceVert::C]) ==
                                base_edge_points[face_edge_indices[FaceEdge::CA]];
      const bool edge_b_order = int2(face_vert_indices[FaceVert::B],
                                     face_vert_indices[FaceVert::C]) ==
                                base_edge_points[face_edge_indices[FaceEdge::BC]];

      MutableSpan<int2> edges = face_edges.slice(face_i * inner_face_edges.total(),
                                                 inner_face_edges.total());
      for (const int line_i : IndexRange(inner_face_edges.hight())) {
        const int begin_vert = edge_a_order ? edge_a_verts[line_i] : edge_a_verts.from_end(line_i);
        const int end_vert = edge_b_order ? edge_b_verts[line_i] : edge_b_verts.from_end(line_i);

        const IndexRange line_verts = faces_vert.slice(inner_face_verts.slice_at(line_i));
        MutableSpan<int2> line_edges = edges.slice(inner_face_edges.slice_at(line_i));
        edge_line_points(int2(begin_vert, end_vert), line_edges, [=](const int edge_i) -> int {
          return line_verts[edge_i];
        });
      }
    }
  }

  {
    SCOPED_TIMER_AVERAGED("face_edge_topology: 2");
    for (const int face_i : IndexRange(base_faces_num)) {
      const int3 face_vert_indices = base_face_points[face_i];
      const int3 face_edge_indices = base_faces_edges[face_i];
      const IndexRange faces_vert = faces_verts.slice(face_i * face_verts_num, face_verts_num);

      /* Edges from C to B, poduct to A. */
      const IndexRange edge_b_verts(
          base_verts_num + face_edge_indices[FaceEdge::AB] * edge_verts_num, edge_verts_num);
      const IndexRange edge_c_verts(
          base_verts_num + face_edge_indices[FaceEdge::CA] * edge_verts_num, edge_verts_num);

      const bool edge_c_order = int2(face_vert_indices[FaceVert::A],
                                     face_vert_indices[FaceVert::B]) ==
                                base_edge_points[face_edge_indices[FaceEdge::AB]];
      const bool edge_b_order = int2(face_vert_indices[FaceVert::A],
                                     face_vert_indices[FaceVert::C]) ==
                                base_edge_points[face_edge_indices[FaceEdge::CA]];

      MutableSpan<int2> edges = face_edges.slice(
          (base_faces_num + face_i) * inner_face_edges.total(), inner_face_edges.total());
      for (const int line_i : IndexRange(inner_face_edges.hight())) {
        const int r_line_i = inner_face_edges.hight() - 1 - line_i;
        const int begin_vert = edge_c_order ? edge_b_verts[line_i] : edge_b_verts.from_end(line_i);
        const int end_vert = edge_b_order ? edge_c_verts[line_i] : edge_c_verts.from_end(line_i);

        MutableSpan<int2> line_edges = edges.slice(inner_face_edges.slice_at(r_line_i));
        edge_line_points(int2(begin_vert, end_vert), line_edges, [=](const int edge_i) -> int {
          return faces_vert[inner_face_verts.last_of(edge_i) - r_line_i];
        });
      }
    }
  }

  {
    SCOPED_TIMER_AVERAGED("face_edge_topology: 3");
    for (const int face_i : IndexRange(base_faces_num)) {
      const int3 face_vert_indices = base_face_points[face_i];
      const int3 face_edge_indices = base_faces_edges[face_i];
      const IndexRange faces_vert = faces_verts.slice(face_i * face_verts_num, face_verts_num);

      /* Edges from C to A, poduct to B. */
      const IndexRange edge_a_verts(
          base_verts_num + face_edge_indices[FaceEdge::AB] * edge_verts_num, edge_verts_num);
      const IndexRange edge_c_verts(
          base_verts_num + face_edge_indices[FaceEdge::BC] * edge_verts_num, edge_verts_num);

      const bool edge_a_order = int2(face_vert_indices[FaceVert::A],
                                     face_vert_indices[FaceVert::B]) ==
                                base_edge_points[face_edge_indices[FaceEdge::AB]];
      const bool edge_c_order = int2(face_vert_indices[FaceVert::C],
                                     face_vert_indices[FaceVert::B]) ==
                                base_edge_points[face_edge_indices[FaceEdge::BC]];

      MutableSpan<int2> edges = face_edges.slice(
          (base_faces_num * 2 + face_i) * inner_face_edges.total(), inner_face_edges.total());
      for (const int line_i : IndexRange(inner_face_edges.hight())) {
        const int begin_vert = edge_a_order ? edge_a_verts[line_i] : edge_a_verts.from_end(line_i);
        const int end_vert = edge_c_order ? edge_c_verts[line_i] : edge_c_verts.from_end(line_i);

        MutableSpan<int2> line_edges = edges.slice(inner_face_edges.slice_at(line_i));
        edge_line_points(int2(begin_vert, end_vert), line_edges, [=](const int edge_i) -> int {
          return faces_vert[inner_face_verts.first_of(edge_i) + line_i];
        });
      }
    }
  }
}

static bool elem_of(int elem, const int2 elems)
{
  return ELEM(elem, elems[0], elems[1]);
}

static void corner_edges_topology(const int edge_edges_num,
                                  const int face_faces_num,
                                  MutableSpan<int> corner_edges)
{
  SCOPED_TIMER_AVERAGED(__func__);
  const Span<int2> base_edge_points = base_edge_point_indices();
  const Span<int3> base_face_points = base_face_point_indices();
  const Span<int3> base_faces_edges = base_face_edge_indices();

  if (corner_edges.size() == base_faces_edges.cast<int>().size()) {
    corner_edges.copy_from(base_faces_edges.cast<int>());
    return;
  }

  const TriangleRange inner_face_edges = TriangleRange(edge_edges_num).drop_bottom(1);

  /* Faces along face-edge except corner faces. */
  const int edge_faces_num = edge_edges_num - 2;

  const IndexRange corner_faces_corners(3 * 3);
  const IndexRange edge_faces_corners(corner_faces_corners.one_after_last(),
                                      edge_faces_num * 3 * 3);
  const IndexRange face_top_faces_corners(edge_faces_corners.one_after_last(),
                                          pyramid_sum(edge_edges_num - 1) * 3);
  const IndexRange face_bottom_faces_corners(face_top_faces_corners.one_after_last(),
                                             pyramid_sum(edge_edges_num - 3) * 3);

  for (const int face_i : IndexRange(base_faces_num)) {
    const int3 face_vert_indices = base_face_points[face_i];
    const int3 face_edge_indices = base_faces_edges[face_i];
    MutableSpan<int> face_cornder_edges = corner_edges.slice(face_i * face_faces_num * 3,
                                                             face_faces_num * 3);

    const IndexRange edge_a_edges(face_edge_indices[FaceEdge::AB] * edge_edges_num,
                                  edge_edges_num);
    const IndexRange edge_b_edges(face_edge_indices[FaceEdge::BC] * edge_edges_num,
                                  edge_edges_num);
    const IndexRange edge_c_edges(face_edge_indices[FaceEdge::CA] * edge_edges_num,
                                  edge_edges_num);

    const IndexRange face_edges_a(base_edges_num * edge_edges_num +
                                      (base_faces_num * 0 + face_i) * inner_face_edges.total(),
                                  inner_face_edges.total());
    const IndexRange face_edges_b(base_edges_num * edge_edges_num +
                                      (base_faces_num * 1 + face_i) * inner_face_edges.total(),
                                  inner_face_edges.total());
    const IndexRange face_edges_c(base_edges_num * edge_edges_num +
                                      (base_faces_num * 2 + face_i) * inner_face_edges.total(),
                                  inner_face_edges.total());

    const bool corner_ca_order = elem_of(
        base_edge_points[face_edge_indices[FaceEdge::CA]][EdgeVert::A],
        base_edge_points[face_edge_indices[FaceEdge::BC]]);
    const bool corner_ab_order = elem_of(
        base_edge_points[face_edge_indices[FaceEdge::AB]][EdgeVert::A],
        base_edge_points[face_edge_indices[FaceEdge::CA]]);
    const bool corner_bc_order = elem_of(
        base_edge_points[face_edge_indices[FaceEdge::BC]][EdgeVert::A],
        base_edge_points[face_edge_indices[FaceEdge::AB]]);

    face_cornder_edges[corner_faces_corners[0]] = face_edges_a.last();
    face_cornder_edges[corner_faces_corners[1]] = !corner_bc_order ? edge_b_edges.first() :
                                                                     edge_b_edges.last();
    face_cornder_edges[corner_faces_corners[2]] = corner_ca_order ? edge_c_edges.first() :
                                                                    edge_c_edges.last();

    face_cornder_edges[corner_faces_corners[3]] = corner_ab_order ? edge_a_edges.first() :
                                                                    edge_a_edges.last();
    face_cornder_edges[corner_faces_corners[4]] = face_edges_b.last();
    face_cornder_edges[corner_faces_corners[5]] = !corner_ca_order ? edge_c_edges.first() :
                                                                     edge_c_edges.last();

    face_cornder_edges[corner_faces_corners[6]] = !corner_ab_order ? edge_a_edges.first() :
                                                                     edge_a_edges.last();
    face_cornder_edges[corner_faces_corners[7]] = corner_bc_order ? edge_b_edges.first() :
                                                                    edge_b_edges.last();
    face_cornder_edges[corner_faces_corners[8]] = face_edges_c.last();

    const bool edge_a_order = int2(face_vert_indices[FaceVert::A],
                                   face_vert_indices[FaceVert::B]) ==
                              base_edge_points[face_edge_indices[FaceEdge::AB]];
    for (const int i : IndexRange(edge_faces_num)) {
      const int r_i = edge_faces_num - 1 - i;
      face_cornder_edges[edge_faces_corners[i * 3 + 0]] = edge_a_order ?
                                                              edge_a_edges[i + 1] :
                                                              edge_a_edges.from_end(i + 1);
      face_cornder_edges[edge_faces_corners[i * 3 + 1]] =
          face_edges_c[inner_face_edges.first_of(i)];
      face_cornder_edges[edge_faces_corners[i * 3 + 2]] =
          face_edges_b[inner_face_edges.first_of(r_i)];
    }

    const bool edge_b_order = int2(face_vert_indices[FaceVert::B],
                                   face_vert_indices[FaceVert::C]) !=
                              base_edge_points[face_edge_indices[FaceEdge::BC]];
    for (const int i : IndexRange(edge_faces_num)) {
      const int r_i = edge_faces_num - 1 - i;
      face_cornder_edges[edge_faces_corners[(edge_faces_num + i) * 3 + 0]] =
          edge_b_order ? edge_b_edges[i + 1] : edge_b_edges.from_end(i + 1);
      face_cornder_edges[edge_faces_corners[(edge_faces_num + i) * 3 + 1]] =
          face_edges_c[inner_face_edges.last_of(i)];
      face_cornder_edges[edge_faces_corners[(edge_faces_num + i) * 3 + 2]] =
          face_edges_a[inner_face_edges.last_of(r_i)];
    }

    const bool edge_c_order = int2(face_vert_indices[FaceVert::A],
                                   face_vert_indices[FaceVert::C]) ==
                              base_edge_points[face_edge_indices[FaceEdge::CA]];
    for (const int i : IndexRange(edge_faces_num)) {
      const int r_i = edge_faces_num - 1 - i;
      face_cornder_edges[edge_faces_corners[(edge_faces_num * 2 + i) * 3 + 0]] =
          edge_c_order ? edge_c_edges[i + 1] : edge_c_edges.from_end(i + 1);
      face_cornder_edges[edge_faces_corners[(edge_faces_num * 2 + i) * 3 + 1]] =
          face_edges_a[inner_face_edges.first_of(i)];
      face_cornder_edges[edge_faces_corners[(edge_faces_num * 2 + i) * 3 + 2]] =
          face_edges_b[inner_face_edges.last_of(r_i)];
    }

    const int top_faces_num = edge_edges_num - 1;
    for (const int line_i : IndexRange(top_faces_num)) {
      const IndexRange line_range = inner_face_edges.slice_at(line_i);
      MutableSpan<int> face_line_corners = face_cornder_edges.slice(
          face_top_faces_corners.slice(line_range.start() * 3, line_range.size() * 3));
      for (const int i : line_range.index_range()) {
        const int r_i = edge_faces_num - i - line_i;
        face_line_corners[i * 3 + 0] = face_edges_a[line_range[i]];
        face_line_corners[i * 3 + 1] = face_edges_c[inner_face_edges.first_of(i)] + line_i;
        face_line_corners[i * 3 + 2] = face_edges_b[inner_face_edges.first_of(r_i)] + line_i;
      }
    }

    const TriangleRange bottom_faces = TriangleRange(edge_edges_num).drop_bottom(3);
    for (const int line_i : IndexRange(bottom_faces.hight())) {
      const int r_line_i = bottom_faces.hight() - line_i - 1;
      const int a_line_start = inner_face_edges.first_of(line_i) + 1;
      const IndexRange line_range = bottom_faces.slice_at(line_i);
      MutableSpan<int> face_line_corners = face_cornder_edges.slice(
          face_bottom_faces_corners.slice(line_range.start() * 3, line_range.size() * 3));
      for (const int i : line_range.index_range()) {
        face_line_corners[i * 3 + 0] = face_edges_a[a_line_start + i];
        face_line_corners[i * 3 + 1] = face_edges_c[inner_face_edges.first_of(i)] + line_i + 1;
        face_line_corners[i * 3 + 2] = face_edges_b[inner_face_edges.first_of(r_line_i - i)] +
                                       line_i + 1;
      }
    }
  }
}

static void corner_verts_from_edges(const Span<int> corner_edges,
                                    const Span<int2> edges,
                                    const int faces_num,
                                    MutableSpan<int> corner_verts)
{
  SCOPED_TIMER_AVERAGED(__func__);
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
  std::cout << std::endl;
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
  const int faces_num = base_faces_num * face_faces_num;
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
  interpolate_face_points(line_subdiv, positions.slice(vert_points), positions.slice(face_points));

  const IndexRange edge_edges(base_edges_num * edge_edges_num);
  const IndexRange face_edges(edge_edges.one_after_last(), face_edges_num * base_faces_num * 3);

  vert_edge_topology(edge_edges_num, edge_verts_num, edges.slice(edge_edges));
  face_edge_topology(
      edge_edges_num, face_verts_num, edge_verts_num, face_points, edges.slice(face_edges));

  corner_edges_topology(edge_edges_num, face_faces_num, corner_edges);
  corner_verts_from_edges(corner_edges, edges, faces_num, corner_verts);

  {
    SCOPED_TIMER_AVERAGED("Flip normals");
    IndexMaskMemory memory;
    const IndexMask normals_mask = IndexMask::from_predicate(
        IndexMask(faces_num), GrainSize(2048), memory, [&](const int i) -> bool {
          const int face_i = i * 3;
          const float3 normal = bke::mesh::face_normal_calc(positions,
                                                            corner_verts.slice(face_i, 3));
          const float3 pos_a = math::normalize(positions[corner_verts[face_i]]);
          return !math::is_equal(normal, pos_a, 0.4f);
        });

    bke::mesh_flip_faces(*mesh, normals_mask);
  }

  BLI_assert(std::all_of(edges.cast<int>().begin(),
                         edges.cast<int>().end(),
                         [=](const int i) -> bool { return IndexRange(verts_num).contains(i); }));

  mesh->tag_loose_verts_none();
  mesh->tag_loose_edges_none();
  mesh->tag_overlapping_none();
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
