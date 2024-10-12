/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <string>

#include "DNA_pointcloud_types.h"

#include "BKE_curves.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "BLI_kdopbvh.h"
#include "BLI_math_geom.h"
#include "BLI_task.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_intersection_cc {

/* Epsilon value for curve intersections and bvh tree. */
constexpr float curve_isect_eps = 0.000001f;

typedef enum PairData {
  Single = 0,
  Paired = 1,
} PairData;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Curve").supported_type(GeometryComponent::Type::Curve);
  b.add_input<decl::Geometry>("Mesh")
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .make_available([](bNode &node) { node.custom1 = GEO_NODE_CURVE_INTERSECT_SURFACE; });
  b.add_input<decl::Bool>("Self Intersections")
      .default_value(false)
      .description("Include self intersections");
  b.add_input<decl::Bool>("All Intersections")
      .default_value(true)
      .description("Include all intersections except self intersections");
  b.add_input<decl::Vector>("Direction")
      .default_value({0.0f, 0.0f, 1.0f})
      .make_available([](bNode &node) { node.custom1 = GEO_NODE_CURVE_INTERSECT_PLANE; })
      .description("Direction of orthographic plane");
  b.add_input<decl::Vector>("Center")
      .subtype(PROP_DISTANCE)
      .make_available([](bNode &node) { node.custom1 = GEO_NODE_CURVE_INTERSECT_PLANE; })
      .description("Center of plane");
  b.add_input<decl::Float>("Distance")
      .subtype(PROP_DISTANCE)
      .min(0.0f)
      .description("Distance epsilon between intersections");

  b.add_output<decl::Geometry>("Points");
  b.add_output<decl::Int>("Curve Index").field_on_all();
  b.add_output<decl::Vector>("Direction").field_on_all();
  b.add_output<decl::Float>("Factor").field_on_all().description(
      "The portion of the spline's total length at the intersection point");
  b.add_output<decl::Float>("Length").field_on_all().description(
      "The distance along the spline at the intersection point");
  b.add_output<decl::Vector>("Pair Position")
      .field_on_all()
      .make_available([](bNode &node) { node.custom1 = GEO_NODE_CURVE_INTERSECT_CURVE; })
      .description("Position of the oppposing pair point");
  b.add_output<decl::Bool>("Pair")
      .field_on_all()
      .make_available([](bNode &node) { node.custom1 = GEO_NODE_CURVE_INTERSECT_CURVE; })
      .description("If the intersection is one of a pair of matching intersections");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  const GeometryNodeCurveIntersectionMode mode = GeometryNodeCurveIntersectionMode(
      static_cast<const bNode *>(ptr->data)->custom1);
  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
  if (ELEM(mode, GEO_NODE_CURVE_INTERSECT_CURVE, GEO_NODE_CURVE_INTERSECT_PROJECT)) {
    uiItemR(layout, ptr, "use_paired_data", UI_ITEM_R_EXPAND, nullptr, ICON_NONE);
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = GEO_NODE_CURVE_INTERSECT_CURVE;
  node->custom2 = int(PairData::Single);
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const GeometryNodeCurveIntersectionMode mode = GeometryNodeCurveIntersectionMode(node->custom1);

  bNodeSocket *mesh = static_cast<bNodeSocket *>(node->inputs.first)->next;
  bNodeSocket *self = mesh->next;
  bNodeSocket *all = self->next;
  bNodeSocket *direction = all->next;
  bNodeSocket *plane_center = direction->next;
  bNodeSocket *distance = plane_center->next;

  const bool curve_intersect = ELEM(
      mode, GEO_NODE_CURVE_INTERSECT_CURVE, GEO_NODE_CURVE_INTERSECT_PROJECT);

  bke::node_set_socket_availability(ntree, mesh, mode == GEO_NODE_CURVE_INTERSECT_SURFACE);
  bke::node_set_socket_availability(ntree, self, curve_intersect);
  bke::node_set_socket_availability(ntree, all, curve_intersect);
  bke::node_set_socket_availability(
      ntree,
      direction,
      ELEM(mode, GEO_NODE_CURVE_INTERSECT_PLANE, GEO_NODE_CURVE_INTERSECT_PROJECT));
  bke::node_set_socket_availability(ntree, plane_center, mode == GEO_NODE_CURVE_INTERSECT_PLANE);
  bke::node_set_socket_availability(ntree, distance, mode == GEO_NODE_CURVE_INTERSECT_CURVE);

  LISTBASE_FOREACH (bNodeSocket *, socket, &node->outputs) {
    if (STREQ(socket->name, "Pair Position")) {
      bke::node_set_socket_availability(ntree, socket, curve_intersect);
    }
    if (STREQ(socket->name, "Pair")) {
      bke::node_set_socket_availability(
          ntree, socket, curve_intersect && (PairData(node->custom2) == PairData::Paired));
    }
  }
}

/* Attribute outputs. */
struct AttributeOutputs {
  std::optional<std::string> curve_index;
  std::optional<std::string> direction;
  std::optional<std::string> factor;
  std::optional<std::string> length;
  std::optional<std::string> pair_position;
  std::optional<std::string> pair;
};

/* Store information from line intersection calculations. */
struct IntersectingLineInfo {
  float3 isect_point_ab;
  float3 isect_point_cd;
  float3 closest_ab;
  float3 closest_cd;
  float lambda_ab;
  float lambda_cd;
  bool intersects;
};

/* Store segment details. Start and End must be at the start for BVHTree search. */
struct Segment {
  float3 start;
  float3 end;
  float3 orig_start;
  float3 orig_end;
  float len_start;
  float len_end;
  bool is_cyclic_segment;
  bool is_first_segment;
  int pos_index;
  int curve_index;
  float curve_length;
};

/* Store data that will be used for attributes and sorting. */
struct IntersectionData {
  Vector<float> sortkey;
  Vector<float3> position;
  Vector<int> curve_index;
  Vector<float3> direction;
  Vector<float> length;
  Vector<float> factor;
  Vector<float3> pair_position;
  Vector<bool> pair;
};

using ThreadLocalData = threading::EnumerableThreadSpecific<IntersectionData>;

static void add_intersection_data(IntersectionData &data,
                                  const float3 position,
                                  const int curve_index,
                                  const float3 direction,
                                  const float length,
                                  const float curve_length,
                                  const float3 pair_position,
                                  const bool pair,
                                  const AttributeOutputs &attribute_outputs)
{
  const float factor = math::safe_divide(length, curve_length);
  data.position.append(position);
  if (attribute_outputs.curve_index) {
    data.curve_index.append(curve_index);
  }
  if (attribute_outputs.length) {
    data.length.append(length);
  }
  if (attribute_outputs.factor) {
    data.factor.append(factor);
  }
  if (attribute_outputs.direction) {
    data.direction.append(direction);
  }
  if (attribute_outputs.pair_position) {
    data.pair_position.append(pair_position);
  }
  if (attribute_outputs.pair) {
    data.pair.append(pair);
  }

  /* Create sortkey for index. Order by curve_index then factor. */
  float sortkey = float(curve_index * 2) + factor;
  data.sortkey.append(sortkey);
}

static void gather_thread_storage(ThreadLocalData &thread_storage,
                                  IntersectionData &r_data,
                                  const AttributeOutputs &attribute_outputs)
{
  int64_t total_intersections = 0;
  for (const IntersectionData &local_data : thread_storage) {
    const int64_t local_size = local_data.position.size();
    BLI_assert(local_data.sortkey.size() == local_size);
    BLI_assert(attribute_outputs.curve_index && local_data.curve_index.size() == local_size);
    BLI_assert(attribute_outputs.length && local_data.length.size() == local_size);
    BLI_assert(attribute_outputs.factor && local_data.factor.size() == local_size);
    BLI_assert(attribute_outputs.direction && local_data.direction.size() == local_size);
    BLI_assert(attribute_outputs.pair_position && local_data.pair_position.size() == local_size);
    BLI_assert(attribute_outputs.pair && local_data.pair.size() == local_size);
    total_intersections += local_size;
  }
  const int64_t start_index = r_data.position.size();
  const int64_t new_size = start_index + total_intersections;
  r_data.position.reserve(new_size);
  r_data.sortkey.reserve(new_size);

  if (attribute_outputs.curve_index) {
    r_data.curve_index.reserve(new_size);
  }
  if (attribute_outputs.length) {
    r_data.length.reserve(new_size);
  }
  if (attribute_outputs.factor) {
    r_data.factor.reserve(new_size);
  }
  if (attribute_outputs.direction) {
    r_data.direction.reserve(new_size);
  }
  if (attribute_outputs.pair_position) {
    r_data.pair_position.reserve(new_size);
  }
  if (attribute_outputs.pair) {
    r_data.pair.reserve(new_size);
  }

  for (IntersectionData &local_data : thread_storage) {
    r_data.position.extend(local_data.position);
    r_data.sortkey.extend(local_data.sortkey);

    if (attribute_outputs.curve_index) {
      r_data.curve_index.extend(local_data.curve_index);
    };
    if (attribute_outputs.length) {
      r_data.length.extend(local_data.length);
    }
    if (attribute_outputs.factor) {
      r_data.factor.extend(local_data.factor);
    }
    if (attribute_outputs.direction) {
      r_data.direction.extend(local_data.direction);
    }
    if (attribute_outputs.pair_position) {
      r_data.pair_position.extend(local_data.pair_position);
    }
    if (attribute_outputs.pair) {
      r_data.pair.extend(local_data.pair);
    }
  }
}

/* isect_line_line_epsilon_v3 is too strict for checking parallel lines. This
 * version adds an epsilon to the parallel line check^. */
static int isect_line_line_v3_loose(const float v1[3],
                                    const float v2[3],
                                    const float v3[3],
                                    const float v4[3],
                                    float r_i1[3],
                                    float r_i2[3],
                                    const float epsilon)
{
  float a[3], b[3], c[3], ab[3], cb[3];
  float d, div;
  sub_v3_v3v3(a, v2, v1);
  sub_v3_v3v3(b, v4, v3);
  sub_v3_v3v3(c, v3, v1);

  cross_v3_v3v3(ab, a, b);
  d = dot_v3v3(c, ab);
  div = dot_v3v3(ab, ab);

  /* ^Epsilon has been added here. */
  if (UNLIKELY(fabsf(div) <= epsilon)) {
    return 0;
  }
  /* test if the two lines are coplanar */
  if (UNLIKELY(fabsf(d) <= epsilon)) {
    cross_v3_v3v3(cb, c, b);

    mul_v3_fl(a, dot_v3v3(cb, ab) / div);
    add_v3_v3v3(r_i1, v1, a);
    copy_v3_v3(r_i2, r_i1);

    return 1; /* one intersection only */
  }
  /* if not */

  float n[3], t[3];
  float v3t[3], v4t[3];
  sub_v3_v3v3(t, v1, v3);

  /* offset between both plane where the lines lies */
  cross_v3_v3v3(n, a, b);
  project_v3_v3v3(t, t, n);

  /* for the first line, offset the second line until it is coplanar */
  add_v3_v3v3(v3t, v3, t);
  add_v3_v3v3(v4t, v4, t);

  sub_v3_v3v3(c, v3t, v1);
  sub_v3_v3v3(a, v2, v1);
  sub_v3_v3v3(b, v4t, v3t);

  cross_v3_v3v3(ab, a, b);
  cross_v3_v3v3(cb, c, b);

  mul_v3_fl(a, dot_v3v3(cb, ab) / dot_v3v3(ab, ab));
  add_v3_v3v3(r_i1, v1, a);

  /* for the second line, just subtract the offset from the first intersection point */
  sub_v3_v3v3(r_i2, r_i1, t);

  return 2; /* two nearest points */
}

/* Check intersection between the line segments ab and cd. Return true only if intersection point
 * is located on both line segments. */
static IntersectingLineInfo intersecting_lines(
    const float3 &a, const float3 &b, const float3 &c, const float3 &d, const float distance)
{
  IntersectingLineInfo isectinfo{};
  if (isect_line_line_v3_loose(
          a, b, c, d, isectinfo.isect_point_ab, isectinfo.isect_point_cd, curve_isect_eps))
  {
    /* Discard intersections too far away. Previous function returns intersections that are not
     * located on the actual segment. */
    if (math::distance(isectinfo.isect_point_ab, isectinfo.isect_point_cd) > distance) {
      isectinfo.intersects = false;
      return isectinfo;
    }
    /* Check intersection is on both line segments ab and cd. Lambda value is
     * captured for interpolation. Epsilon is required for matches very close to the segment end
     * points. */
    isectinfo.lambda_ab = closest_to_line_v3(isectinfo.closest_ab, isectinfo.isect_point_ab, a, b);
    if (isectinfo.lambda_ab <= -curve_isect_eps || isectinfo.lambda_ab >= 1.0f + curve_isect_eps) {
      isectinfo.intersects = false;
      return isectinfo;
    }
    isectinfo.lambda_cd = closest_to_line_v3(isectinfo.closest_cd, isectinfo.isect_point_cd, c, d);
    if (isectinfo.lambda_cd <= -curve_isect_eps || isectinfo.lambda_cd >= 1.0f + curve_isect_eps) {
      isectinfo.intersects = false;
      return isectinfo;
    }
    if (math::distance(isectinfo.closest_ab, isectinfo.closest_cd) <= distance) {
      /* Remove epsilon and clamp to 0,1 range. */
      isectinfo.lambda_ab = math::clamp(isectinfo.lambda_ab, 0.0f, 1.0f);
      isectinfo.lambda_cd = math::clamp(isectinfo.lambda_cd, 0.0f, 1.0f);
      isectinfo.intersects = true;
      return isectinfo;
    }
  }
  isectinfo.intersects = false;
  return isectinfo;
}

static float3 project_v3_plane(const float3 vector, const float3 direction)
{
  return vector - math::project(vector, direction);
}

/* Buuild curve segment bvh. */
static BVHTree *create_curve_segment_bvhtree(const bke::CurvesGeometry &src_curves,
                                             Vector<Segment> *r_curve_segments,
                                             const bool project,
                                             const float3 project_axis)
{
  src_curves.ensure_evaluated_lengths();

  const int bvh_points_num = src_curves.evaluated_points_num() + src_curves.curves_num();
  BVHTree *bvhtree = BLI_bvhtree_new(bvh_points_num, curve_isect_eps, 8, 8);
  const VArray<bool> cyclic = src_curves.cyclic();
  const OffsetIndices evaluated_points_by_curve = src_curves.evaluated_points_by_curve();

  /* Preprocess curve segments for each curve. */
  for (const int64_t curve_i : src_curves.curves_range()) {
    const IndexRange points = evaluated_points_by_curve[curve_i];
    const Span<float3> positions = src_curves.evaluated_positions().slice(points);
    const Span<float> lengths = src_curves.evaluated_lengths_for_curve(curve_i, cyclic[curve_i]);
    const float curve_length = src_curves.evaluated_length_total_for_curve(curve_i,
                                                                           cyclic[curve_i]);
    const int segment_count = positions.size() - 1;
    for (const int index : IndexRange(positions.size()).drop_back(1)) {
      const bool first_segment = (index == 0);
      Segment segment;
      segment.is_cyclic_segment = false;
      segment.is_first_segment = first_segment;
      segment.pos_index = index;
      segment.curve_index = curve_i;
      segment.curve_length = curve_length;
      segment.orig_start = positions[index];
      segment.orig_end = positions[1 + index];
      segment.start = project ? project_v3_plane(segment.orig_start, project_axis) :
                                segment.orig_start;
      segment.end = project ? project_v3_plane(segment.orig_end, project_axis) : segment.orig_end;
      segment.len_start = first_segment ? 0.0f : lengths[index - 1];
      segment.len_end = lengths[index];
      const int bvh_index = r_curve_segments->append_and_get_index(segment);
      BLI_bvhtree_insert(bvhtree, bvh_index, reinterpret_cast<float *>(&segment), 2);
    }
    if (cyclic[curve_i]) {
      Segment segment;
      segment.is_cyclic_segment = true;
      segment.is_first_segment = false;
      segment.pos_index = segment_count;
      segment.curve_index = curve_i;
      segment.curve_length = curve_length;
      segment.orig_start = positions.last();
      segment.orig_end = positions.first();
      segment.start = project ? project_v3_plane(segment.orig_start, project_axis) :
                                segment.orig_start;
      segment.end = project ? project_v3_plane(segment.orig_end, project_axis) : segment.orig_end;
      segment.len_start = lengths[segment_count - 1];
      segment.len_end = 1.0f;
      const int bvh_index = r_curve_segments->append_and_get_index(segment);
      BLI_bvhtree_insert(bvhtree, bvh_index, reinterpret_cast<float *>(&segment), 2);
    }
  }

  BLI_bvhtree_balance(bvhtree);

  return bvhtree;
}

/* Based on isect_line_plane_v3 with additional check that lines cross between start and end
 * points. It also stores the lambda. */
static bool isect_line_plane_v3_crossing(const float3 point_1,
                                         const float3 point_2,
                                         const float3 surface_center,
                                         const float3 surface_normal,
                                         float3 &r_isect_co,
                                         float &lambda)
{
  const float3 u = point_2 - point_1;
  const float dot = math::dot(surface_normal, u);

  /* The segment is parallel to plane */
  if (math::abs(dot) <= FLT_EPSILON) {
    return false;
  }
  const float3 h = point_1 - surface_center;
  lambda = -math::dot(surface_normal, h) / dot;

  /* Test lambda to check intersection is between the start and end points. */
  if (lambda >= -curve_isect_eps && lambda <= 1.0f + curve_isect_eps) {
    /* Remove epsilon from lambda. */
    lambda = math::clamp(lambda, 0.0f, 1.0f);
    r_isect_co = point_1 + u * lambda;
    return true;
  }

  return false;
}

/* Calculate intersections between a curve and a plane. */
static void set_curve_intersections_plane(const bke::CurvesGeometry &src_curves,
                                          const float3 plane_center,
                                          const float3 direction,
                                          const AttributeOutputs &attribute_outputs,
                                          IntersectionData &r_data)
{
  const VArray<bool> cyclic = src_curves.cyclic();
  const OffsetIndices evaluated_points_by_curve = src_curves.evaluated_points_by_curve();
  src_curves.ensure_evaluated_lengths();

  /* Loop each curve for intersections. */
  ThreadLocalData thread_storage;
  threading::parallel_for(src_curves.curves_range(), 128, [&](IndexRange curve_range) {
    IntersectionData &local_data = thread_storage.local();
    threading::isolate_task([&]() {
      for (const int64_t curve_i : curve_range) {
        const IndexRange points = evaluated_points_by_curve[curve_i];
        const Span<float3> positions = src_curves.evaluated_positions().slice(points);
        if (positions.size() <= 1) {
          continue;
        }
        const Span<float> lengths = src_curves.evaluated_lengths_for_curve(curve_i,
                                                                           cyclic[curve_i]);
        const float length = src_curves.evaluated_length_total_for_curve(curve_i, cyclic[curve_i]);

        auto add_closest = [&](const float3 a,
                               const float3 b,
                               const float len_start,
                               const float len_end,
                               const float curve_length) {
          float3 closest = float3(0.0f);
          float lambda = 0.0f;
          if (isect_line_plane_v3_crossing(a, b, plane_center, direction, closest, lambda)) {
            const float len_at_isect = math::interpolate(len_start, len_end, lambda);
            const float3 dir_of_isect = math::normalize(b - a);
            add_intersection_data(local_data,
                                  closest,
                                  curve_i,
                                  dir_of_isect,
                                  len_at_isect,
                                  curve_length,
                                  float3(0.0f),
                                  false,
                                  attribute_outputs);
          }
        };

        /* Loop segments from start until we have an intersection. */
        for (const int index : IndexRange(positions.size()).drop_back(1)) {
          const float3 a = positions[index];
          const float3 b = positions[1 + index];
          const float len_start = (index == 0) ? 0.0f : lengths[index - 1];
          const float len_end = lengths[index];
          add_closest(a, b, len_start, len_end, length);
        }
        if (cyclic[curve_i]) {
          const float3 a = positions.last();
          const float3 b = positions.first();
          const float len_start = lengths.last();
          const float len_end = 1.0f;
          add_closest(a, b, len_start, len_end, length);
        }
      }
    });
  });
  gather_thread_storage(thread_storage, r_data, attribute_outputs);
}

/* Calculate intersections between 3d curves, optionally projected onto 2d plane. */
static void set_curve_intersections(const bke::CurvesGeometry &src_curves,
                                    const bool self_intersect,
                                    const bool all_intersect,
                                    const float distance,
                                    const bool project,
                                    const float3 direction,
                                    const PairData &pair_data,
                                    const AttributeOutputs &attribute_outputs,
                                    IntersectionData &r_data)
{
  /* Build bvh. */
  Vector<Segment> curve_segments;
  BVHTree *bvhtree = create_curve_segment_bvhtree(src_curves, &curve_segments, project, direction);
  BLI_SCOPED_DEFER([&]() { BLI_bvhtree_free(bvhtree); });

  const float max_distance = math::max(curve_isect_eps, distance);

  /* Loop through segments. */
  ThreadLocalData thread_storage;
  threading::parallel_for(curve_segments.index_range(), 128, [&](IndexRange range) {
    IntersectionData &local_data = thread_storage.local();
    threading::isolate_task([&]() {
      for (const int64_t segment_index : range) {
        const Segment ab = curve_segments[segment_index];
        BLI_bvhtree_range_query_cpp(
            *bvhtree,
            math::midpoint(ab.start, ab.end),
            math::distance(ab.start, ab.end) + max_distance,
            [&](const int index, const float3 & /*co*/, const float /*dist_sq*/) {
              if (segment_index <= index) {
                /* Skip matching segments or previously matched segments. */
                return;
              }
              const Segment cd = curve_segments[index];
              const bool same_curve = ab.curve_index == cd.curve_index;
              const bool calc_all = (all_intersect && !same_curve);
              /* Skip adjecent segments in same curve. */
              const bool calc_self = (self_intersect && same_curve &&
                                      ((math::abs(ab.pos_index - cd.pos_index) > 1) &&
                                       !((ab.is_first_segment && cd.is_cyclic_segment) ||
                                         (cd.is_first_segment && ab.is_cyclic_segment))));
              if (calc_all || calc_self) {
                const IntersectingLineInfo isectinfo = intersecting_lines(
                    ab.start, ab.end, cd.start, cd.end, max_distance);

                if (isectinfo.intersects) {
                  const float3 closest_ab = math::interpolate(
                      ab.orig_start, ab.orig_end, isectinfo.lambda_ab);
                  const float3 closest_cd = math::interpolate(
                      cd.orig_start, cd.orig_end, isectinfo.lambda_cd);
                  const bool pair_weight = ab.curve_index > cd.curve_index;
                  add_intersection_data(
                      local_data,
                      closest_ab,
                      ab.curve_index,
                      math::normalize(ab.end - ab.start),
                      math::interpolate(ab.len_start, ab.len_end, isectinfo.lambda_ab),
                      ab.curve_length,
                      closest_cd,
                      !pair_weight,
                      attribute_outputs);
                  /* Always return both intersection points on same curve. */
                  if (pair_data == PairData::Paired) {
                    add_intersection_data(
                        local_data,
                        closest_cd,
                        cd.curve_index,
                        math::normalize(cd.end - cd.start),
                        math::interpolate(cd.len_start, cd.len_end, isectinfo.lambda_cd),
                        cd.curve_length,
                        closest_ab,
                        pair_weight,
                        attribute_outputs);
                  }
                }
              }
            });
      }
    });
  });
  gather_thread_storage(thread_storage, r_data, attribute_outputs);
}

/* Calculate intersections between curve and mesh surface. */
static void set_curve_intersections_mesh(GeometrySet &mesh_set,
                                         const bke::CurvesGeometry &src_curves,
                                         const AttributeOutputs &attribute_outputs,
                                         IntersectionData &r_data)
{
  /* Build bvh. */
  Vector<Segment> curve_segments;
  BVHTree *bvhtree = create_curve_segment_bvhtree(
      src_curves, &curve_segments, false, float3(0.0f));
  BLI_SCOPED_DEFER([&]() { BLI_bvhtree_free(bvhtree); });

  /* Loop mesh data. */
  mesh_set.modify_geometry_sets([&](GeometrySet &mesh_set) {
    if (!mesh_set.has_mesh()) {
      return;
    }
    const Mesh &mesh = *mesh_set.get_mesh();
    if (mesh.faces_num < 1) {
      return;
    }
    const Span<float3> positions = mesh.vert_positions();
    const Span<int> corner_verts = mesh.corner_verts();
    const Span<int3> corner_tris = mesh.corner_tris();

    /* Loop face data. */
    ThreadLocalData thread_storage;
    threading::parallel_for(corner_tris.index_range(), 128, [&](IndexRange range) {
      IntersectionData &local_data = thread_storage.local();
      threading::isolate_task([&]() {
        for (const int64_t face_index : range) {
          const int3 &tri = corner_tris[face_index];
          const int v0_loop = tri[0];
          const int v1_loop = tri[1];
          const int v2_loop = tri[2];
          const float3 &v0_pos = positions[corner_verts[v0_loop]];
          const float3 &v1_pos = positions[corner_verts[v1_loop]];
          const float3 &v2_pos = positions[corner_verts[v2_loop]];

          float3 center_pos;
          interp_v3_v3v3v3(center_pos, v0_pos, v1_pos, v2_pos, float3(1.0f / 3.0f));
          const float distance = math::max(
              math::max(math::distance(center_pos, v0_pos), math::distance(center_pos, v1_pos)),
              math::distance(center_pos, v2_pos));
          BLI_bvhtree_range_query_cpp(
              *bvhtree,
              center_pos,
              distance + curve_isect_eps,
              [&](const int index, const float3 & /*co*/, const float /*dist_sq*/) {
                const Segment seg = curve_segments[index];
                float lambda = 0.0f;
                if (isect_line_segment_tri_epsilon_v3(seg.start,
                                                      seg.end,
                                                      v0_pos,
                                                      v1_pos,
                                                      v2_pos,
                                                      &lambda,
                                                      nullptr,
                                                      curve_isect_eps))
                {
                  const float len_at_isect = math::interpolate(seg.len_start, seg.len_end, lambda);
                  const float3 dir_of_isect = math::normalize(seg.end - seg.start);
                  const float3 closest_position = math::interpolate(seg.start, seg.end, lambda);
                  add_intersection_data(local_data,
                                        closest_position,
                                        seg.curve_index,
                                        dir_of_isect,
                                        len_at_isect,
                                        seg.curve_length,
                                        float3(0.0f),
                                        false,
                                        attribute_outputs);
                }
              });
        }
      });
    });
    gather_thread_storage(thread_storage, r_data, attribute_outputs);
  });
}

/* Sorting intersections by sortkey (which is curve_index and factor postion on curve). */
static IntersectionData sort_intersection_data(IntersectionData &data,
                                               const AttributeOutputs &attribute_outputs)
{
  const int64_t data_size = data.position.size();

  Vector<std::pair<int64_t, float>> sort_index;
  sort_index.reserve(data_size);

  for (int64_t i = 0; i < data_size; i++) {
    sort_index.append(std::pair(i, data.sortkey[i]));
  }

  std::sort(sort_index.begin(),
            sort_index.end(),
            [&](const std::pair<int64_t, float> &a, const std::pair<int64_t, float> &b) {
              return (a.second < b.second);
            });

  /* Ignore sortdata for return data. */
  IntersectionData r_data;
  r_data.position.reserve(data_size);
  if (attribute_outputs.curve_index) {
    r_data.curve_index.reserve(data_size);
  }
  if (attribute_outputs.length) {
    r_data.length.reserve(data_size);
  }
  if (attribute_outputs.factor) {
    r_data.factor.reserve(data_size);
  }
  if (attribute_outputs.direction) {
    r_data.direction.reserve(data_size);
  }
  if (attribute_outputs.pair_position) {
    r_data.pair_position.reserve(data_size);
  }
  if (attribute_outputs.pair) {
    r_data.pair.reserve(data_size);
  }

  for (const std::pair key_val : sort_index) {
    const int64_t key_index = key_val.first;
    r_data.position.append(data.position[key_index]);
    if (attribute_outputs.curve_index) {
      r_data.curve_index.append(data.curve_index[key_index]);
    }
    if (attribute_outputs.length) {
      r_data.length.append(data.length[key_index]);
    }
    if (attribute_outputs.factor) {
      r_data.factor.append(data.factor[key_index]);
    }
    if (attribute_outputs.direction) {
      r_data.direction.append(data.direction[key_index]);
    }
    if (attribute_outputs.pair_position) {
      r_data.pair_position.append(data.pair_position[key_index]);
    }
    if (attribute_outputs.pair) {
      r_data.pair.append(data.pair[key_index]);
    }
  }
  BLI_assert(data.position.size() == r_data.position.size());
  return r_data;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeometryNodeCurveIntersectionMode mode = GeometryNodeCurveIntersectionMode(
      params.node().custom1);
  const PairData pair_data = PairData(params.node().custom2);
  const bool curve_mode = ELEM(
      mode, GEO_NODE_CURVE_INTERSECT_CURVE, GEO_NODE_CURVE_INTERSECT_PROJECT);

  GeometrySet geometry_set = params.extract_input<GeometrySet>("Curve");
  GeometryComponentEditData::remember_deformed_positions_if_necessary(geometry_set);

  lazy_threading::send_hint();

  AttributeOutputs attribute_outputs;
  attribute_outputs.direction = params.get_output_anonymous_attribute_id_if_needed("Direction");
  attribute_outputs.curve_index = params.get_output_anonymous_attribute_id_if_needed(
      "Curve Index");
  attribute_outputs.factor = params.get_output_anonymous_attribute_id_if_needed("Factor");
  attribute_outputs.length = params.get_output_anonymous_attribute_id_if_needed("Length");
  if (curve_mode) {
    attribute_outputs.pair_position = params.get_output_anonymous_attribute_id_if_needed(
        "Pair Position");
  }
  if (curve_mode && pair_data == PairData::Paired) {
    attribute_outputs.pair = params.get_output_anonymous_attribute_id_if_needed("Pair");
  }

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (!geometry_set.has_curves()) {
      return;
    }
    const Curves &src_curves_id = *geometry_set.get_curves();
    const bke::CurvesGeometry &src_curves = src_curves_id.geometry.wrap();

    if (src_curves.curves_range().is_empty()) {
      return;
    }

    IntersectionData r_data;

    switch (mode) {
      case GEO_NODE_CURVE_INTERSECT_CURVE: {
        const float distance = params.extract_input<float>("Distance");
        const bool self = params.extract_input<bool>("Self Intersections");
        const bool all = params.extract_input<bool>("All Intersections");
        set_curve_intersections(src_curves,
                                self,
                                all,
                                distance,
                                false,
                                float3(0.0f),
                                pair_data,
                                attribute_outputs,
                                r_data);
        break;
      }
      case GEO_NODE_CURVE_INTERSECT_PROJECT: {
        const float3 direction = params.extract_input<float3>("Direction");
        const bool self = params.extract_input<bool>("Self Intersections");
        const bool all = params.extract_input<bool>("All Intersections");
        set_curve_intersections(
            src_curves, self, all, 0.0f, true, direction, pair_data, attribute_outputs, r_data);
        break;
      }
      case GEO_NODE_CURVE_INTERSECT_PLANE: {
        const float3 direction = params.extract_input<float3>("Direction");
        const float3 plane_center = params.extract_input<float3>("Center");
        set_curve_intersections_plane(
            src_curves, plane_center, direction, attribute_outputs, r_data);
        break;
      }
      case GEO_NODE_CURVE_INTERSECT_SURFACE: {
        GeometrySet mesh_set = params.extract_input<GeometrySet>("Mesh");
        if (mesh_set.has_mesh()) {
          set_curve_intersections_mesh(mesh_set, src_curves, attribute_outputs, r_data);
        }
        break;
      }
      default: {
        BLI_assert_unreachable();
        break;
      }
    }

    geometry_set.remove_geometry_during_modify();

    /* Gather and sort data for attributes. */
    if (r_data.position.size() > 0) {

      IntersectionData sorted_data = sort_intersection_data(r_data, attribute_outputs);

      PointCloud *pointcloud = BKE_pointcloud_new_nomain(sorted_data.position.size());
      MutableAttributeAccessor point_attributes = pointcloud->attributes_for_write();

      geometry_set.replace_pointcloud(pointcloud);

      SpanAttributeWriter<float3> point_positions =
          point_attributes.lookup_or_add_for_write_only_span<float3>("position",
                                                                     AttrDomain::Point);
      point_positions.span.copy_from(sorted_data.position);
      point_positions.finish();

      SpanAttributeWriter<float> point_radii =
          point_attributes.lookup_or_add_for_write_only_span<float>("radius", AttrDomain::Point);
      point_radii.span.fill(0.05f);
      point_radii.finish();

      if (attribute_outputs.curve_index) {
        SpanAttributeWriter<int> curve_index;
        curve_index = point_attributes.lookup_or_add_for_write_only_span<int>(
            *attribute_outputs.curve_index, AttrDomain::Point);
        curve_index.span.copy_from(sorted_data.curve_index);
        curve_index.finish();
      }

      if (attribute_outputs.direction) {
        SpanAttributeWriter<float3> directions;
        directions = point_attributes.lookup_or_add_for_write_only_span<float3>(
            *attribute_outputs.direction, AttrDomain::Point);
        directions.span.copy_from(sorted_data.direction);
        directions.finish();
      }

      if (attribute_outputs.factor) {
        SpanAttributeWriter<float> factor;
        factor = point_attributes.lookup_or_add_for_write_only_span<float>(
            *attribute_outputs.factor, AttrDomain::Point);
        factor.span.copy_from(sorted_data.factor);
        factor.finish();
      }

      if (attribute_outputs.length) {
        SpanAttributeWriter<float> length;
        length = point_attributes.lookup_or_add_for_write_only_span<float>(
            *attribute_outputs.length, AttrDomain::Point);
        length.span.copy_from(sorted_data.length);
        length.finish();
      }

      if (attribute_outputs.pair_position) {
        SpanAttributeWriter<float3> pair_position;
        pair_position = point_attributes.lookup_or_add_for_write_only_span<float3>(
            *attribute_outputs.pair_position, AttrDomain::Point);
        pair_position.span.copy_from(sorted_data.pair_position);
        pair_position.finish();
      }

      if (attribute_outputs.pair && pair_data == PairData::Paired) {
        SpanAttributeWriter<bool> pair;
        pair = point_attributes.lookup_or_add_for_write_only_span<bool>(*attribute_outputs.pair,
                                                                        AttrDomain::Point);
        pair.span.copy_from(sorted_data.pair);
        pair.finish();
      }
    }
  });

  params.set_output("Points", std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  static EnumPropertyItem mode_items[] = {
      {GEO_NODE_CURVE_INTERSECT_CURVE,
       "CURVE",
       0,
       "Curve",
       "Find the intersection positions between curves in 3d space"},
      {GEO_NODE_CURVE_INTERSECT_PLANE,
       "PLANE",
       0,
       "Plane",
       "Find all the intersection positions for each curve in reference to a plane"},
      {GEO_NODE_CURVE_INTERSECT_SURFACE,
       "SURFACE",
       0,
       "Surface",
       "Find all the intersection positions for each curve in reference to a mesh surface"},
      {GEO_NODE_CURVE_INTERSECT_PROJECT,
       "PROJECT",
       0,
       "Project",
       "Find all the intersection positions for all curves projected onto orthographic plane"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "Method to find intersection positions for the spline",
                    mode_items,
                    NOD_inline_enum_accessors(custom1));

  static EnumPropertyItem pair_items[] = {
      {int(PairData::Single),
       "SINGLE",
       0,
       "Single",
       "Return first intersection found, weighted to lowest curve id"},
      {int(PairData::Paired), "PAIRED", 0, "Paired", "Return all paired intersections"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "use_paired_data",
                    "Use Paired Data",
                    "Return either single or paired data",
                    pair_items,
                    NOD_inline_enum_accessors(custom2));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_CURVE_INTERSECTIONS, "Curve Intersections", NODE_CLASS_GEOMETRY);
  bke::node_type_size_preset(&ntype, bke::eNodeSizePreset::Middle);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.updatefunc = node_update;
  blender::bke::node_register_type(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_curve_intersection_cc
