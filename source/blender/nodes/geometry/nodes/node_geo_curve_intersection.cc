/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

#include "DNA_pointcloud_types.h"

#include "BKE_curves.hh"
#include "BKE_pointcloud.hh"

#include "BLI_bounds.hh"
#include "BLI_kdopbvh.h"
#include "BLI_math_geom.h"
#include "BLI_task.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_intersection_cc {

#define CURVE_ISECT_EPS 0.000001f

NODE_STORAGE_FUNCS(NodeGeometryCurveIntersections)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Curve").supported_type(GeometryComponent::Type::Curve);
  b.add_input<decl::Bool>("Self Intersection")
      .default_value(false)
      .description("Include self intersections");
  b.add_input<decl::Vector>("Direction")
      .default_value({0.0f, 0.0f, 1.0f})
      .make_available(
          [](bNode &node) { node_storage(node).mode = GEO_NODE_CURVE_INTERSECT_PLANE; })
      .description("Direction of plane");
  b.add_input<decl::Vector>("Plane Offset").subtype(PROP_DISTANCE).make_available([](bNode &node) {
    node_storage(node).mode = GEO_NODE_CURVE_INTERSECT_PLANE;
  });
  b.add_input<decl::Float>("Distance")
      .subtype(PROP_DISTANCE)
      .min(0.0f)
      .description("Distance between intersections");
  b.add_input<decl::Float>("Offset")
      .subtype(PROP_DISTANCE)
      .description("Offset intersection points by this distance");
  b.add_output<decl::Geometry>("Points");
  b.add_output<decl::Int>("Curve Index").field_on_all();
  b.add_output<decl::Float>("Factor").field_on_all();
  b.add_output<decl::Float>("Length").field_on_all();
  b.add_output<decl::Vector>("Direction").field_on_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryCurveIntersections *data = MEM_cnew<NodeGeometryCurveIntersections>(__func__);

  data->mode = GEO_NODE_CURVE_INTERSECT_SELF;
  node->storage = data;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const NodeGeometryCurveIntersections &storage = node_storage(*node);
  const GeometryNodeCurveIntersectionMode mode = GeometryNodeCurveIntersectionMode(storage.mode);

  bNodeSocket *self = static_cast<bNodeSocket *>(node->inputs.first)->next;
  bNodeSocket *direction = self->next;
  bNodeSocket *plane_offset = direction->next;
  bNodeSocket *distance = plane_offset->next;

  bke::node_set_socket_availability(ntree, self, mode == GEO_NODE_CURVE_INTERSECT_ALL);
  bke::node_set_socket_availability(ntree, direction, mode == GEO_NODE_CURVE_INTERSECT_PLANE);
  bke::node_set_socket_availability(ntree, plane_offset, mode == GEO_NODE_CURVE_INTERSECT_PLANE);
  bke::node_set_socket_availability(ntree, distance, mode != GEO_NODE_CURVE_INTERSECT_PLANE);
}

struct IntersectionData {
  Vector<float3> position;
  Vector<int> curve_id;
  Vector<float> length;
  Vector<float> factor;
  Vector<float3> direction;
};
using ThreadLocalData = threading::EnumerableThreadSpecific<IntersectionData>;

static void add_intersection_data(IntersectionData &data,
                                  const float3 position,
                                  const float3 direction,
                                  const int curve_id,
                                  const float length,
                                  const float curve_length)
{
  data.position.append(position);
  data.curve_id.append(curve_id);
  data.length.append(length);
  data.factor.append(math::safe_divide(length, curve_length));
  data.direction.append(direction);
}

static void gather_thread_storage(ThreadLocalData &thread_storage, IntersectionData &r_data)
{
  int64_t total_intersections = 0;
  for (const IntersectionData &local_data : thread_storage) {
    const int64_t local_size = local_data.position.size();
    BLI_assert(local_data.curve_id.size() == local_size);
    BLI_assert(local_data.length.size() == local_size);
    BLI_assert(local_data.factor.size() == local_size);
    BLI_assert(local_data.direction.size() == local_size);
    total_intersections += local_size;
  }
  const int64_t start_index = r_data.position.size();
  const int64_t new_size = start_index + total_intersections;
  r_data.position.reserve(new_size);
  r_data.curve_id.reserve(new_size);
  r_data.length.reserve(new_size);
  r_data.factor.reserve(new_size);
  r_data.direction.reserve(new_size);

  for (IntersectionData &local_data : thread_storage) {
    const int64_t local_size = local_data.position.size();
    r_data.position.extend(local_data.position);
    r_data.curve_id.extend(local_data.curve_id);
    r_data.length.extend(local_data.length);
    r_data.factor.extend(local_data.factor);
    r_data.direction.extend(local_data.direction);
  }
}

struct IntersectingLineInfo {
  float3 isect_point_ab;
  float3 isect_point_cd;
  float3 closest_ab;
  float3 closest_cd;
  float lambda_ab;
  float lambda_cd;
  bool intersects;
};

struct Segment {
  float3 start;
  float3 end;
  float len_start;
  float len_end;
  bool is_end_segment;
  int pos_index;
  int curve_index;
  float curve_length;
};

/* Check intersection between line ab and line cd. */
static IntersectingLineInfo intersecting_lines(
    const float3 &a, const float3 &b, const float3 &c, const float3 &d, const float distance)
{
  IntersectingLineInfo isectinfo{};
  if (isect_line_line_v3(a, b, c, d, isectinfo.isect_point_ab, isectinfo.isect_point_cd)) {
    if (math::distance(isectinfo.isect_point_ab, isectinfo.isect_point_cd) > distance) {
      isectinfo.intersects = false;
      return isectinfo;
    }
    /* Check intersection is on both line segments ab and cd. */
    isectinfo.lambda_ab = closest_to_line_v3(isectinfo.closest_ab, isectinfo.isect_point_ab, a, b);
    if (isectinfo.lambda_ab < 0.0f || isectinfo.lambda_ab > 1.0f) {
      isectinfo.intersects = false;
      return isectinfo;
    }
    isectinfo.lambda_cd = closest_to_line_v3(isectinfo.closest_cd, isectinfo.isect_point_cd, c, d);
    if (isectinfo.lambda_cd < 0.0f || isectinfo.lambda_cd > 1.0f) {
      isectinfo.intersects = false;
      return isectinfo;
    }
    if (math::distance(isectinfo.closest_ab, isectinfo.closest_cd) <= distance) {
      isectinfo.intersects = true;
      return isectinfo;
    }
  }
  isectinfo.intersects = false;
  return isectinfo;
}

/* Buuild curve segment bvh. */
static BVHTree *create_curve_segment_bvhtree(const bke::CurvesGeometry &src_curves,
                                             Vector<Segment> *r_curve_segments)
{
  src_curves.ensure_evaluated_lengths();

  const int bvh_points_num = src_curves.evaluated_points_num() + src_curves.curves_num();
  BVHTree *bvhtree = BLI_bvhtree_new(bvh_points_num, CURVE_ISECT_EPS, 8, 8);
  const VArray<bool> cyclic = src_curves.cyclic();
  const OffsetIndices evaluated_points_by_curve = src_curves.evaluated_points_by_curve();

  /* Preprocess curve segments. */
  for (const int64_t curve_i : src_curves.curves_range()) {
    const IndexRange points = evaluated_points_by_curve[curve_i];
    const Span<float3> positions = src_curves.evaluated_positions().slice(points);
    const Span<float> lengths = src_curves.evaluated_lengths_for_curve(curve_i, cyclic[curve_i]);
    const float curve_length = src_curves.evaluated_length_total_for_curve(curve_i,
                                                                           cyclic[curve_i]);
    const int totpoints = positions.size() - 1;
    const int loopcount = cyclic[curve_i] ? totpoints + 1 : totpoints;
    for (const int index : IndexRange(loopcount)) {
      const bool cyclic_segment = (cyclic[curve_i] && index == totpoints);
      Segment segment;
      segment.is_end_segment = (index == 0) || cyclic_segment;
      segment.pos_index = index;
      segment.curve_index = curve_i;
      segment.curve_length = curve_length;
      segment.start = cyclic_segment ? positions.last() : positions[index];
      segment.end = cyclic_segment ? positions.first() : positions[1 + index];
      segment.len_start = (index == 0) ? 0.0f : lengths[index - 1];
      segment.len_end = cyclic_segment ? 1.0f : lengths[index];
      const int bvh_index = r_curve_segments->append_and_get_index(segment);
      BLI_bvhtree_insert(bvhtree, bvh_index, reinterpret_cast<float *>(&segment), 2);
    }
  }

  BLI_bvhtree_balance(bvhtree);

  return bvhtree;
}

/* Based on isect_line_plane_v3 with additional check that lines cross between start and end
 * points. It also stores the lambda. */
static bool isect_line_plane_crossing(const float3 point_1,
                                      const float3 point_2,
                                      const float3 surface_center,
                                      const float3 surface_normal,
                                      float3 &r_isect_co,
                                      float &lambda)
{
  const float3 u = point_2 - point_1;
  const float dot = math::dot<float>(surface_normal, u);

  /* The segment is parallel to plane */
  if (math::abs<float>(dot) <= FLT_EPSILON) {
    return false;
  }
  const float3 h = point_1 - surface_center;
  lambda = -math::dot<float>(surface_normal, h) / dot;
  r_isect_co = point_1 + u * lambda;

  /* Test lambda to check intersection is between the start and end points. */
  if (lambda >= 0.0f && lambda <= 1.0f) {
    return true;
  }

  return false;
}

static void offset_intersections(const Segment seg,
                                 const float offset,
                                 const float lambda,
                                 const float3 closest,
                                 IntersectionData &r_data)
{
  const float3 dir_of_isect = math::normalize(seg.end - seg.start);
  const float3 off = dir_of_isect * offset;
  const float len_at_isect = math::interpolate(seg.len_start, seg.len_end, lambda);

  if (lambda == 0.0f) {
    add_intersection_data(
        r_data, closest + off, dir_of_isect, seg.curve_index, len_at_isect, seg.curve_length);
  }
  else if (lambda < 1.0f) {
    add_intersection_data(
        r_data, closest + off, dir_of_isect, seg.curve_index, len_at_isect, seg.curve_length);
    add_intersection_data(
        r_data, closest - off, dir_of_isect, seg.curve_index, len_at_isect, seg.curve_length);
  }
  else if (lambda == 1.0f) {
    add_intersection_data(
        r_data, closest - off, dir_of_isect, seg.curve_index, len_at_isect, seg.curve_length);
  }
};

static void set_curve_intersections_plane(const bke::CurvesGeometry &src_curves,
                                          const float3 plane_offset,
                                          const float3 direction,
                                          const float offset,
                                          IntersectionData &r_data)
{
  const VArray<bool> cyclic = src_curves.cyclic();
  const OffsetIndices evaluated_points_by_curve = src_curves.evaluated_points_by_curve();
  src_curves.ensure_evaluated_lengths();

  threading::parallel_for(src_curves.curves_range(), 1024, [&](IndexRange curve_range) {
    for (const int64_t curve_i : curve_range) {
      const IndexRange points = evaluated_points_by_curve[curve_i];
      const Span<float3> positions = src_curves.evaluated_positions().slice(points);
      if (positions.size() <= 1) {
        continue;
      }
      const Span<float> lengths = src_curves.evaluated_lengths_for_curve(curve_i, cyclic[curve_i]);
      const float length = src_curves.evaluated_length_total_for_curve(curve_i, cyclic[curve_i]);

      auto new_closest = [&](const float3 a,
                             const float3 b,
                             const float len_start,
                             const float len_end,
                             const float curve_length) {
        float3 closest = float3(0.0f);
        float lambda = 0.0f;
        if (isect_line_plane_crossing(a, b, plane_offset, direction, closest, lambda)) {
          const float len_at_isect = math::interpolate(len_start, len_end, lambda);
          const float3 dir_of_isect = math::normalize(b - a);
          if (offset != 0.0f) {
            const float3 off = dir_of_isect * offset;
            add_intersection_data(
                r_data, closest + off, dir_of_isect, curve_i, len_at_isect, curve_length);
            add_intersection_data(
                r_data, closest - off, dir_of_isect, curve_i, len_at_isect, curve_length);
          }
          else {
            add_intersection_data(
                r_data, closest, dir_of_isect, curve_i, len_at_isect, curve_length);
          }
        }
      };

      /* Loop segments from start until we have an intersection. */
      for (const int index : IndexRange(positions.size()).drop_back(1)) {
        const float3 a = positions[index];
        const float3 b = positions[1 + index];
        const float len_start = (index == 0) ? 0.0f : lengths[index - 1];
        const float len_end = lengths[index];
        new_closest(a, b, len_start, len_end, length);
      }
      if (cyclic[curve_i]) {
        const float3 a = positions.last();
        const float3 b = positions.first();
        const float len_start = lengths.last();
        const float len_end = 1.0f;
        new_closest(a, b, len_start, len_end, length);
      }
    }
  });
}

static void set_curve_intersections(const bke::CurvesGeometry &src_curves,
                                    const bool self_intersect,
                                    const bool all_intersect,
                                    const float distance,
                                    const float offset,
                                    IntersectionData &r_data)
{
  /* Build bvh. */
  Vector<Segment> curve_segments;
  BVHTree *bvhtree = create_curve_segment_bvhtree(src_curves, &curve_segments);
  float min_distance = math::max(CURVE_ISECT_EPS, distance);

  /* Loop through segments. */
  ThreadLocalData thread_storage;
  threading::parallel_for(curve_segments.index_range(), 128, [&](IndexRange range) {
    for (const int64_t segment_index : range) {
      IntersectionData &data = thread_storage.local();
      const Segment ab = curve_segments[segment_index];
      BLI_bvhtree_range_query_cpp(
          *bvhtree,
          math::midpoint(ab.start, ab.end),
          math::distance(ab.start, ab.end) + min_distance,
          [&](const int index, const float3 & /*co*/, const float /*dist_sq*/) {
            if (segment_index <= index) {
              /* Skip matching segments or previously matched segments. */
              return;
            }
            const Segment cd = curve_segments[index];
            const bool calc_self = (self_intersect && (ab.curve_index == cd.curve_index &&
                                                       (abs(ab.pos_index - cd.pos_index) > 1) &&
                                                       !(ab.is_end_segment && cd.is_end_segment)));
            const bool calc_all = (all_intersect && (ab.curve_index != cd.curve_index));
            if (calc_self || calc_all) {
              const IntersectingLineInfo isectinfo = intersecting_lines(
                  ab.start, ab.end, cd.start, cd.end, min_distance);
              if (offset != 0.0f) {
                if (isectinfo.intersects) {
                  offset_intersections(
                      ab, offset, isectinfo.lambda_ab, isectinfo.closest_ab, data);
                  offset_intersections(
                      cd, offset, isectinfo.lambda_cd, isectinfo.closest_cd, data);
                }
              }
              else {
                if (isectinfo.intersects && isectinfo.lambda_ab != 1.0f &&
                    isectinfo.lambda_cd != 1.0f) {
                  add_intersection_data(
                      data,
                      isectinfo.closest_ab,
                      math::normalize(ab.end - ab.start),
                      ab.curve_index,
                      math::interpolate(ab.len_start, ab.len_end, isectinfo.lambda_ab),
                      ab.curve_length);
                  add_intersection_data(
                      data,
                      isectinfo.closest_cd,
                      math::normalize(cd.end - cd.start),
                      cd.curve_index,
                      math::interpolate(cd.len_start, cd.len_end, isectinfo.lambda_cd),
                      cd.curve_length);
                }
              }
            }
          });
    }
  });
  gather_thread_storage(thread_storage, r_data);

  BLI_SCOPED_DEFER([&]() { BLI_bvhtree_free(bvhtree); });
}

struct AttributeOutputs {
  std::optional<std::string> curve_index_attr_id;
  std::optional<std::string> factor_attr_id;
  std::optional<std::string> length_attr_id;
  std::optional<std::string> direction_attr_id;
};

static void node_geo_exec(GeoNodeExecParams params)
{
  const NodeGeometryCurveIntersections &storage = node_storage(params.node());
  const GeometryNodeCurveIntersectionMode mode = GeometryNodeCurveIntersectionMode(storage.mode);

  GeometrySet geometry_set = params.extract_input<GeometrySet>("Curve");
  GeometryComponentEditData::remember_deformed_positions_if_necessary(geometry_set);

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
    const float offset = params.extract_input<float>("Offset");
    switch (mode) {
      case GEO_NODE_CURVE_INTERSECT_SELF: {
        const float distance = params.extract_input<float>("Distance");
        set_curve_intersections(src_curves, true, false, distance, offset, r_data);
        break;
      }
      case GEO_NODE_CURVE_INTERSECT_ALL: {
        const float distance = params.extract_input<float>("Distance");
        const bool self = params.extract_input<bool>("Self Intersection");
        set_curve_intersections(src_curves, self, true, distance, offset, r_data);
        break;
      }
      case GEO_NODE_CURVE_INTERSECT_PLANE: {
        const float3 direction = params.extract_input<float3>("Direction");
        const float3 plane_offset = params.extract_input<float3>("Plane Offset");
        set_curve_intersections_plane(src_curves, plane_offset, direction, offset, r_data);
        break;
      }
      default: {
        BLI_assert_unreachable();
        break;
      }
    }

    geometry_set.remove_geometry_during_modify();

    PointCloud *pointcloud = BKE_pointcloud_new_nomain(r_data.position.size());
    MutableAttributeAccessor point_attributes = pointcloud->attributes_for_write();

    AttributeOutputs attribute_outputs;
    attribute_outputs.curve_index_attr_id = params.get_output_anonymous_attribute_id_if_needed(
        "Curve Index");
    attribute_outputs.factor_attr_id = params.get_output_anonymous_attribute_id_if_needed(
        "Factor");
    attribute_outputs.length_attr_id = params.get_output_anonymous_attribute_id_if_needed(
        "Length");
    attribute_outputs.direction_attr_id = params.get_output_anonymous_attribute_id_if_needed(
        "Direction");

    geometry_set.replace_pointcloud(pointcloud);

    SpanAttributeWriter<float3> point_positions =
        point_attributes.lookup_or_add_for_write_only_span<float3>("position", AttrDomain::Point);
    point_positions.span.copy_from(r_data.position);
    point_positions.finish();

    point_attributes.add<int>("Curve Index",
                              bke::AttrDomain::Point,
                              bke::AttributeInitVArray(VArray<int>::ForSpan(r_data.curve_id)));

    point_attributes.add<float>(
        "Factor",
        AttrDomain::Point,
        blender::bke::AttributeInitVArray(VArray<float>::ForSpan(r_data.factor)));

    point_attributes.add<float>(
        "Length",
        AttrDomain::Point,
        blender::bke::AttributeInitVArray(VArray<float>::ForSpan(r_data.length)));

    point_attributes.add<float3>(
        "Direction",
        AttrDomain::Point,
        bke::AttributeInitVArray(VArray<float3>::ForSpan(r_data.direction)));
  });

  params.set_output("Points", std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  static EnumPropertyItem mode_items[] = {
      {GEO_NODE_CURVE_INTERSECT_SELF,
       "SELF",
       0,
       "Self",
       "Find the self intersection positions for each curve"},
      {GEO_NODE_CURVE_INTERSECT_ALL,
       "ALL",
       0,
       "All",
       "Find all the intersection positions for all curves"},
      {GEO_NODE_CURVE_INTERSECT_PLANE,
       "PLANE",
       0,
       "Plane",
       "Find all the intersection positions for each curve in reference to a plane"},
      {0, NULL, 0, NULL, NULL},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "How to find intersection positions for the spline",
                    mode_items,
                    NOD_storage_enum_accessors(mode));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_CURVE_INTERSECTIONS, "Curve Intersections", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.updatefunc = node_update;
  blender::bke::node_type_storage(&ntype,
                                  "NodeGeometryCurveIntersections",
                                  node_free_standard_storage,
                                  node_copy_standard_storage);
  blender::bke::node_register_type(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_curve_intersection_cc
