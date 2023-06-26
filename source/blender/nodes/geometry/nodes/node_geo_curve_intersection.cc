/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

#include "DNA_pointcloud_types.h"

#include "BKE_curves.hh"
#include "BKE_pointcloud.h"

#include "BLI_bounds.hh"
#include "BLI_kdopbvh.h"
#include "BLI_task.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_socket_search_link.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_intersection_cc {

#define CURVE_ISECT_EPS 0.000001f

NODE_STORAGE_FUNCS(NodeGeometryCurveIntersections)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Curve")).supported_type(GeometryComponent::Type::Curve);
  b.add_input<decl::Bool>(N_("Self Intersection"))
      .default_value(false)
      .description(N_("Include self intersections"));
  b.add_input<decl::Vector>(N_("Direction"))
      .make_available(
          [](bNode &node) { node_storage(node).mode = GEO_NODE_CURVE_INTERSECT_PLANE; })
      .description(N_("Direction of plane"));
  b.add_input<decl::Vector>(N_("Plane Offset"))
      .subtype(PROP_DISTANCE)
      .make_available(
          [](bNode &node) { node_storage(node).mode = GEO_NODE_CURVE_INTERSECT_PLANE; })
      .description(N_("Plane offset"));
  b.add_input<decl::Float>(N_("Distance"))
      .subtype(PROP_DISTANCE)
      .min(CURVE_ISECT_EPS)
      .default_value(CURVE_ISECT_EPS)
      .description(N_("Distance between intersections"));
  b.add_input<decl::Float>(N_("Offset"))
      .subtype(PROP_DISTANCE)
      .description(N_("Offset intersections by this distance"));
  b.add_output<decl::Geometry>(N_("Points"));
  b.add_output<decl::Int>(N_("Curve Index")).field_on_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", 0, "", ICON_NONE);
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
  const GeometryNodeCurveIntersectionMode mode = (GeometryNodeCurveIntersectionMode)storage.mode;

  bNodeSocket *self = static_cast<bNodeSocket *>(node->inputs.first)->next;
  bNodeSocket *direction = self->next;
  bNodeSocket *plane_offset = direction->next;
  bNodeSocket *distance = plane_offset->next;

  bke::nodeSetSocketAvailability(ntree, self, mode == GEO_NODE_CURVE_INTERSECT_ALL);
  bke::nodeSetSocketAvailability(ntree, direction, mode == GEO_NODE_CURVE_INTERSECT_PLANE);
  bke::nodeSetSocketAvailability(ntree, plane_offset, mode == GEO_NODE_CURVE_INTERSECT_PLANE);
  bke::nodeSetSocketAvailability(ntree, distance, mode != GEO_NODE_CURVE_INTERSECT_PLANE);
}

typedef std::pair<float3, int> OutputData;

struct IntersectingLineInfo {
  float3 isect_point_ab;
  float3 isect_point_cd;
  float3 closest_ab;
  float3 closest_cd;
  float lambda_ab;
  float lambda_cd;
  bool intersects;
};

struct CurveInfo {
  Span<float3> positions;
  bool cyclic;
  bool process;
};

struct Segment {
  float3 start;
  float3 end;
  bool is_end_segment;
  int pos_index;
  int curve_index;
};

struct Intersection {
  float3 position;
  int curve_index;
};

/* Check intersection between line ab and line cd. */
static IntersectingLineInfo intersecting_lines(
    const float3 &a, const float3 &b, const float3 &c, const float3 &d, const float distance)
{
  IntersectingLineInfo info{};
  if (isect_line_line_v3(a, b, c, d, info.isect_point_ab, info.isect_point_cd)) {
    if (math::distance(info.isect_point_ab, info.isect_point_cd) > distance) {
      info.intersects = false;
      return info;
    }
    /* Check intersection is on both line segments ab and cd. */
    info.lambda_ab = closest_to_line_v3(info.closest_ab, info.isect_point_ab, a, b);
    if (info.lambda_ab < 0.0f || info.lambda_ab > 1.0f) {
      info.intersects = false;
      return info;
    }
    info.lambda_cd = closest_to_line_v3(info.closest_cd, info.isect_point_cd, c, d);
    if (info.lambda_cd < 0.0f || info.lambda_cd > 1.0f) {
      info.intersects = false;
      return info;
    }
    if (math::distance(info.closest_ab, info.closest_cd) <= distance) {
      info.intersects = true;
      return info;
    }
  }
  info.intersects = false;
  return info;
}

/* Buuild curve segment bvh. */
static BVHTree *create_curve_segment_bvhtree(const bke::CurvesGeometry &src_curves,
                                             Vector<Segment> *r_curve_segments)
{
  const int bvh_points_num = src_curves.evaluated_points_num() + src_curves.curves_num();
  BVHTree *bvhtree = BLI_bvhtree_new(bvh_points_num, CURVE_ISECT_EPS, 8, 8);
  const VArray<bool> cyclic = src_curves.cyclic();

  /* Preprocess individual curves. */
  Array<CurveInfo> curve_data(src_curves.curves_num());
  threading::parallel_for(src_curves.curves_range(), 2048, [&](IndexRange curve_range) {
    for (const int64_t curve_i : curve_range) {
      CurveInfo &curveinfo = curve_data[curve_i];
      curveinfo.cyclic = cyclic[curve_i];
      curveinfo.positions = src_curves.evaluated_positions_for_curve(curve_i);
      if (curveinfo.positions.size() == 1) {
        curveinfo.process = false;
      }
      else {
        curveinfo.process = true;
      }
    }
  });

  /* Preprocess curve segments. */
  for (const int64_t curve_i : src_curves.curves_range()) {
    const CurveInfo &curveinfo = curve_data[curve_i];
    const int totpoints = curveinfo.positions.size() - 1;
    const int loopcount = curveinfo.cyclic ? totpoints + 1 : totpoints;
    for (const int index : IndexRange(loopcount)) {
      const bool cyclic_segment = (curveinfo.cyclic && index == totpoints);
      Segment segment;
      segment.is_end_segment = (index == 0) || cyclic_segment;
      segment.pos_index = index;
      segment.curve_index = curve_i;
      segment.start = cyclic_segment ? curveinfo.positions.last() : curveinfo.positions[index];
      segment.end = cyclic_segment ? curveinfo.positions.first() : curveinfo.positions[1 + index];
      const int bvh_index = r_curve_segments->append_and_get_index(segment);
      BLI_bvhtree_insert(bvhtree, bvh_index, reinterpret_cast<float *>(&segment), 2);
    }
  }

  BLI_bvhtree_balance(bvhtree);

  return bvhtree;
}

/* Based on isect_line_plane_v3 with check that lines cross between start and end points. */
static bool isect_line_plane_crossing(const float3 point_1,
                                      const float3 point_2,
                                      const float3 surface_center,
                                      const float3 surface_normal,
                                      float3 &r_isect_co)
{
  const float3 u = point_2 - point_1;
  const float dot = math::dot<float>(surface_normal, u);

  /* The segment is parallel to plane */
  if (math::abs<float>(dot) <= FLT_EPSILON) {
    return false;
  }
  const float3 h = point_1 - surface_center;
  const float lambda = -math::dot<float>(surface_normal, h) / dot;
  r_isect_co = point_1 + u * lambda;

  /* Test lambda to check intersection is between the start and end points. */
  if (lambda >= 0.0f && lambda <= 1.0f) {
    return true;
  }

  return false;
}

static void curve_plane_intersections_to_points(const CurveInfo curveinfo,
                                                const float3 direction,
                                                const float3 plane_offset,
                                                const float offset,
                                                const int curve_id,
                                                Vector<OutputData> &r_data)
{
  /* Loop segments from start until we have an intersection. */
  auto new_closest = [&](const float3 a, const float3 b) {
    float3 closest = float3(0.0f);
    if (isect_line_plane_crossing(a, b, plane_offset, direction, closest)) {
      if (offset != 0.0f) {
        const float3 dir = math::normalize(b - a) * offset;
        r_data.append({closest + dir, curve_id});
        r_data.append({closest - dir, curve_id});
      }
      else {
        r_data.append({closest, curve_id});
      }
    }
  };

  for (const int index : IndexRange(curveinfo.positions.size()).drop_back(1)) {
    const float3 a = curveinfo.positions[index];
    const float3 b = curveinfo.positions[1 + index];
    new_closest(a, b);
  }
  if (curveinfo.cyclic) {
    const float3 a = curveinfo.positions.last();
    const float3 b = curveinfo.positions.first();
    new_closest(a, b);
  }
}

static void set_curve_intersections_plane(const bke::CurvesGeometry &src_curves,
                                          const float3 plane_offset,
                                          const float3 direction,
                                          const float offset,
                                          Vector<OutputData> &r_data)
{
  const VArray<bool> cyclic = src_curves.cyclic();
  src_curves.evaluated_points_by_curve();
  threading::parallel_for(src_curves.curves_range(), 4096, [&](IndexRange curve_range) {
    for (const int64_t curve_i : curve_range) {
      CurveInfo curveinfo;
      curveinfo.positions = src_curves.evaluated_positions_for_curve(curve_i);
      curveinfo.cyclic = cyclic[curve_i];
      if (curveinfo.positions.size() <= 1) {
        continue;
      }

      curve_plane_intersections_to_points(
          curveinfo, direction, plane_offset, offset, curve_i, r_data);
    }
  });
}

static void offset_intersections(const float3 start,
                                 const float3 end,
                                 const float offset,
                                 const float lambda,
                                 const float3 closest,
                                 const int curve_id,
                                 Vector<OutputData> &r_data)
{
  float3 dir = math::normalize(end - start) * offset;
  if (lambda == 0.0f) {
    r_data.append({closest + dir, curve_id});
  }
  else if (lambda < 1.0f) {
    r_data.append({closest + dir, curve_id});
    r_data.append({closest - dir, curve_id});
  }
  else if (lambda == 1.0f) {
    r_data.append({closest - dir, curve_id});
  }
};

static void set_curve_intersections(const bke::CurvesGeometry &src_curves,
                                    const bool self_intersect,
                                    const bool all_intersect,
                                    const float distance,
                                    const float offset,
                                    Vector<OutputData> &r_data)
{
  src_curves.evaluated_points_by_curve();

  /* Build bvh. */
  Vector<Segment> curve_segments;
  BVHTree *bvhtree = create_curve_segment_bvhtree(src_curves, &curve_segments);

  const int max_segments = curve_segments.size() + 1;
  Array<Vector<OutputData>> intersection_output_data;
  intersection_output_data.reinitialize(max_segments);
  threading::parallel_for(curve_segments.index_range(), 1024, [&](IndexRange range) {
    for (const int64_t segment_index : range) {
      Vector<OutputData> output_data;
      Vector<int> curve_ids;
      Vector<int> bvh_indices;
      bvh_indices.reserve(max_segments);
      const Segment ab = curve_segments[segment_index];
      BLI_bvhtree_range_query_cpp(*bvhtree,
                                  math::midpoint(ab.start, ab.end),
                                  math::distance(ab.start, ab.end) + distance,
                                  [&](const int index,
                                      const float3 & /*co*/,
                                      const float /*dist_sq*/) { bvh_indices.append(index); });

      for (int64_t idx : bvh_indices.index_range()) {
        const int64_t bvh_segment_index = bvh_indices[idx];
        if (segment_index <= bvh_segment_index) {
          /* Skip matching segments or previously matched segments. */
          continue;
        }
        const Segment cd = curve_segments[bvh_segment_index];
        const bool calc_self = (self_intersect && (ab.curve_index == cd.curve_index &&
                                                   (abs(ab.pos_index - cd.pos_index) > 1) &&
                                                   !(ab.is_end_segment && cd.is_end_segment)));
        const bool calc_all = (all_intersect && (ab.curve_index != cd.curve_index));
        if (calc_self || calc_all) {
          const IntersectingLineInfo lineinfo = intersecting_lines(
              ab.start, ab.end, cd.start, cd.end, distance);
          if (offset != 0.0f) {
            if (lineinfo.intersects) {
              offset_intersections(ab.start,
                                   ab.end,
                                   offset,
                                   lineinfo.lambda_ab,
                                   lineinfo.closest_ab,
                                   ab.curve_index,
                                   output_data);
              offset_intersections(cd.start,
                                   cd.end,
                                   offset,
                                   lineinfo.lambda_cd,
                                   lineinfo.closest_cd,
                                   cd.curve_index,
                                   output_data);
            }
          }
          else {
            if (lineinfo.intersects && lineinfo.lambda_ab != 1.0f && lineinfo.lambda_cd != 1.0f) {
              output_data.append({lineinfo.closest_ab, ab.curve_index});
              output_data.append({lineinfo.closest_cd, cd.curve_index});
            }
          }
        }
      }
      intersection_output_data[segment_index] = std::move(output_data);
    }
  });

  for (int64_t i : intersection_output_data.index_range()) {
    const Vector<OutputData> data = intersection_output_data[i];
    if (!data.is_empty()) {
      r_data.insert(r_data.end(), data.begin(), data.end());
    }
  }

  BLI_bvhtree_free(bvhtree);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const NodeGeometryCurveIntersections &storage = node_storage(params.node());
  const GeometryNodeCurveIntersectionMode mode = (GeometryNodeCurveIntersectionMode)storage.mode;

  GeometrySet geometry_set = params.extract_input<GeometrySet>("Curve");
  GeometryComponentEditData::remember_deformed_curve_positions_if_necessary(geometry_set);

  AnonymousAttributeIDPtr curve_index_attr_id = params.get_output_anonymous_attribute_id_if_needed(
      "Curve Index");

  lazy_threading::send_hint();

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (!geometry_set.has_curves()) {
      return;
    }
    const Curves &src_curves_id = *geometry_set.get_curves_for_read();
    const bke::CurvesGeometry &src_curves = src_curves_id.geometry.wrap();

    if (src_curves.curves_range().is_empty()) {
      return;
    }
    Vector<OutputData> r_data;
    const float offset = params.extract_input<float>("Offset");
    switch (mode) {
      case GEO_NODE_CURVE_INTERSECT_SELF: {
        const float distance = math::max(CURVE_ISECT_EPS, params.extract_input<float>("Distance"));
        set_curve_intersections(src_curves, true, false, distance, offset, r_data);
        break;
      }
      case GEO_NODE_CURVE_INTERSECT_PLANE: {
        const float3 direction = params.extract_input<float3>("Direction");
        const float3 plane_offset = params.extract_input<float3>("Plane Offset");
        set_curve_intersections_plane(src_curves, plane_offset, direction, offset, r_data);
        break;
      }
      case GEO_NODE_CURVE_INTERSECT_ALL: {
        const float distance = math::max(CURVE_ISECT_EPS, params.extract_input<float>("Distance"));
        const bool self = params.extract_input<bool>("Self Intersection");
        set_curve_intersections(src_curves, self, true, distance, offset, r_data);
        break;
      }
      default: {
        BLI_assert_unreachable();
        break;
      }
    }

    geometry_set.remove_geometry_during_modify();

    if (r_data.is_empty()) {
      return;
    }

    Vector<float3> r_position;
    Vector<int> r_curve_id;

    for (auto begin = std::make_move_iterator(r_data.begin()),
              end = std::make_move_iterator(r_data.end());
         begin != end;
         ++begin)
    {
      r_position.append(std::move(begin->first));
      r_curve_id.append(std::move(begin->second));
    }

    PointCloud *pointcloud = BKE_pointcloud_new_nomain(r_data.size());
    geometry_set.replace_pointcloud(pointcloud);

    bke::MutableAttributeAccessor point_attributes = pointcloud->attributes_for_write();
    bke::SpanAttributeWriter<float3> point_positions =
        point_attributes.lookup_or_add_for_write_only_span<float3>("position", ATTR_DOMAIN_POINT);
    point_positions.span.copy_from(r_position);
    point_positions.finish();

    if (curve_index_attr_id) {
      bke::SpanAttributeWriter<int> point_curve_id =
          point_attributes.lookup_or_add_for_write_only_span<int>(curve_index_attr_id.get(),
                                                                  ATTR_DOMAIN_POINT);
      point_curve_id.span.copy_from(r_curve_id);
      point_curve_id.finish();
    }
  });

  params.set_output("Points", std::move(geometry_set));
}
}  // namespace blender::nodes::node_geo_curve_intersection_cc

void register_node_type_geo_curve_intersections()
{
  namespace file_ns = blender::nodes::node_geo_curve_intersection_cc;

  static bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_CURVE_INTERSECTIONS, "Curve Intersections", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  node_type_storage(&ntype,
                    "NodeGeometryCurveIntersections",
                    node_free_standard_storage,
                    node_copy_standard_storage);
  nodeRegisterType(&ntype);
}
