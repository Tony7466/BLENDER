/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BLI_sort.hh"

#include "BKE_geometry_set.hh"

namespace blender::nodes::node_geo_points_to_curves_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Points")
      .supported_type(GeometryComponent::Type::PointCloud)
      .description("Points to build curves");
  b.add_input<decl::Int>("Curve Group ID")
      .field_on_all()
      .hide_value()
      .description(
          "Index of curve for each point. ID groups will be converted to curves indices without "
          "gaps");
  b.add_input<decl::Float>("Weight").field_on_all().hide_value().description(
      "Weight to sort points of curve");

  b.add_output<decl::Geometry>("Curves").propagate_all();
}

void inverse_gather(const GVArray &src, const Span<int> &indices, GMutableSpan dst)
{
  bke::attribute_math::convert_to_static_type(dst.type(), [&](auto dummy) {
    using T = decltype(dummy);
    const VArray<T> src_typed = src.typed<T>();
    MutableSpan<T> dst_typed = dst.typed<T>();
    devirtualize_varray(src_typed, [&](const auto src_typed) {
      threading::parallel_for(indices.index_range(), 4096, [&](const IndexRange range) {
        for (const int64_t index : range) {
          dst_typed[indices[index]] = src_typed[index];
        }
      });
    });
  });
}

void inverse_fill(const Span<int> &indices, GMutableSpan dst)
{
  bke::attribute_math::convert_to_static_type(dst.type(), [&](auto dummy) {
    using T = decltype(dummy);
    MutableSpan<T> dst_typed = dst.typed<T>();
    threading::parallel_for(indices.index_range(), 4096, [&](const IndexRange range) {
      for (const int64_t index : range) {
        dst_typed[indices[index]] = T();
      }
    });
  });
}

static Curves *curves_from_points(const PointCloud *points,
                                  const Field<int> &group_id_field,
                                  const Field<float> &length_field,
                                  const Map<AttributeIDRef, AttributeKind> &attributes)
{
  if (points == nullptr) {
    return nullptr;
  }

  const int domain_size = points->totpoint;

  Array<int> group_ids(domain_size);
  Array<float> lengths(domain_size);

  const bke::PointCloudFieldContext context(*points);
  fn::FieldEvaluator evaluator(context, domain_size);
  evaluator.add_with_destination(group_id_field, group_ids.as_mutable_span());
  evaluator.add_with_destination(length_field, lengths.as_mutable_span());
  evaluator.evaluate();

  VectorSet<int> curve_by_id(group_ids.as_span());
  const int total_curves = curve_by_id.size();

  Curves *curves_id = bke::curves_new_nomain(domain_size, total_curves);
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  curves.fill_curve_types(CURVE_TYPE_POLY);
  curves.cyclic_for_write().fill(false);
  MutableSpan<int> accumulated_curves = curves.offsets_for_write();

  accumulated_curves.fill(0);
  /* Indices from global src to dst in each curve. Not to global dst yet. */
  Array<int> src_to_dst_indices(domain_size);
  for (const int index : IndexRange(domain_size)) {
    const int curve_id = group_ids[index];
    const int curve_index = curve_by_id.index_of(curve_id);
    src_to_dst_indices[index] = accumulated_curves[curve_index];
    accumulated_curves[curve_index]++;
  }
  const OffsetIndices<int> curves_offsets = offset_indices::accumulate_counts_to_offsets(
      accumulated_curves);

  /* Global to global. */
  Array<int> dst_to_src_indices(domain_size);
  for (const int index : IndexRange(domain_size)) {
    const int curve_id = group_ids[index];
    const int curve_index = curve_by_id.index_of(curve_id);
    const IndexRange curve_points = curves_offsets[curve_index];
    const int index_in_curve = src_to_dst_indices[index];
    /* Now dst index can be global. But not required to be rewrited. */
    const int dst_index = curve_points[index_in_curve];
    dst_to_src_indices[dst_index] = index;
  }

  for (const int curve_index : IndexRange(total_curves)) {
    const IndexRange curve_points = curves_offsets[curve_index];
    MutableSpan<int> src_indices_of_curve = dst_to_src_indices.as_mutable_span().slice(
        curve_points);
    parallel_sort(src_indices_of_curve.begin(),
                  src_indices_of_curve.end(),
                  [&](const int index_a, const int index_b) {
                    const float length_a = lengths[index_a];
                    const float length_b = lengths[index_b];
                    if (UNLIKELY(length_a == length_b)) {
                      /* Approach to make it stable. */
                      return index_a > index_b;
                    }
                    return length_a > length_b;
                  });
  }

  /* Now indices is from global src to global dst. */
  for (const int index : IndexRange(domain_size)) {
    const int src_index = dst_to_src_indices[index];
    src_to_dst_indices[src_index] = index;
  }

  const AttributeAccessor src_attributes = points->attributes();
  MutableAttributeAccessor dst_attributes = curves.attributes_for_write();

  for (MapItem<AttributeIDRef, AttributeKind> entry : attributes.items()) {
    const AttributeIDRef attribute_id = entry.key;
    const eCustomDataType output_data_type = entry.value.data_type;

    GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        attribute_id, ATTR_DOMAIN_POINT, output_data_type);
    if (!dst) {
      continue;
    }

    const GAttributeReader src = src_attributes.lookup(attribute_id);
    if (!src || src.domain != ATTR_DOMAIN_POINT) {
      inverse_fill(src_to_dst_indices, dst.span);
    }
    else {
      inverse_gather(src.varray, src_to_dst_indices, dst.span);
    }

    dst.finish();
  }

  return curves_id;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Points");
  const Field<int> group_id_field = params.extract_input<Field<int>>("Curve Group ID");
  const Field<float> length_field = params.extract_input<Field<float>>("Weight");

  Map<AttributeIDRef, AttributeKind> attributes;
  geometry_set.gather_attributes_for_propagation({GeometryComponent::Type::PointCloud},
                                                 GeometryComponent::Type::Curve,
                                                 false,
                                                 params.get_output_propagation_info("Curves"),
                                                 attributes);

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    const PointCloud *points = geometry_set.get_pointcloud_for_read();
    Curves *curves_id = curves_from_points(points, group_id_field, length_field, attributes);
    geometry_set.replace_curves(curves_id);
    geometry_set.keep_only_during_modify({GeometryComponent::Type::Curve});
  });

  params.set_output("Curves", std::move(geometry_set));
}

}  // namespace blender::nodes::node_geo_points_to_curves_cc

void register_node_type_geo_points_to_curves()
{
  namespace file_ns = blender::nodes::node_geo_points_to_curves_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_POINTS_TO_CURVES, "Points to Curves", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
