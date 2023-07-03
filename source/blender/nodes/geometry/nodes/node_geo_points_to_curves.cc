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
      .supported_type({GeometryComponent::Type::Curve,
                       GeometryComponent::Type::Mesh,
                       GeometryComponent::Type::PointCloud})
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

static Curves *curves_from_all_points(const Span<const GeometryComponent *> components,
                                      const Field<int> &curves_group_id_field,
                                      const Field<float> &length_in_curve_field,
                                      const Map<AttributeIDRef, AttributeKind> &attributes)
{
  if (components.is_empty()) {
    return nullptr;
  }

  const int total_components = components.size();
  Array<int> accumulated_domain_sizes(total_components + 1);
  for (const int component_index : components.index_range()) {
    const GeometryComponent *component = components[component_index];
    const int domain_size = component->attribute_domain_size(ATTR_DOMAIN_POINT);
    accumulated_domain_sizes[component_index] = domain_size;
  }
  offset_indices::accumulate_counts_to_offsets(accumulated_domain_sizes);
  const OffsetIndices<int> components_offsets(accumulated_domain_sizes);
  const int domain_size = accumulated_domain_sizes.last();

  Array<int> all_group_ids(domain_size);
  Array<float> all_lengths(domain_size);

  for (const int component_index : components.index_range()) {
    const IndexRange component_range = components_offsets[component_index];
    const GeometryComponent *component = components[component_index];
    const bke::GeometryFieldContext context(*component, ATTR_DOMAIN_POINT);
    fn::FieldEvaluator evaluator(context, component_range.size());
    evaluator.add_with_destination(curves_group_id_field,
                                   all_group_ids.as_mutable_span().slice(component_range));
    evaluator.add_with_destination(length_in_curve_field,
                                   all_lengths.as_mutable_span().slice(component_range));
    evaluator.evaluate();
  }

  VectorSet<int> curve_by_id(all_group_ids.as_span()); /* Have to be sotred to stable. */
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
    const int curve_id = all_group_ids[index];
    const int curve_index = curve_by_id.index_of(curve_id);
    src_to_dst_indices[index] = accumulated_curves[curve_index];
    accumulated_curves[curve_index]++;
  }
  offset_indices::accumulate_counts_to_offsets(accumulated_curves);
  const OffsetIndices<int> curves_offsets(accumulated_curves);

  /* Global to global. */
  Array<int> dst_to_src_indices(domain_size);
  for (const int index : IndexRange(domain_size)) {
    const int curve_id = all_group_ids[index];
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
                    const float length_a = all_lengths[index_a];
                    const float length_b = all_lengths[index_b];
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

  for (const int component_index : components.index_range()) {
    const GeometryComponent *component = components[component_index];
    const IndexRange component_range = components_offsets[component_index];
    const Span<int> indices = src_to_dst_indices.as_span().slice(component_range);

    const AttributeAccessor src_attributes = *component->attributes();
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
        inverse_fill(indices, dst.span);
      }
      else {
        inverse_gather(src.varray, indices, dst.span);
      }

      dst.finish();
    }
  }

  return curves_id;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Points");
  Field<int> curves_group_id_field = params.extract_input<Field<int>>("Curve Group ID");
  Field<float> length_in_curve_field = params.extract_input<Field<float>>("Weight");

  static const Array<GeometryComponent::Type, 3> supported_types = {
      GeometryComponent::Type::Curve,
      GeometryComponent::Type::Mesh,
      GeometryComponent::Type::PointCloud};

  Map<AttributeIDRef, AttributeKind> attributes;
  geometry_set.gather_attributes_for_propagation(supported_types,
                                                 GeometryComponent::Type::Curve,
                                                 false,
                                                 params.get_output_propagation_info("Curves"),
                                                 attributes);

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    Vector<const GeometryComponent *, 3> point_components;
    for (const GeometryComponent::Type type : supported_types) {
      if (!geometry_set.has(type)) {
        continue;
      }
      const GeometryComponent *component = geometry_set.get_component_for_read(type);
      if (component->attribute_domain_size(ATTR_DOMAIN_POINT) != 0) {
        point_components.append(component);
      }
    }

    Curves *curves_id = curves_from_all_points(
        point_components, curves_group_id_field, length_in_curve_field, attributes);
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
