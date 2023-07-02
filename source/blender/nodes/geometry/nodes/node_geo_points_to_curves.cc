/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

//#include "BLI_array.hh"
//#include "BLI_vector_set.hh"
//#include "BLI_vector.hh"
//#include "BLI_index_range.hh"
//#include "BLI_offsets_indices.hh"
#include "BLI_array_utils.hh"
#include "BLI_sort.hh"

//#include "BKE_attribute_math.hh"
//#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
//#include "BKE_geometry_fields.hh"

#include "DNA_pointcloud_types.h"

namespace blender::nodes::node_geo_points_to_curves_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Points");
  b.add_input<decl::Int>("Curve Group ID").field_on_all().hide_value();
  b.add_input<decl::Float>("Length in Curve").field_on_all().hide_value();

  b.add_output<decl::Geometry>("Curves").propagate_all();
}

void gather(const GSpan &src, const Span<int> &indices, GMutableSpan dst)
{
  bke::attribute_math::convert_to_static_type(src.type(), [&](auto dummy) {
    using T = decltype(dummy);
    array_utils::gather<T, int>(src.typed<T>(), indices, dst.typed<T>());
  });
}

static Curves *curves_from_all_points(const Span<const GeometryComponent *> components,
                                      const Field<int> &curves_group_id_field,
                                      const Field<float> &length_in_curve_field,
                                      const Map<AttributeIDRef, AttributeKind> &attributes)
{
  const GeometryComponent *component = components.first();

  const int domain_size = component->attribute_domain_size(ATTR_DOMAIN_POINT);
  const bke::GeometryFieldContext context(*component, ATTR_DOMAIN_POINT);
  fn::FieldEvaluator evaluator{context, domain_size};
  evaluator.add(curves_group_id_field);
  evaluator.add(length_in_curve_field);
  evaluator.evaluate();
  const VArray<int> curves_group_id = evaluator.get_evaluated<int>(0);
  const VArray<float> length_in_curve = evaluator.get_evaluated<float>(1);

  VectorSet<int> curve_index_by_id;
  for (const int index : IndexRange(domain_size)) {
    const int curve_id = curves_group_id[index];
    curve_index_by_id.add(curve_id);
  }

  Curves *curves_id = bke::curves_new_nomain(domain_size, curve_index_by_id.size());
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  curves.fill_curve_types(CURVE_TYPE_POLY);

  MutableSpan<int> curves_offsets = curves.offsets_for_write();
  curves_offsets.fill(0);

  Array<int> indices_in_curve(domain_size);
  for (const int index : IndexRange(domain_size)) {
    const int curve_id = curves_group_id[index];
    const int curve_index = curve_index_by_id.index_of(curve_id);
    indices_in_curve[index] = curves_offsets[curve_index];
    curves_offsets[curve_index]++;
  }
  offset_indices::accumulate_counts_to_offsets(curves_offsets);
  const OffsetIndices<int> offsets(curves_offsets);

  for (const int index : curves.curves_range()) {
    curves.cyclic_for_write()[index] = false;
  }

  Array<int> curves_points_mapping(domain_size);
  for (const int index : IndexRange(domain_size)) {
    const int curve_id = curves_group_id[index];
    const int curve_index = curve_index_by_id.index_of(curve_id);
    const IndexRange curve_points = offsets[curve_index];
    const int index_in_curve = indices_in_curve[index];
    const int dst_index = curve_points[index_in_curve];
    curves_points_mapping[dst_index] = index;
  }

  for (const int index : IndexRange(domain_size)) {
    const int curve_id = curves_group_id[index];
    const int curve_index = curve_index_by_id.index_of(curve_id);
    const IndexRange curve_points = offsets[curve_index];
    MutableSpan<int> curve_mapping = curves_points_mapping.as_mutable_span().slice(curve_points);
    parallel_sort(
        curve_mapping.begin(), curve_mapping.end(), [&](const int index_a, const int index_b) {
          const float length_a = length_in_curve[index_a];
          const float length_b = length_in_curve[index_b];
          if (UNLIKELY(length_a == length_b)) {
            /* Approach to make it stable. */
            return index_a > index_b;
          }
          return length_a > length_b;
        });
  }

  const AttributeAccessor src_attributes = *component->attributes();
  MutableAttributeAccessor dst_attributes = curves.attributes_for_write();

  for (const auto [attribute_id_ref, attribute_kind] : attributes.items()) {
    //
  }

  const AttributeReader<float3> points_positions_attribute = src_attributes.lookup<float3>(
      "position", ATTR_DOMAIN_POINT);
  const VArraySpan<float3> points_positions = *points_positions_attribute;
  SpanAttributeWriter<float3> curve_positions_attribute =
      dst_attributes.lookup_or_add_for_write_span<float3>("position", ATTR_DOMAIN_POINT);
  MutableVArraySpan<float3> &curve_positions = curve_positions_attribute.span;

  gather(points_positions, curves_points_mapping, curve_positions);

  curve_positions_attribute.finish();

  return curves_id;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Points");
  Field<int> curves_group_id_field = params.extract_input<Field<int>>("Curve Group ID");
  Field<float> length_in_curve_field = params.extract_input<Field<float>>("Length in Curve");

  static const Array<GeometryComponent::Type, 3> supported_types = {
      GeometryComponent::Type::Mesh,
      GeometryComponent::Type::PointCloud,
      GeometryComponent::Type::Curve};

  Map<AttributeIDRef, AttributeKind> attributes;
  geometry_set.gather_attributes_for_propagation(supported_types,
                                                 GeometryComponent::Type::Mesh,
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

    if (!point_components.is_empty()) {
      Curves *curves_id = curves_from_all_points(
          point_components, curves_group_id_field, length_in_curve_field, attributes);
      geometry_set.replace_curves(curves_id);
    }

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
