/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"

#include "DNA_pointcloud_types.h"

#include "BKE_attribute_math.hh"
#include "BKE_instances.hh"
#include "BKE_pointcloud.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_instances_to_points_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Instances").only_instances();
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value().field_on_all();
  b.add_input<decl::Vector>("Position").implicit_field_on_all(implicit_field_inputs::position);
  b.add_input<decl::Float>("Radius")
      .default_value(0.05f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .field_on_all();
  b.add_output<decl::Geometry>("Points").propagate_all();
}

static void convert_instances_to_points(GeometrySet &geometry_set,
                                        Field<float3> position_field,
                                        Field<float> radius_field,
                                        Field<bool> selection_field,
                                        const AttributeFilter &attribute_filter)
{
  const bke::Instances &instances = *geometry_set.get_instances();

  const bke::InstancesFieldContext context{instances};
  fn::FieldEvaluator evaluator{context, instances.instances_num()};
  evaluator.set_selection(std::move(selection_field));
  evaluator.add(std::move(position_field));
  evaluator.add(std::move(radius_field));
  evaluator.evaluate();
  const IndexMask selection = evaluator.get_evaluated_selection_as_mask();
  if (selection.is_empty()) {
    return;
  }
  const GVArray positions = evaluator.get_evaluated(0);
  const GVArray radii = evaluator.get_evaluated(1);

  PointCloud *pointcloud = bke::pointcloud_new_no_attributes(selection.size());
  geometry_set.replace_pointcloud(pointcloud);
  MutableAttributeAccessor dst_attributes = pointcloud->attributes_for_write();

  /* TODO: Compose filter to include skip of positions ans radius attribute from gathering. */
  bke::gather_attributes(instances.attributes(),
                         bke::AttrDomain::Instance,
                         bke::AttrDomain::Point,
                         attribute_filter,
                         selection,
                         dst_attributes);

  bke::GSpanAttributeWriter point_positions = dst_attributes.lookup_or_add_for_write_only_span(
      "position", AttrDomain::Point, CD_PROP_FLOAT3);
  array_utils::gather(positions, selection, point_positions.span);
  point_positions.finish();

  bke::GSpanAttributeWriter point_radii = dst_attributes.lookup_or_add_for_write_only_span(
      "radius", AttrDomain::Point, CD_PROP_FLOAT);
  array_utils::gather(radii, selection, point_radii.span);
  point_radii.finish();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Instances");

  if (geometry_set.has_instances()) {
    convert_instances_to_points(geometry_set,
                                params.extract_input<Field<float3>>("Position"),
                                params.extract_input<Field<float>>("Radius"),
                                params.extract_input<Field<bool>>("Selection"),
                                params.get_attribute_filter("Points"));
    geometry_set.keep_only({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Edit});
    params.set_output("Points", std::move(geometry_set));
  }
  else {
    params.set_default_remaining_outputs();
  }
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_INSTANCES_TO_POINTS, "Instances to Points", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_instances_to_points_cc
