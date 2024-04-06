/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_fields.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"

#include "BLI_math_matrix.hh"

#include "NOD_socket_search_link.hh"

#include "FN_field.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_transform_instances_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Instances");
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value().field_on_all();
  b.add_input<decl::Matrix>("Transform").implicit_field_on_all(implicit_field_inputs::transform);
  b.add_input<decl::Matrix>("To Apply").field_on_all();
  b.add_output<decl::Geometry>("Instances").propagate_all();
}

static void search_link_ops(GatherLinkSearchOpParams &params)
{
  if (U.experimental.use_new_matrix_socket) {
    nodes::search_link_ops_for_basic_node(params);
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Instances");
  Field<float4x4> transform_field = params.extract_input<Field<float4x4>>("Transform");
  Field<float4x4> transformation_field = params.extract_input<Field<float4x4>>("To Apply");
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");

  static const auto apply_fn = mf::build::SI2_SO<float4x4, float4x4, float4x4>(
      "Apply Transform", [](float4x4 a, float4x4 b) { return a * b; });

  const Field<float4x4> transform_field_to_stor(FieldOperation::Create(
      apply_fn, {std::move(transform_field), std::move(transformation_field)}));

  if (bke::Instances *instances = geometry_set.get_instances_for_write()) {
    const bke::InstancesFieldContext context(*instances);
    bke::try_capture_field_on_geometry(instances->attributes_for_write(),
                                       context,
                                       "instance_transform",
                                       bke::AttrDomain::Instance,
                                       selection_field,
                                       transform_field_to_stor);
  }

  params.set_output("Instances", std::move(geometry_set));
}

static void register_node()
{
  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_TRANSFORM_INSTANCES, "Transform Instances", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(register_node)

}  // namespace blender::nodes::node_geo_transform_instances_cc
