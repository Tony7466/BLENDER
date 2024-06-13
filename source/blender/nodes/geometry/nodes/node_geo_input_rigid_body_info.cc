/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_node.hh"
#include "BKE_physics_geometry.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_rigid_body_info_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Int>("ID").field_source();
  b.add_output<decl::Float>("Mass").field_source();
  b.add_output<decl::Vector>("Inertia").field_source();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  params.set_output("ID",
                    AttributeFieldInput::Create<int>(bke::PhysicsGeometry::builtin_attributes.id));
  params.set_output(
      "Mass", AttributeFieldInput::Create<float>(bke::PhysicsGeometry::builtin_attributes.mass));
  params.set_output(
      "Inertia",
      AttributeFieldInput::Create<float3>(bke::PhysicsGeometry::builtin_attributes.inertia));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INPUT_RIGID_BODY_INFO, "Rigid Body Info", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_rigid_body_info_cc
