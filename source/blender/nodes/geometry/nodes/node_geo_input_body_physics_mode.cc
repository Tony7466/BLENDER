/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_node.hh"
#include "BKE_physics_geometry.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_body_physics_mode_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Bool>("Static").field_source();
  b.add_output<decl::Bool>("Kinematic").field_source();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<bool> static_field = AttributeFieldInput::Create<bool>(
      bke::PhysicsGeometry::builtin_attributes.is_static);
  Field<bool> kinematic_field = AttributeFieldInput::Create<bool>(
      bke::PhysicsGeometry::builtin_attributes.is_kinematic);
  params.set_output("Static", std::move(static_field));
  params.set_output("Kinematic", std::move(kinematic_field));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INPUT_BODY_PHYSICS_MODE, "Physics Mode", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_body_physics_mode_cc
