/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_node.hh"
#include "BKE_physics_geometry.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_body_motion_state_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Bool>("Static").field_source();
  b.add_output<decl::Bool>("Kinematic").field_source();
  b.add_output<decl::Vector>("Position").field_source();
  b.add_output<decl::Rotation>("Rotation").field_source();
  b.add_output<decl::Vector>("Velocity").field_source();
  b.add_output<decl::Vector>("Angular Velocity").field_source();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  params.set_output("Static",
                    AttributeFieldInput::Create<bool>(bke::PhysicsGeometry::body_attribute_name(
                        bke::PhysicsGeometry::BodyAttribute::is_static)));
  params.set_output("Kinematic",
                    AttributeFieldInput::Create<bool>(bke::PhysicsGeometry::body_attribute_name(
                        bke::PhysicsGeometry::BodyAttribute::is_kinematic)));

  params.set_output("Position",
                    AttributeFieldInput::Create<float3>(bke::PhysicsGeometry::body_attribute_name(
                        bke::PhysicsGeometry::BodyAttribute::position)));
  params.set_output(
      "Rotation",
      AttributeFieldInput::Create<math::Quaternion>(bke::PhysicsGeometry::body_attribute_name(
          bke::PhysicsGeometry::BodyAttribute::rotation)));
  params.set_output("Velocity",
                    AttributeFieldInput::Create<float3>(bke::PhysicsGeometry::body_attribute_name(
                        bke::PhysicsGeometry::BodyAttribute::velocity)));
  params.set_output("Angular Velocity",
                    AttributeFieldInput::Create<float3>(bke::PhysicsGeometry::body_attribute_name(
                        bke::PhysicsGeometry::BodyAttribute::angular_velocity)));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_INPUT_BODY_MOTION_STATE, "Body Motion State", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_body_motion_state_cc
