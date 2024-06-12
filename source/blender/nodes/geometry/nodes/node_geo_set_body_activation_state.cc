/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_node.hh"
#include "BKE_physics_geometry.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_set_body_activation_state_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Physics");
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value().field_on_all();
  b.add_input<decl::Int>("Activation State").min(0).max(4).field_on_all();
  b.add_output<decl::Geometry>("Physics").propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Physics");
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  Field<int> activation_state_field = params.extract_input<Field<int>>("Activation State");

  if (bke::PhysicsGeometry *physics = geometry_set.get_physics_for_write()) {
    bke::PhysicsFieldContext context(*physics, AttrDomain::Point);
    bke::try_capture_field_on_geometry(physics->attributes_for_write(),
                                       context,
                                       bke::PhysicsGeometry::builtin_attributes.activation_state,
                                       AttrDomain::Point,
                                       selection_field,
                                       activation_state_field);

    physics->tag_physics_changed();
  }

  params.set_output("Physics", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype,
                     GEO_NODE_SET_BODY_ACTIVATION_STATE,
                     "Set Body Activation State",
                     NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_set_body_activation_state_cc
