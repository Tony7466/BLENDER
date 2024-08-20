/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_fields.hh"
#include "BKE_node.hh"
#include "BKE_physics_geometry.hh"

#include "RNA_enum_types.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_set_body_static_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Physics").supported_type(bke::GeometryComponent::Type::Physics);
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value().field_on_all();
  b.add_input<decl::Bool>("Static").default_value(true).field_on_all();
  b.add_output<decl::Geometry>("Physics").propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Physics");
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  Field<bool> static_field = params.extract_input<Field<bool>>("Static");

  // XXX bodies should not change flags while in the world (gets copied to collision state on
  // construction time). Remove and add back bodies where the flag changes!

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (bke::PhysicsGeometry *physics = geometry_set.get_physics_for_write()) {
      bke::PhysicsFieldContext context(*physics, AttrDomain::Point);
      bke::try_capture_field_on_geometry(physics->attributes_for_write(),
                                         context,
                                         bke::PhysicsGeometry::body_attribute_name(
                                             bke::PhysicsGeometry::BodyAttribute::is_static),
                                         AttrDomain::Point,
                                         selection_field,
                                         static_field);
    }
  });

  params.set_output("Physics", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_SET_BODY_STATIC, "Set Body Static", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;

  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_set_body_static_cc
