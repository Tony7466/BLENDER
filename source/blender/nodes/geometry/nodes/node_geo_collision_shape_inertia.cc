/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

namespace blender::nodes::node_geo_collision_shape_inertia_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Shape");
  b.add_output<decl::Vector>("Inertia");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeometrySet geometry_set = params.extract_input<GeometrySet>("Shape");
  const bke::CollisionShape *shape = geometry_set.get_collision_shape();
  if (shape == nullptr) {
    params.set_default_remaining_outputs();
    return;
  }

  const float mass = 1.0f;
  params.set_output("Inertia", shape->calculate_local_inertia(mass));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_COLLISION_SHAPE_INERTIA, "Collision Shape Inertia", NODE_CLASS_CONVERTER);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_collision_shape_inertia_cc
