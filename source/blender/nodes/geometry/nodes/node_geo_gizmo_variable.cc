/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_gizmo_variable_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("Value").hide_value();
  b.add_input<decl::Float>("Derivative").default_value(1.0f);
  b.add_output<decl::Float>("Value");
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_GIZMO_VARIABLE, "Gizmo Variable", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_gizmo_variable_cc
