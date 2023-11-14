/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_gizmo_arrow_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("Value").hide_value().multi_input();
  b.add_input<decl::Vector>("Position");
  b.add_input<decl::Vector>("Direction").default_value({0, 0, 1});
  b.add_output<decl::Geometry>("Transform");
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_GIZMO_ARROW, "Arrow Gizmo", NODE_CLASS_INTERFACE);
  ntype.declare = node_declare;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_gizmo_arrow_cc
