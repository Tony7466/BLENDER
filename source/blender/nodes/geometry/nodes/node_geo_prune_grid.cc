/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_prune_grid_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  //  b.add_input<decl::Grid>("Grid");
  b.add_input<decl::Float>("Tolerance").default_value(0.01f).min(0.0f);

  //  b.add_output<decl::Grid>("Grid");
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_OPENVDB
  params.set_default_remaining_outputs();
#else
  params.set_default_remaining_outputs();
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without OpenVDB"));
#endif
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_PRUNE_GRID, "Prune Grid", NODE_CLASS_CONVERTER);

  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_prune_grid_cc
