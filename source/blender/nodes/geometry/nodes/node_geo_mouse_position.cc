/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_mouse_position_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Int>("Mouse X");
  b.add_output<decl::Int>("Mouse Y");
  b.add_output<decl::Int>("Size X");
  b.add_output<decl::Int>("Size Y");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  if (!check_tool_context_and_error(params)) {
    return;
  }
  const int2 mouse = params.user_data()->call_data->operator_data->mouse_position;
  const int2 size = params.user_data()->call_data->operator_data->region_size;
  params.set_output("Mouse X", mouse.x);
  params.set_output("Mouse Y", mouse.y);
  params.set_output("Size X", size.x);
  params.set_output("Size Y", size.y);
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_TOOL_MOUSE_POSITION, "Mouse Position", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.gather_link_search_ops = search_link_ops_for_tool_node;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_mouse_position_cc
