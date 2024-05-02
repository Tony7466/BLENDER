/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "DNA_meshdata_types.h"

#include "BKE_mesh.hh"

namespace blender::nodes::node_geo_tool_active_element_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Int>("Point Index")
      .default_value(-1)
      .description("Index of the active point in the mesh, or -1 if no point is active");
  b.add_output<decl::Int>("Edge Index")
      .default_value(-1)
      .description("Index of the active edge in the mesh, or -1 if no edge is active");
  b.add_output<decl::Int>("Face Index")
      .default_value(-1)
      .description("Index of the active face in the mesh, or -1 if no face is active");
}

static void node_exec(GeoNodeExecParams params)
{
  if (!check_tool_context_and_error(params)) {
    return;
  }

  if (params.user_data()->call_data->operator_data->mode != OB_MODE_EDIT) {
    params.set_default_remaining_outputs();
    return;
  }

  const Object *self = params.user_data()->call_data->self_object();

  if (self->type != OB_MESH) {
    params.set_default_remaining_outputs();
    return;
  }

  const GeoNodesOperatorData *operator_data = params.user_data()->call_data->operator_data;

  params.set_output("Point Index", operator_data->active_point_index);
  params.set_output("Edge Index", operator_data->active_edge_index);
  params.set_output("Face Index", operator_data->active_face_index);
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_TOOL_ACTIVE_ELEMENT, "Active Element", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = node_exec;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops_for_tool_node;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_tool_active_element_cc
