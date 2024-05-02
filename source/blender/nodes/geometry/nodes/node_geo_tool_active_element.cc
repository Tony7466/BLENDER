/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DEG_depsgraph_query.hh"

#include "node_geometry_util.hh"

#include "DNA_meshdata_types.h"

#include "BKE_mesh.hh"

namespace blender::nodes::node_geo_tool_active_element_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Int>("Vertex Index");
  b.add_output<decl::Int>("Edge Index");
  b.add_output<decl::Int>("Face Index");
}

static void node_exec(GeoNodeExecParams params)
{
  if (!check_tool_context_and_error(params)) {
    return;
  }

  if (params.user_data()->call_data->operator_data->mode == OB_MODE_EDIT) {
    // All the BKE and BM functions expect non-const pointers, so we have to cast away the constness.
    Object* self = const_cast<Object*>(params.user_data()->call_data->self_object());

    if (self->type != OB_MESH) {
      params.set_default_remaining_outputs();
      return;
    }

    Mesh* mesh = BKE_mesh_from_object(self);
    params.set_output<int>("Vertex Index", BKE_mesh_mselect_active_get(mesh, ME_VSEL));
    params.set_output<int>("Edge Index", BKE_mesh_mselect_active_get(mesh, ME_ESEL));
    params.set_output<int>("Face Index", BKE_mesh_mselect_active_get(mesh, ME_FSEL));
  }

  params.set_default_remaining_outputs();
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
