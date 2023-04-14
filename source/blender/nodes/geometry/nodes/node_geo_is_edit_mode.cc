/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_is_edit_mode_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Bool>(N_("Is Edit Mode"));
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const Object *object = params.self_object();
  const bool is_edit_mode = object->mode == OB_MODE_EDIT;

  params.set_output("Is Edit Mode", is_edit_mode);
}

} // namespace blender::nodes::node_geo_is_edit_mode_cc

void register_node_type_geo_is_edit_mode()
{
  namespace file_ns = blender::nodes::node_geo_is_edit_mode_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_IS_EDIT_MODE, "Is Edit Mode", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}