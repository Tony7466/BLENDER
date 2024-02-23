/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_view3d_types.h"

#include "BLI_math_matrix.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_viewport_transform_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Matrix>("Viewport Transform")
      .description("The view direction and location of the 3D viewport");
  b.add_output<decl::Bool>("Orthographic")
      .description("True if the viewport is in orthographic mode");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  if (!check_tool_context_and_error(params)) {
    return;
  }
  const Object &self_object = *params.self_object();
  const RegionView3D &rv3d = *params.user_data()->call_data->operator_data->rv3d;
  params.set_output("Viewport Transform", self_object.world_to_object() * float4x4(rv3d.viewmat));
  params.set_output("Orthographic", bool(rv3d.is_persp));
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_TOOL_VIEWPORT_TRANSFORM, "Viewport Transform", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.gather_link_search_ops = search_link_ops_for_tool_node;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_viewport_transform_cc
