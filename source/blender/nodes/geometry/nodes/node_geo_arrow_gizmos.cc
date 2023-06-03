/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_gizmos.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_arrow_gizmo_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Geometry>("Gizmo");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  bke::GizmosGeometry *gizmo = new bke::GizmosGeometry("path!", const_cast<bNode *>(&params.node()));

  params.set_output("Gizmo", GeometrySet::create_with_gizmos(gizmo));
}

}  // namespace blender::nodes::node_geo_arrow_gizmo_cc

void register_node_type_geo_arrow_gizmo()
{
  namespace file_ns = blender::nodes::node_geo_arrow_gizmo_cc;
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_ARROW_GIZMO, "Arrow Gizmo", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
