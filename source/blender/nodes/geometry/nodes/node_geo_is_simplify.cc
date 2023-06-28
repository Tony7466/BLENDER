/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DEG_depsgraph_query.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_is_simplify_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Bool>("Is Simplify");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const Depsgraph *depsgraph = params.depsgraph();
  const Scene *scene = DEG_get_input_scene(depsgraph);
  const bool is_simplify = scene->r.mode & R_SIMPLIFY;

  params.set_output("Is Simplify", is_simplify);
}

}  // namespace blender::nodes::node_geo_is_simplify_cc

void register_node_type_geo_is_simplify()
{
  namespace file_ns = blender::nodes::node_geo_is_simplify_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_IS_SIMPLIFY, "Is Simplify", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
