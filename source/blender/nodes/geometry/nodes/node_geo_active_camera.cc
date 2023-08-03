/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"
#include "DEG_depsgraph_query.h"

namespace blender::nodes::node_geo_active_camera_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Object>(N_("Active Camera"));
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const Scene *scene = DEG_get_input_scene(params.depsgraph());
  params.set_output("Active Camera", const_cast<Object *>(scene->camera));
}

} // namespace blender::nodes::node_geo_active_camera_cc

void register_node_type_geo_active_camera()
{
  namespace file_ns = blender::nodes::node_geo_active_camera_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_ACTIVE_CAMERA, "Active Camera", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
