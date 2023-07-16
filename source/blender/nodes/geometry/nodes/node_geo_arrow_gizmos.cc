/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <fmt/format.h>

#include "BKE_gizmos.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_arrow_gizmo_cc {

NODE_STORAGE_FUNCS(NodeGeometryArrowGizmo)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Geometry>("Gizmo");
  b.add_output<decl::Float>("Value");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "value", 0, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryArrowGizmo *data = MEM_cnew<NodeGeometryArrowGizmo>(__func__);
  data->value = 0.0f;
  node->storage = data;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const bNode &node = params.node();
  const bNodeTree &tree = node.owner_tree();
  std::string gizmos_value_path_str = fmt::format(
      TIP_("node_groups[\"{}\"].nodes[\"{}\"]"), tree.id.name + 2, node.name);
  bke::GizmosGeometry *gizmo = new bke::GizmosGeometry(std::move(gizmos_value_path_str));
  params.set_output("Gizmo", GeometrySet::create_with_gizmos(gizmo));

  const NodeGeometryArrowGizmo &storage = node_storage(node);
  params.set_output("Value", storage.value);
  std::cout << ">> " << __func__ << ";\n";
}

}  // namespace blender::nodes::node_geo_arrow_gizmo_cc

void register_node_type_geo_arrow_gizmo()
{
  namespace file_ns = blender::nodes::node_geo_arrow_gizmo_cc;
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_ARROW_GIZMO, "Arrow Gizmo", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.initfunc = file_ns::node_init;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.declare = file_ns::node_declare;
  node_type_storage(
      &ntype, "NodeGeometryArrowGizmo", node_free_standard_storage, node_copy_standard_storage);
  nodeRegisterType(&ntype);
}
