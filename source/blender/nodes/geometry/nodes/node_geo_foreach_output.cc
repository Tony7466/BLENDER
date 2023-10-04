/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_compute_contexts.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_geometry.hh"
#include "NOD_socket.hh"
#include "NOD_zone_socket_items.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_foreach_output_cc {

NODE_STORAGE_FUNCS(NodeGeometryForEachOutput);

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_output<decl::Geometry>("Geometry");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryForEachOutput *data = MEM_cnew<NodeGeometryForEachOutput>(__func__);

  node->storage = data;
}

static void node_free_storage(bNode *node)
{
  socket_items::destruct_array<ForEachInputItemsAccessor>(*node);
  socket_items::destruct_array<ForEachOutputItemsAccessor>(*node);
  MEM_freeN(node->storage);
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryForEachOutput &src_storage = node_storage(*src_node);
  auto *dst_storage = MEM_new<NodeGeometryForEachOutput>(__func__, src_storage);
  dst_node->storage = dst_storage;

  socket_items::copy_array<ForEachInputItemsAccessor>(*src_node, *dst_node);
  socket_items::copy_array<ForEachOutputItemsAccessor>(*src_node, *dst_node);
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_FOR_EACH_OUTPUT, "For-Each Output", NODE_CLASS_INTERFACE);
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = nullptr;
  node_type_storage(&ntype, "NodeGeometryForEachOutput", node_free_storage, node_copy_storage);
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_foreach_output_cc
