/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_evaluate_closure_cc {

NODE_STORAGE_FUNCS(NodeGeometryEvaluateClosure)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Extend>("", "__extend__");
  b.add_input<decl::Closure>("Closure");
  b.add_input<decl::Extend>("", "__extend__");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  auto *storage = MEM_cnew<NodeGeometryEvaluateClosure>(__func__);
  node->storage = storage;
}

static void node_copy_storage(bNodeTree * /*tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryEvaluateClosure &src_storage = node_storage(*src_node);
  auto *dst_storage = MEM_cnew<NodeGeometryEvaluateClosure>(__func__, src_storage);
  dst_node->storage = dst_storage;
}

static void node_free_storage(bNode *node)
{
  MEM_freeN(node->storage);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  params.set_default_remaining_outputs();
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_EVALUATE_CLOSURE, "Evaluate Closure", NODE_CLASS_INTERFACE);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_type_storage(
      &ntype, "NodeGeometryEvaluateClosure", node_free_storage, node_copy_storage);
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_evaluate_closure_cc
