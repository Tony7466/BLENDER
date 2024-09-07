/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BLI_string_utf8.h"

namespace blender::nodes::node_geo_foreach_geometry_element_cc {

namespace input_node {

NODE_STORAGE_FUNCS(NodeGeometryForeachGeometryElementInput);

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Geometry");
  b.add_output<decl::Geometry>("Geometry").align_with_previous();

  b.add_input<decl::Bool>("Selection").default_value(true).hide_value();
  b.add_output<decl::Int>("Index").align_with_previous();
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryForeachGeometryElementInput *data =
      MEM_cnew<NodeGeometryForeachGeometryElementInput>(__func__);
  /* Needs to be initialized for the node to work. */
  data->output_node_id = 0;
  node->storage = data;
}

static void node_label(const bNodeTree * /*ntree*/,
                       const bNode * /*node*/,
                       char *label,
                       const int label_maxncpy)
{
  BLI_strncpy_utf8(label, IFACE_("For-Each Element"), label_maxncpy);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype,
                     GEO_NODE_FOREACH_GEOMETRY_ELEMENT_INPUT,
                     "For-Each Geometry Element Input",
                     NODE_CLASS_INTERFACE);
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.labelfunc = node_label;
  ntype.gather_link_search_ops = nullptr;
  ntype.no_muting = true;
  blender::bke::node_type_storage(&ntype,
                                  "NodeGeometryForeachGeometryElementInput",
                                  node_free_standard_storage,
                                  node_copy_standard_storage);
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace input_node

namespace output_node {

NODE_STORAGE_FUNCS(NodeGeometryForeachGeometryElementOutput);

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_output<decl::Geometry>("Geometry");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryForeachGeometryElementOutput *data =
      MEM_cnew<NodeGeometryForeachGeometryElementOutput>(__func__);

  node->storage = data;
}

static void node_free_storage(bNode *node)
{
  MEM_freeN(node->storage);
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryForeachGeometryElementOutput &src_storage = node_storage(*src_node);
  auto *dst_storage = MEM_cnew<NodeGeometryForeachGeometryElementOutput>(__func__, src_storage);
  dst_node->storage = dst_storage;
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype,
                     GEO_NODE_FOREACH_GEOMETRY_ELEMENT_OUTPUT,
                     "For-Each Geometry Element Output",
                     NODE_CLASS_INTERFACE);
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.labelfunc = input_node::node_label;
  ntype.no_muting = true;
  blender::bke::node_type_storage(
      &ntype, "NodeGeometryForeachGeometryElementOutput", node_free_storage, node_copy_storage);
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace output_node

}  // namespace blender::nodes::node_geo_foreach_geometry_element_cc
