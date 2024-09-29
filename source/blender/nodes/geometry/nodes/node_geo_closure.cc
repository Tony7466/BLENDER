/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BLI_string_utf8.h"

namespace blender::nodes::node_geo_closure_cc {

namespace input_node {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Extend>("", "__extend__");
}

static void node_label(const bNodeTree * /*ntree*/,
                       const bNode * /*node*/,
                       char *label,
                       const int label_maxncpy)
{
  BLI_strncpy_utf8(label, IFACE_("Closure"), label_maxncpy);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryClosureInput *data = MEM_cnew<NodeGeometryClosureInput>(__func__);
  /* Needs to be initialized for the node to work. */
  data->output_node_id = 0;
  node->storage = data;
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_CLOSURE_INPUT, "Closure Input", NODE_CLASS_INTERFACE);
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = nullptr;
  ntype.initfunc = node_init;
  ntype.labelfunc = node_label;
  ntype.no_muting = true;
  blender::bke::node_type_storage(
      &ntype, "NodeGeometryClosureInput", node_free_standard_storage, node_copy_standard_storage);
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace input_node

namespace output_node {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Closure>("Closure");
  b.add_input<decl::Extend>("", "__extend__");
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_CLOSURE_OUTPUT, "Closure Output", NODE_CLASS_INTERFACE);
  ntype.declare = node_declare;
  ntype.labelfunc = input_node::node_label;
  ntype.no_muting = true;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace output_node

}  // namespace blender::nodes::node_geo_closure_cc
