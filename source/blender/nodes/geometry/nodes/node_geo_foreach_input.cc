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

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_foreach_input_cc {

NODE_STORAGE_FUNCS(NodeGeometryForEachInput);

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Int>("Amount").min(0).default_value(1);
  b.add_output<decl::Int>("Index");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryForEachInput *data = MEM_cnew<NodeGeometryForEachInput>(__func__);
  /* Needs to be initialized for the node to work. */
  data->output_node_id = 0;
  node->storage = data;
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_FOR_EACH_INPUT, "For-Each Input", NODE_CLASS_INTERFACE);
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = nullptr;
  node_type_storage(
      &ntype, "NodeGeometryForEachInput", node_free_standard_storage, node_copy_standard_storage);
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_foreach_input_cc
