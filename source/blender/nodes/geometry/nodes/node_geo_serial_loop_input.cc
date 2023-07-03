/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_compute_contexts.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_geometry.hh"
#include "NOD_socket.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_serial_loop_input_cc {

NODE_STORAGE_FUNCS(NodeGeometrySerialLoopInput);

static void node_declare_dynamic(const bNodeTree &tree,
                                 const bNode &node,
                                 NodeDeclaration &r_declaration)
{
  NodeDeclarationBuilder b{r_declaration};
  b.add_input<decl::Int>(N_("Iterations")).min(0).default_value(1);

  const NodeGeometrySerialLoopInput &storage = node_storage(node);
  const bNode *output_node = tree.node_by_id(storage.output_node_id);
  if (output_node != nullptr) {
    const NodeGeometrySerialLoopOutput &output_storage =
        *static_cast<const NodeGeometrySerialLoopOutput *>(output_node->storage);
    socket_declarations_for_serial_loop_items(output_storage.items_span(), r_declaration);
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySerialLoopInput *data = MEM_cnew<NodeGeometrySerialLoopInput>(__func__);
  /* Needs to be initialized for the node to work. */
  data->output_node_id = 0;
  node->storage = data;
}

static bool node_insert_link(bNodeTree *ntree, bNode *node, bNodeLink *link)
{
  const bNode *output_node = ntree->node_by_id(node_storage(*node).output_node_id);
  if (!output_node) {
    return true;
  }
  auto &storage = *static_cast<NodeGeometrySerialLoopOutput *>(output_node->storage);
  if (link->tonode == node) {
    if (link->tosock->identifier == StringRef("__extend__")) {
      if (const NodeSerialLoopItem *item = storage.add_item(
              link->fromsock->name, eNodeSocketDatatype(link->fromsock->type)))
      {
        update_node_declaration_and_sockets(*ntree, *node);
        link->tosock = nodeFindSocket(node, SOCK_IN, item->identifier_str().c_str());
        return true;
      }
    }
    else {
      return true;
    }
  }
  if (link->fromnode == node) {
    if (link->fromsock->identifier == StringRef("__extend__")) {
      if (const NodeSerialLoopItem *item = storage.add_item(
              link->tosock->name, eNodeSocketDatatype(link->tosock->type)))
      {
        update_node_declaration_and_sockets(*ntree, *node);
        link->fromsock = nodeFindSocket(node, SOCK_OUT, item->identifier_str().c_str());
        return true;
      }
    }
    else {
      return true;
    }
  }
  return false;
}

}  // namespace blender::nodes::node_geo_serial_loop_input_cc

void register_node_type_geo_serial_loop_input()
{
  namespace file_ns = blender::nodes::node_geo_serial_loop_input_cc;

  static bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_SERIAL_LOOP_INPUT, "Serial Loop Input", NODE_CLASS_INTERFACE);
  ntype.initfunc = file_ns::node_init;
  ntype.declare_dynamic = file_ns::node_declare_dynamic;
  ntype.gather_add_node_search_ops = nullptr;
  ntype.gather_link_search_ops = nullptr;
  ntype.insert_link = file_ns::node_insert_link;
  node_type_storage(&ntype,
                    "NodeGeometrySerialLoopInput",
                    node_free_standard_storage,
                    node_copy_standard_storage);
  nodeRegisterType(&ntype);
}

bool NOD_geometry_serial_loop_input_pair_with_output(const bNodeTree *node_tree,
                                                     bNode *serial_loop_input_node,
                                                     const bNode *serial_loop_output_node)
{
  namespace file_ns = blender::nodes::node_geo_serial_loop_input_cc;

  BLI_assert(serial_loop_input_node->type == GEO_NODE_SERIAL_LOOP_INPUT);
  if (serial_loop_output_node->type != GEO_NODE_SERIAL_LOOP_OUTPUT) {
    return false;
  }

  /* Allow only one input paired to an output. */
  for (const bNode *other_input_node : node_tree->nodes_by_type("GeometryNodeSerialLoopInput")) {
    if (other_input_node != serial_loop_input_node) {
      const NodeGeometrySerialLoopInput &other_storage = file_ns::node_storage(*other_input_node);
      if (other_storage.output_node_id == serial_loop_output_node->identifier) {
        return false;
      }
    }
  }

  NodeGeometrySerialLoopInput &storage = file_ns::node_storage(*serial_loop_input_node);
  storage.output_node_id = serial_loop_output_node->identifier;
  return true;
}
