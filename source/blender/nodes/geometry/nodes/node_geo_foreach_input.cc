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

namespace blender::nodes::node_geo_foreach_input_cc {

NODE_STORAGE_FUNCS(NodeGeometryForEachInput);

static void node_declare_dynamic(const bNodeTree &tree,
                                 const bNode &node,
                                 NodeDeclaration &r_declaration)
{
  const NodeGeometryForEachInput &input_storage = node_storage(node);
  const bNode *output_node = tree.node_by_id(input_storage.output_node_id);
  if (output_node == nullptr) {
    r_declaration.skip_updating_sockets = true;
    return;
  }
  {
    NodeDeclarationBuilder b{r_declaration};
    b.add_input<decl::Int>("Amount").min(0).default_value(1);
    b.add_output<decl::Int>("Index");
  }
  const auto &output_storage = *static_cast<const NodeGeometryForEachOutput *>(
      output_node->storage);
  for (const int i : IndexRange(output_storage.input_items_num)) {
    const NodeForEachInputItem &item = output_storage.input_items[i];
    const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
    const std::string identifier = ForEachInputItemsAccessor::socket_identifier_for_item(item);
    {
      SocketDeclarationPtr decl = make_declaration_for_socket_type(socket_type);
      BLI_assert(decl);
      decl->name = StringRef(item.name);
      decl->identifier = identifier;
      decl->in_out = SOCK_IN;
      if (socket_type_supports_fields(socket_type)) {
        decl->input_field_type = InputSocketFieldType::IsSupported;
      }
      r_declaration.inputs.append(decl.get());
      r_declaration.items.append(std::move(decl));
    }
    {
      SocketDeclarationPtr decl = make_declaration_for_socket_type(socket_type);
      BLI_assert(decl);
      decl->name = StringRef(item.name);
      decl->identifier = identifier;
      decl->in_out = SOCK_OUT;
      r_declaration.outputs.append(decl.get());
      r_declaration.items.append(std::move(decl));
    }
  }
  SocketDeclarationPtr input_extend_decl = decl::create_extend_declaration(SOCK_IN);
  r_declaration.inputs.append(input_extend_decl.get());
  r_declaration.items.append(std::move(input_extend_decl));

  SocketDeclarationPtr output_extend_decl = decl::create_extend_declaration(SOCK_OUT);
  r_declaration.outputs.append(output_extend_decl.get());
  r_declaration.items.append(std::move(output_extend_decl));
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryForEachInput *data = MEM_cnew<NodeGeometryForEachInput>(__func__);
  /* Needs to be initialized for the node to work. */
  data->output_node_id = 0;
  node->storage = data;
}

static bool node_insert_link(bNodeTree *tree, bNode *node, bNodeLink *link)
{
  const NodeGeometryForEachInput &input_storage = node_storage(*node);
  bNode *output_node = tree->node_by_id(input_storage.output_node_id);
  if (output_node == nullptr) {
    return true;
  }
  return socket_items::try_add_item_via_any_extend_socket<ForEachInputItemsAccessor>(
      *tree, *node, *output_node, *link);
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_FOR_EACH_INPUT, "For-Each Input", NODE_CLASS_INTERFACE);
  ntype.initfunc = node_init;
  ntype.declare_dynamic = node_declare_dynamic;
  ntype.gather_link_search_ops = nullptr;
  ntype.insert_link = node_insert_link;
  node_type_storage(
      &ntype, "NodeGeometryForEachInput", node_free_standard_storage, node_copy_standard_storage);
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_foreach_input_cc
