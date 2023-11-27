/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_rna_define.hh"
#include "NOD_zone_socket_items.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLI_string.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_bake_cc {

NODE_STORAGE_FUNCS(NodeGeometryBake)

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }
  const NodeGeometryBake &storage = node_storage(*node);

  for (const int i : IndexRange(storage.items_num)) {
    const NodeGeometryBakeItem &item = storage.items[i];
    const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
    const StringRef name = item.name;
    const std::string identifier = BakeItemsAccessor::socket_identifier_for_item(item);
    auto &input_decl = b.add_input(socket_type, name, identifier);
    auto &output_decl = b.add_output(socket_type, name, identifier);
    if (socket_type_supports_fields(socket_type)) {
      input_decl.supports_field();
      output_decl.dependent_field({input_decl.input_index()});
    }
  }
  b.add_input<decl::Extend>("", "__extend__");
  b.add_output<decl::Extend>("", "__extend__");
}

static void node_layout(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  UNUSED_VARS(layout, C, ptr);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryBake *data = MEM_cnew<NodeGeometryBake>(__func__);

  data->items = MEM_cnew_array<NodeGeometryBakeItem>(1, __func__);
  data->items_num = 1;

  NodeGeometryBakeItem &item = data->items[0];
  item.name = BLI_strdup("Geometry");
  item.identifier = data->next_identifier++;
  item.attribute_domain = ATTR_DOMAIN_POINT;
  item.socket_type = SOCK_GEOMETRY;

  node->storage = data;
}

static void node_free_storage(bNode *node)
{
  socket_items::destruct_array<BakeItemsAccessor>(*node);
  MEM_freeN(node->storage);
}

static void node_copy_storage(bNodeTree * /*tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryBake &src_storage = node_storage(*src_node);
  auto *dst_storage = MEM_new<NodeGeometryBake>(__func__, src_storage);
  dst_node->storage = dst_storage;

  socket_items::copy_array<BakeItemsAccessor>(*src_node, *dst_node);
}

static bool node_insert_link(bNodeTree *ntree, bNode *node, bNodeLink *link)
{
  return socket_items::try_add_item_via_any_extend_socket<BakeItemsAccessor>(
      *ntree, *node, *node, *link);
}

const CPPType &get_item_cpp_type(const eNodeSocketDatatype socket_type)
{
  const char *socket_idname = nodeStaticSocketType(socket_type, 0);
  const bNodeSocketType *typeinfo = nodeSocketTypeFind(socket_idname);
  BLI_assert(typeinfo);
  BLI_assert(typeinfo->geometry_nodes_cpp_type);
  return *typeinfo->geometry_nodes_cpp_type;
}

class LazyFunctionForBakeNode final : public LazyFunction {
  const bNode &node_;
  Span<NodeGeometryBakeItem> bake_items_;

 public:
  LazyFunctionForBakeNode(const bNode &node, GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
      : node_(node)
  {
    debug_name_ = "Bake";
    const NodeGeometryBake &storage = node_storage(node);
    bake_items_ = {storage.items, storage.items_num};

    MutableSpan<int> lf_index_by_bsocket = lf_graph_info.mapping.lf_index_by_bsocket;

    for (const int i : bake_items_.index_range()) {
      const NodeGeometryBakeItem &item = bake_items_[i];
      const bNodeSocket &input_bsocket = node.input_socket(i);
      const bNodeSocket &output_bsocket = node.output_socket(i);
      const CPPType &type = get_item_cpp_type(eNodeSocketDatatype(item.socket_type));
      lf_index_by_bsocket[input_bsocket.index_in_tree()] = inputs_.append_and_get_index_as(
          item.name, type, lf::ValueUsage::Maybe);
      lf_index_by_bsocket[output_bsocket.index_in_tree()] = outputs_.append_and_get_index_as(
          item.name, type);
    }
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const final
  {
    GeoNodesLFUserData &user_data = *static_cast<GeoNodesLFUserData *>(context.user_data);
    UNUSED_VARS(user_data);
    params.set_default_remaining_outputs();
  }
};

static void node_rna(StructRNA *srna)
{
  UNUSED_VARS(srna);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_BAKE, "Bake", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.insert_link = node_insert_link;
  node_type_storage(&ntype, "NodeGeometryBake", node_free_storage, node_copy_storage);
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_bake_cc

namespace blender::nodes {

std::unique_ptr<LazyFunction> get_bake_lazy_function(
    const bNode &node, GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
{
  namespace file_ns = blender::nodes::node_geo_bake_cc;
  BLI_assert(node.type == GEO_NODE_BAKE);
  return std::make_unique<file_ns::LazyFunctionForBakeNode>(node, lf_graph_info);
}

};  // namespace blender::nodes
