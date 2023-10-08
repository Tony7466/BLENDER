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

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_foreach_input_cc {

NODE_STORAGE_FUNCS(NodeGeometryForEachInput);

static void node_declare_dynamic(const bNodeTree &tree,
                                 const bNode &node,
                                 NodeDeclarationBuilder &b)
{
  const NodeGeometryForEachInput &input_storage = node_storage(node);
  const bNode *output_node = tree.node_by_id(input_storage.output_node_id);
  if (output_node == nullptr) {
    return;
  }
  const auto &output_storage = *static_cast<const NodeGeometryForEachOutput *>(
      output_node->storage);
  const GeometryNodeForEachMode mode = GeometryNodeForEachMode(output_storage.mode);
  {
    /* Add standard inputs. */
    switch (mode) {
      case GEO_NODE_FOR_EACH_MODE_INDEX: {
        b.add_input<decl::Int>("Amount").min(0).default_value(1);
        b.add_output<decl::Int>("Index");
        break;
      }
      case GEO_NODE_FOR_EACH_MODE_GEOMETRY_ELEMENT: {
        b.add_input<decl::Geometry>("Geometry");
        b.add_input<decl::Bool>("Selection").default_value(true).hide_value(true).supports_field();
        b.add_output<decl::Int>("Index");
        if (output_storage.domain != ATTR_DOMAIN_CORNER) {
          b.add_output<decl::Geometry>("Element");
        }
        break;
      }
      case GEO_NODE_FOR_EACH_MODE_INSTANCE: {
        b.add_input<decl::Geometry>("Instances");
        b.add_output<decl::Geometry>("Geometry");
        break;
      }
    }
  }
  if (mode == GEO_NODE_FOR_EACH_MODE_INSTANCE) {
    /* Other inputs are not allowed in this mode. */
    return;
  }
  /* Add dynamic sockets. */
  for (const int i : IndexRange(output_storage.input_items_num)) {
    const NodeForEachInputItem &item = output_storage.input_items[i];
    const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
    const StringRef name = item.name;
    const std::string identifier = ForEachInputItemsAccessor::socket_identifier_for_item(item);
    auto &input_decl = b.add_input(socket_type, name, identifier);
    b.add_output(socket_type, name, identifier);
    if (socket_type_supports_fields(socket_type)) {
      input_decl.supports_field();
    }
  }
  b.add_input<decl::Extend>("", "__extend__");
  b.add_output<decl::Extend>("", "__extend__");
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

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  bNodeTree &tree = *reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bNode &input_node = *static_cast<bNode *>(ptr->data);
  const NodeGeometryForEachInput &input_storage = node_storage(input_node);
  bNode *output_node = tree.node_by_id(input_storage.output_node_id);
  if (output_node == nullptr) {
    return;
  }
  const auto &output_storage = *static_cast<const NodeGeometryForEachOutput *>(
      output_node->storage);
  PointerRNA output_node_ptr = RNA_pointer_create(ptr->owner_id, &RNA_Node, output_node);

  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiLayout *col = uiLayoutColumn(layout, false);
  uiItemR(col, &output_node_ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
  if (output_storage.mode == GEO_NODE_FOR_EACH_MODE_GEOMETRY_ELEMENT) {
    uiItemR(col, &output_node_ptr, "domain", UI_ITEM_NONE, "Domain", ICON_NONE);
  }
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_FOR_EACH_INPUT, "For-Each Input", NODE_CLASS_INTERFACE);
  ntype.initfunc = node_init;
  ntype.declare_dynamic = node_declare_dynamic;
  ntype.gather_link_search_ops = nullptr;
  ntype.insert_link = node_insert_link;
  ntype.draw_buttons = node_layout;
  node_type_storage(
      &ntype, "NodeGeometryForEachInput", node_free_standard_storage, node_copy_standard_storage);
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_foreach_input_cc
