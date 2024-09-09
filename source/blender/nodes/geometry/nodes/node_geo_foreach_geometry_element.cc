/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BLI_string_utf8.h"

#include "BLO_read_write.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "NOD_geo_foreach_geometry_element.hh"
#include "NOD_socket_items_ops.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BKE_screen.hh"

#include "WM_api.hh"

namespace blender::nodes::node_geo_foreach_geometry_element_cc {

static void draw_item(uiList * /*ui_list*/,
                      const bContext *C,
                      uiLayout *layout,
                      PointerRNA * /*idataptr*/,
                      PointerRNA *itemptr,
                      int /*icon*/,
                      PointerRNA * /*active_dataptr*/,
                      const char * /*active_propname*/,
                      int /*index*/,
                      int /*flt_flag*/)
{
  uiLayout *row = uiLayoutRow(layout, true);
  float4 color;
  RNA_float_get_array(itemptr, "color", color);
  uiTemplateNodeSocket(row, const_cast<bContext *>(C), color);
  uiLayoutSetEmboss(row, UI_EMBOSS_NONE);
  uiItemR(row, itemptr, "name", UI_ITEM_NONE, "", ICON_NONE);
}

/** Shared between zone input and output node. */
static void node_layout_ex(uiLayout *layout, bContext *C, PointerRNA *current_node_ptr)
{
  bNodeTree &ntree = *reinterpret_cast<bNodeTree *>(current_node_ptr->owner_id);
  bNode *current_node = static_cast<bNode *>(current_node_ptr->data);

  const bke::bNodeTreeZones *zones = ntree.zones();
  if (!zones) {
    return;
  }
  const bke::bNodeTreeZone *zone = zones->get_zone_by_node(current_node->identifier);
  if (!zone) {
    return;
  }
  if (!zone->output_node) {
    return;
  }
  const bool is_zone_input_node = current_node->type == GEO_NODE_FOREACH_GEOMETRY_ELEMENT_INPUT;
  bNode &output_node = const_cast<bNode &>(*zone->output_node);
  PointerRNA output_node_ptr = RNA_pointer_create(
      current_node_ptr->owner_id, &RNA_Node, &output_node);
  auto &storage = *static_cast<NodeGeometryForeachGeometryElementOutput *>(output_node.storage);

  if (uiLayout *panel = uiLayoutPanel(C, layout, "foreach_items", false, TIP_("Items"))) {
    if (is_zone_input_node) {
      static const uiListType *input_items_list = []() {
        uiListType *list = MEM_cnew<uiListType>(__func__);
        STRNCPY(list->idname, "DATA_UL_foreach_geometry_element_input_items");
        list->draw_item = draw_item;
        WM_uilisttype_add(list);
        return list;
      }();
      uiLayout *row = uiLayoutRow(panel, false);
      uiTemplateList(row,
                     C,
                     input_items_list->idname,
                     "",
                     &output_node_ptr,
                     "input_items",
                     &output_node_ptr,
                     "active_input_index",
                     nullptr,
                     3,
                     5,
                     UILST_LAYOUT_DEFAULT,
                     0,
                     UI_TEMPLATE_LIST_FLAG_NONE);
      {
        uiLayout *ops_col = uiLayoutColumn(row, false);
        {
          uiLayout *add_remove_col = uiLayoutColumn(ops_col, true);
          uiItemO(
              add_remove_col, "", ICON_ADD, "node.foreach_geometry_element_zone_input_item_add");
          uiItemO(add_remove_col,
                  "",
                  ICON_REMOVE,
                  "node.foreach_geometry_element_zone_input_item_remove");
        }
        {
          uiLayout *up_down_col = uiLayoutColumn(ops_col, true);
          uiItemEnumO(up_down_col,
                      "node.foreach_geometry_element_zone_input_item_move",
                      "",
                      ICON_TRIA_UP,
                      "direction",
                      0);
          uiItemEnumO(up_down_col,
                      "node.foreach_geometry_element_zone_input_item_move",
                      "",
                      ICON_TRIA_DOWN,
                      "direction",
                      1);
        }
      }

      if (storage.input_items.active_index >= 0 &&
          storage.input_items.active_index < storage.input_items.items_num)
      {
        NodeForeachGeometryElementInputItem &active_item =
            storage.input_items.items[storage.input_items.active_index];
        PointerRNA item_ptr = RNA_pointer_create(
            output_node_ptr.owner_id,
            ForeachGeometryElementInputItemsAccessor::item_srna,
            &active_item);
        uiLayoutSetPropSep(panel, true);
        uiLayoutSetPropDecorate(panel, false);
        uiItemR(panel, &item_ptr, "socket_type", UI_ITEM_NONE, nullptr, ICON_NONE);
      }
    }
    else {
      static const uiListType *output_items_list = []() {
        uiListType *list = MEM_cnew<uiListType>(__func__);
        STRNCPY(list->idname, "DATA_UL_foreach_geometry_element_output_items");
        list->draw_item = draw_item;
        WM_uilisttype_add(list);
        return list;
      }();
      uiLayout *row = uiLayoutRow(panel, false);
      uiTemplateList(row,
                     C,
                     output_items_list->idname,
                     "",
                     &output_node_ptr,
                     "output_items",
                     &output_node_ptr,
                     "active_output_index",
                     nullptr,
                     3,
                     5,
                     UILST_LAYOUT_DEFAULT,
                     0,
                     UI_TEMPLATE_LIST_FLAG_NONE);
      {
        uiLayout *ops_col = uiLayoutColumn(row, false);
        {
          uiLayout *add_remove_col = uiLayoutColumn(ops_col, true);
          uiItemO(
              add_remove_col, "", ICON_ADD, "node.foreach_geometry_element_zone_output_item_add");
          uiItemO(add_remove_col,
                  "",
                  ICON_REMOVE,
                  "node.foreach_geometry_element_zone_output_item_remove");
        }
        {
          uiLayout *up_down_col = uiLayoutColumn(ops_col, true);
          uiItemEnumO(up_down_col,
                      "node.foreach_geometry_element_zone_output_item_move",
                      "",
                      ICON_TRIA_UP,
                      "direction",
                      0);
          uiItemEnumO(up_down_col,
                      "node.foreach_geometry_element_zone_output_item_move",
                      "",
                      ICON_TRIA_DOWN,
                      "direction",
                      1);
        }
      }

      if (storage.output_items.active_index >= 0 &&
          storage.output_items.active_index < storage.output_items.items_num)
      {
        NodeForeachGeometryElementOutputItem &active_item =
            storage.output_items.items[storage.output_items.active_index];
        PointerRNA item_ptr = RNA_pointer_create(
            output_node_ptr.owner_id,
            ForeachGeometryElementOutputItemsAccessor::item_srna,
            &active_item);
        uiLayoutSetPropSep(panel, true);
        uiLayoutSetPropDecorate(panel, false);
        uiItemR(panel, &item_ptr, "socket_type", UI_ITEM_NONE, nullptr, ICON_NONE);
        uiItemR(panel, &item_ptr, "domain", UI_ITEM_NONE, nullptr, ICON_NONE);
      }
    }
  }

  uiItemR(layout, &output_node_ptr, "inspection_index", UI_ITEM_NONE, nullptr, ICON_NONE);
}

namespace input_node {

NODE_STORAGE_FUNCS(NodeGeometryForeachGeometryElementInput);

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Geometry").description("Geometry whose elements are iterated over");

  b.add_input<decl::Bool>("Selection")
      .default_value(true)
      .hide_value()
      .field_on_all()
      .description("Selection on the iteration domain");
  b.add_output<decl::Int>("Index").align_with_previous().description(
      "Index of the element in the source geometry. Note that the same index can occure more than "
      "once when iterating over multiple components at once");

  const bNode *node = b.node_or_null();
  const bNodeTree *tree = b.tree_or_null();
  if (node && tree) {
    const NodeGeometryForeachGeometryElementInput &storage = node_storage(*node);
    const bNode *output_node = tree->node_by_id(storage.output_node_id);
    if (output_node) {
      const auto &output_storage = *static_cast<const NodeGeometryForeachGeometryElementOutput *>(
          output_node->storage);
      for (const int i : IndexRange(output_storage.input_items.items_num)) {
        const NodeForeachGeometryElementInputItem &item = output_storage.input_items.items[i];
        const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
        const StringRef name = item.name ? item.name : "";
        const std::string identifier =
            ForeachGeometryElementInputItemsAccessor::socket_identifier_for_item(item);
        auto &input_decl =
            b.add_input(socket_type, name, identifier)
                .socket_name_ptr(
                    &tree->id, ForeachGeometryElementInputItemsAccessor::item_srna, &item, "name")
                .description("Field that is evaluated on the iteration domain");
        b.add_output(socket_type, name, identifier)
            .align_with_previous()
            .description("Evaluated field value for the current element");
        input_decl.supports_field();
      }
    }
  }
  b.add_input<decl::Extend>("", "__extend__");
  b.add_output<decl::Extend>("", "__extend__").align_with_previous();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  bNodeTree &tree = *reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bNode &node = *static_cast<bNode *>(ptr->data);
  const NodeGeometryForeachGeometryElementInput &storage = node_storage(node);
  bNode *output_node = tree.node_by_id(storage.output_node_id);

  PointerRNA output_node_ptr = RNA_pointer_create(ptr->owner_id, &RNA_Node, output_node);
  uiItemR(layout, &output_node_ptr, "domain", UI_ITEM_NONE, "", ICON_NONE);
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

static bool node_insert_link(bNodeTree *ntree, bNode *node, bNodeLink *link)
{
  bNode *output_node = ntree->node_by_id(node_storage(*node).output_node_id);
  if (!output_node) {
    return true;
  }
  return socket_items::try_add_item_via_any_extend_socket<
      ForeachGeometryElementInputItemsAccessor>(*ntree, *node, *output_node, *link);
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
  ntype.draw_buttons = node_layout;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.labelfunc = node_label;
  ntype.insert_link = node_insert_link;
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

  /* TODO: This propagates attributes from the geometry in the input node. */
  b.add_output<decl::Geometry>("Geometry")
      .description(
          "The original input geometry with potentially new attributes that are output by the "
          "zone");

  aal::RelationsInNode &relations = b.get_anonymous_attribute_relations();

  const bNode *node = b.node_or_null();
  const bNodeTree *tree = b.tree_or_null();
  if (node && tree) {
    const NodeGeometryForeachGeometryElementOutput &storage = node_storage(*node);
    int previous_geometry_index = 0;
    for (const int i : IndexRange(storage.output_items.items_num)) {
      const NodeForeachGeometryElementOutputItem &item = storage.output_items.items[i];
      const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
      if (socket_type == SOCK_GEOMETRY) {
        previous_geometry_index = i + 1;
      }
      const StringRef name = item.name ? item.name : "";
      std::string identifier =
          ForeachGeometryElementOutputItemsAccessor::socket_identifier_for_item(item);
      auto &input_decl = b.add_input(socket_type, name, identifier)
                             .socket_name_ptr(&tree->id,
                                              ForeachGeometryElementOutputItemsAccessor::item_srna,
                                              &item,
                                              "name");
      auto &output_decl = b.add_output(socket_type, name, identifier).align_with_previous();
      if (socket_type == SOCK_GEOMETRY) {
        output_decl.propagate_all();
        aal::PropagateRelation relation;
        relation.from_geometry_input = input_decl.index();
        relation.to_geometry_output = output_decl.index();
        relations.propagate_relations.append(relation);

        input_decl.description(
            "Geometry generated in the current iteration. Will be joined with geometries from all "
            "other iterations");
        output_decl.description("Result of joining generated geometries from each iteration");
      }
      else {
        if (previous_geometry_index > 0) {
          input_decl.supports_field();
          input_decl.description("Field that will be stored as attribute on the geometry above");
        }
        else {
          input_decl.description(
              "Attribute value that will be stored for the current element on the main geometry");
        }
        output_decl.field_on({previous_geometry_index});
        output_decl.description("Attribute on the geometry above");
      }
    }
  }

  b.add_input<decl::Extend>("", "__extend__");
  b.add_output<decl::Extend>("", "__extend__").align_with_previous();
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryForeachGeometryElementOutput *data =
      MEM_cnew<NodeGeometryForeachGeometryElementOutput>(__func__);

  node->storage = data;
}

static void node_free_storage(bNode *node)
{
  socket_items::destruct_array<ForeachGeometryElementInputItemsAccessor>(*node);
  socket_items::destruct_array<ForeachGeometryElementOutputItemsAccessor>(*node);
  MEM_freeN(node->storage);
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryForeachGeometryElementOutput &src_storage = node_storage(*src_node);
  auto *dst_storage = MEM_cnew<NodeGeometryForeachGeometryElementOutput>(__func__, src_storage);
  dst_node->storage = dst_storage;

  socket_items::copy_array<ForeachGeometryElementInputItemsAccessor>(*src_node, *dst_node);
  socket_items::copy_array<ForeachGeometryElementOutputItemsAccessor>(*src_node, *dst_node);
}

static bool node_insert_link(bNodeTree *ntree, bNode *node, bNodeLink *link)
{
  return socket_items::try_add_item_via_any_extend_socket<
      ForeachGeometryElementOutputItemsAccessor>(*ntree, *node, *node, *link);
}

static void NODE_OT_foreach_geometry_element_zone_input_item_remove(wmOperatorType *ot)
{
  socket_items::ops::remove_active_item<ForeachGeometryElementInputItemsAccessor>(
      ot, "Remove For-Each Input Item", __func__, "Remove active for-each input item");
}

static void NODE_OT_foreach_geometry_element_zone_input_item_add(wmOperatorType *ot)
{
  socket_items::ops::add_item<ForeachGeometryElementInputItemsAccessor>(
      ot, "Add For-Each Input Item", __func__, "Add for-each input item");
}

static void NODE_OT_foreach_geometry_element_zone_input_item_move(wmOperatorType *ot)
{
  socket_items::ops::move_active_item<ForeachGeometryElementInputItemsAccessor>(
      ot, "Move For-Each Input Item", __func__, "Move active for-each input item");
}

static void NODE_OT_foreach_geometry_element_zone_output_item_remove(wmOperatorType *ot)
{
  socket_items::ops::remove_active_item<ForeachGeometryElementOutputItemsAccessor>(
      ot, "Remove For-Each Output Item", __func__, "Remove active for-each output item");
}

static void NODE_OT_foreach_geometry_element_zone_output_item_add(wmOperatorType *ot)
{
  socket_items::ops::add_item<ForeachGeometryElementOutputItemsAccessor>(
      ot, "Add For-Each Output Item", __func__, "Add for-each output item");
}

static void NODE_OT_foreach_geometry_element_zone_output_item_move(wmOperatorType *ot)
{
  socket_items::ops::move_active_item<ForeachGeometryElementOutputItemsAccessor>(
      ot, "Move For-Each Output Item", __func__, "Move active for-each output item");
}

static void node_operators()
{
  WM_operatortype_append(NODE_OT_foreach_geometry_element_zone_input_item_add);
  WM_operatortype_append(NODE_OT_foreach_geometry_element_zone_input_item_remove);
  WM_operatortype_append(NODE_OT_foreach_geometry_element_zone_input_item_move);

  WM_operatortype_append(NODE_OT_foreach_geometry_element_zone_output_item_add);
  WM_operatortype_append(NODE_OT_foreach_geometry_element_zone_output_item_remove);
  WM_operatortype_append(NODE_OT_foreach_geometry_element_zone_output_item_move);
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
  ntype.insert_link = node_insert_link;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.register_operators = node_operators;
  ntype.no_muting = true;
  blender::bke::node_type_storage(
      &ntype, "NodeGeometryForeachGeometryElementOutput", node_free_storage, node_copy_storage);
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace output_node

}  // namespace blender::nodes::node_geo_foreach_geometry_element_cc

namespace blender::nodes {

StructRNA *ForeachGeometryElementOutputItemsAccessor::item_srna =
    &RNA_ForeachGeometryElementOutputItem;
int ForeachGeometryElementOutputItemsAccessor::node_type =
    GEO_NODE_FOREACH_GEOMETRY_ELEMENT_OUTPUT;

void ForeachGeometryElementOutputItemsAccessor::blend_write(BlendWriter *writer, const bNode &node)
{
  const auto &storage = *static_cast<const NodeGeometryForeachGeometryElementOutput *>(
      node.storage);
  BLO_write_struct_array(writer,
                         NodeForeachGeometryElementOutputItem,
                         storage.output_items.items_num,
                         storage.output_items.items);
  for (const NodeForeachGeometryElementOutputItem &item :
       Span(storage.output_items.items, storage.output_items.items_num))
  {
    BLO_write_string(writer, item.name);
  }
}

void ForeachGeometryElementOutputItemsAccessor::blend_read_data(BlendDataReader *reader,
                                                                bNode &node)
{
  auto &storage = *static_cast<NodeGeometryForeachGeometryElementOutput *>(node.storage);
  BLO_read_struct_array(reader,
                        NodeForeachGeometryElementOutputItem,
                        storage.output_items.items_num,
                        &storage.output_items.items);
  for (const NodeForeachGeometryElementOutputItem &item :
       Span(storage.output_items.items, storage.output_items.items_num))
  {
    BLO_read_string(reader, &item.name);
  }
}

StructRNA *ForeachGeometryElementInputItemsAccessor::item_srna =
    &RNA_ForeachGeometryElementInputItem;
int ForeachGeometryElementInputItemsAccessor::node_type = GEO_NODE_FOREACH_GEOMETRY_ELEMENT_OUTPUT;

void ForeachGeometryElementInputItemsAccessor::blend_write(BlendWriter *writer, const bNode &node)
{
  const auto &storage = *static_cast<const NodeGeometryForeachGeometryElementOutput *>(
      node.storage);
  BLO_write_struct_array(writer,
                         NodeForeachGeometryElementInputItem,
                         storage.input_items.items_num,
                         storage.input_items.items);
  for (const NodeForeachGeometryElementInputItem &item :
       Span(storage.input_items.items, storage.input_items.items_num))
  {
    BLO_write_string(writer, item.name);
  }
}

void ForeachGeometryElementInputItemsAccessor::blend_read_data(BlendDataReader *reader,
                                                               bNode &node)
{
  auto &storage = *static_cast<NodeGeometryForeachGeometryElementOutput *>(node.storage);
  BLO_read_struct_array(reader,
                        NodeForeachGeometryElementInputItem,
                        storage.input_items.items_num,
                        &storage.input_items.items);
  for (const NodeForeachGeometryElementInputItem &item :
       Span(storage.input_items.items, storage.input_items.items_num))
  {
    BLO_read_string(reader, &item.name);
  }
}

}  // namespace blender::nodes
