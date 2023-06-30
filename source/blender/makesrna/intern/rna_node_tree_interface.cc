/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup RNA
 */

#include "DNA_node_tree_interface_types.h"

#include "RNA_define.h"
#include "RNA_enum_types.h"
#include "RNA_types.h"

#include "rna_internal.h"

#include "WM_types.h"

const EnumPropertyItem rna_enum_node_tree_interface_item_type_items[] = {
    {NODE_INTERFACE_SOCKET, "SOCKET", 0, "Socket", ""},
    {NODE_INTERFACE_PANEL, "PANEL", 0, "Panel", ""},
    {0, nullptr, 0, nullptr, nullptr}};

const EnumPropertyItem rna_enum_node_tree_interface_socket_kind_items[] = {
    {NODE_INTERFACE_INPUT, "INPUT", 0, "Input", ""},
    {NODE_INTERFACE_OUTPUT, "OUTPUT", 0, "Output", ""},
    {0, nullptr, 0, nullptr, nullptr}};

//#ifdef RNA_RUNTIME

#include "BKE_node.h"
#include "BKE_node_tree_interface.hh"
#include "BKE_node_tree_update.h"
#include "ED_node.h"
#include "WM_api.h"

static void rna_NodeTreeInterfaceItem_update(Main *bmain, Scene * /*scene*/, PointerRNA *ptr)
{
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(ptr->owner_id);
  BKE_ntree_update_tag_interface(ntree);
  ED_node_tree_propagate_change(nullptr, bmain, ntree);
}

static StructRNA *rna_NodeTreeInterfaceItem_refine(PointerRNA *ptr)
{
  bNodeTreeInterfaceItem *item = static_cast<bNodeTreeInterfaceItem *>(ptr->data);

  switch (item->item_type) {
    case NODE_INTERFACE_SOCKET:
      return &RNA_NodeTreeInterfaceSocket;
    case NODE_INTERFACE_PANEL:
      return &RNA_NodeTreeInterfacePanel;
    default:
      return &RNA_NodeTreeInterfaceItem;
  }
}

static void rna_NodeTreeInterfaceSocket_identifier_get(PointerRNA *ptr, char *value)
{
  bNodeTreeInterfaceSocket *socket = static_cast<bNodeTreeInterfaceSocket *>(ptr->data);
  strcpy(value, socket->socket_identifier().c_str());
}

static int rna_NodeTreeInterfaceSocket_identifier_length(PointerRNA *ptr)
{
  bNodeTreeInterfaceSocket *socket = static_cast<bNodeTreeInterfaceSocket *>(ptr->data);
  return socket->socket_identifier().length();
}

static int rna_NodeTreeInterfaceSocket_data_type_get(PointerRNA *ptr)
{
  bNodeTreeInterfaceSocket *socket = static_cast<bNodeTreeInterfaceSocket *>(ptr->data);
  return rna_node_socket_idname_to_enum(socket->data_type);
}

static void rna_NodeTreeInterfaceSocket_data_type_set(PointerRNA *ptr, int value)
{
  bNodeSocketType *typeinfo = rna_node_socket_type_from_enum(value);

  if (typeinfo) {
    bNodeTreeInterfaceSocket *socket = static_cast<bNodeTreeInterfaceSocket *>(ptr->data);
    socket->data_type = BLI_strdup(typeinfo->idname);
  }
}

static bool is_data_type_supported(bNodeTreeType *ntreetype, bNodeSocketType *socket_type)
{
  /* Check if the node tree supports the socket type. */
  if (ntreetype->valid_socket_type && !ntreetype->valid_socket_type(ntreetype, socket_type)) {
    return false;
  }

  /* Only use basic socket types for this enum. */
  if (socket_type->subtype != PROP_NONE) {
    return false;
  }

  if (!U.experimental.use_rotation_socket && socket_type->type == SOCK_ROTATION) {
    return false;
  }

  return true;
}

static bNodeSocketType *find_supported_data_type(bNodeTreeType *ntree_type)
{
  NODE_SOCKET_TYPES_BEGIN (socket_type) {
    if (is_data_type_supported(ntree_type, socket_type)) {
      return socket_type;
    }
  }
  NODE_SOCKET_TYPES_END;
  return nullptr;
}

static bool rna_NodeTreeInterfaceSocket_data_type_poll(void *userdata,
                                                       bNodeSocketType *socket_type)
{
  bNodeTreeType *ntreetype = static_cast<bNodeTreeType *>(userdata);
  return is_data_type_supported(ntreetype, socket_type);
}

static const EnumPropertyItem *rna_NodeTreeInterfaceSocket_data_type_itemf(bContext * /*C*/,
                                                                           PointerRNA *ptr,
                                                                           PropertyRNA * /*prop*/,
                                                                           bool *r_free)
{
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(ptr->owner_id);

  if (!ntree) {
    return DummyRNA_NULL_items;
  }

  return rna_node_socket_type_itemf(
      ntree->typeinfo, rna_NodeTreeInterfaceSocket_data_type_poll, r_free);
}

// static PointerRNA rna_NodeTreeInterfaceItems_active_get(PointerRNA *ptr)
//{
//  bNodeTreeInterface *interface = static_cast<bNodeTreeInterface *>(ptr->data);
//  PointerRNA r_ptr;
//  RNA_pointer_create(ptr->owner_id, &RNA_NodeTreeInterfaceItem, interface->active_item(),
//  &r_ptr); return r_ptr;
//}

// static void rna_NodeTreeInterfaceItems_active_set(PointerRNA *ptr,
//                                                  PointerRNA value,
//                                                  ReportList * /*reports*/)
//{
//  bNodeTreeInterface *interface = static_cast<bNodeTreeInterface *>(ptr->data);
//  bNodeTreeInterfaceItem *item = static_cast<bNodeTreeInterfaceItem *>(value.data);
//  interface->active_item_set(item);
//}

static bNodeTreeInterfaceSocket *rna_NodeTreeInterfaceItems_new_socket(
    ID *id,
    bNodeTreeInterface *interface,
    Main *bmain,
    ReportList *reports,
    const char *name,
    const char *description,
    int data_type_enum,
    int in_out)
{
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
  bNodeSocketType *typeinfo = rna_node_socket_type_from_enum(data_type_enum);
  if (typeinfo == nullptr) {
    BKE_report(reports, RPT_ERROR_INVALID_INPUT, "Unknown socket type");
    return nullptr;
  }

  /* If data type is unsupported try to find a valid type. */
  if (!is_data_type_supported(ntree->typeinfo, typeinfo)) {
    typeinfo = find_supported_data_type(ntree->typeinfo);
    if (typeinfo == nullptr) {
      BKE_report(reports, RPT_ERROR, "Could not find supported socket type");
      return nullptr;
    }
  }
  const char *data_type = typeinfo->idname;

  bNodeTreeInterfaceSocket *socket = interface->add_socket(
      name, description, data_type, eNodeTreeInterfaceSocketKind(in_out));

  if (socket == nullptr) {
    BKE_report(reports, RPT_ERROR, "Unable to create socket");
  }
  else {
    BKE_ntree_update_tag_interface(ntree);
    ED_node_tree_propagate_change(nullptr, bmain, ntree);
    WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
  }

  return socket;
}

static bNodeTreeInterfacePanel *rna_NodeTreeInterfaceItems_new_panel(
    ID *id, bNodeTreeInterface *interface, Main *bmain, ReportList *reports, const char *name)
{
  bNodeTreeInterfacePanel *panel = interface->add_panel(name);

  if (panel == nullptr) {
    BKE_report(reports, RPT_ERROR, "Unable to create panel");
  }
  else {
    bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
    BKE_ntree_update_tag_interface(ntree);
    ED_node_tree_propagate_change(nullptr, bmain, ntree);
    WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
  }

  return panel;
}

static bNodeTreeInterfaceItem *rna_NodeTreeInterfaceItems_copy(ID *id,
                                                               bNodeTreeInterface *interface,
                                                               Main *bmain,
                                                               ReportList *reports,
                                                               bNodeTreeInterfaceItem *item)
{
  const int index = interface->items().as_span().first_index_try(item);
  if (!interface->items().index_range().contains(index)) {
    return nullptr;
  }

  bNodeTreeInterfaceItem *item_copy = interface->insert_item_copy(*item, index + 1);

  if (item_copy == nullptr) {
    BKE_report(reports, RPT_ERROR, "Unable to copy item");
  }
  else {
    bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
    BKE_ntree_update_tag_interface(ntree);
    ED_node_tree_propagate_change(nullptr, bmain, ntree);
    WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
  }

  return item_copy;
}

static void rna_NodeTreeInterfaceItems_remove(ID *id,
                                              bNodeTreeInterface *interface,
                                              Main *bmain,
                                              bNodeTreeInterfaceItem *item)
{
  interface->remove_item(*item);

  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
  BKE_ntree_update_tag_interface(ntree);
  ED_node_tree_propagate_change(nullptr, bmain, ntree);
  WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
}

static void rna_NodeTreeInterfaceItems_clear(ID *id, bNodeTreeInterface *interface, Main *bmain)
{
  interface->clear_items();

  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
  BKE_ntree_update_tag_interface(ntree);
  ED_node_tree_propagate_change(nullptr, bmain, ntree);
  WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
}

static void rna_NodeTreeInterfaceItems_move(
    ID *id, bNodeTreeInterface *interface, Main *bmain, bNodeTreeInterfaceItem *item, int to_index)
{
  if (!interface->items().index_range().contains(to_index)) {
    return;
  }

  interface->move_item(*item, to_index);

  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
  BKE_ntree_update_tag_interface(ntree);
  ED_node_tree_propagate_change(nullptr, bmain, ntree);
  WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
}

#else

static void rna_def_node_interface_item(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "NodeTreeInterfaceItem", nullptr);
  RNA_def_struct_ui_text(srna, "Node Tree Interface Item", "Item in a node tree interface");
  RNA_def_struct_sdna(srna, "bNodeTreeInterfaceItem");
  RNA_def_struct_refine_func(srna, "rna_NodeTreeInterfaceItem_refine");

  prop = RNA_def_property(srna, "item_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "item_type");
  RNA_def_property_enum_items(prop, rna_enum_node_tree_interface_item_type_items);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(prop, "Item Type", "Type of interface item");
}

static void rna_def_node_interface_socket(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "NodeTreeInterfaceSocket", "NodeTreeInterfaceItem");
  RNA_def_struct_ui_text(srna, "Node Tree Interface Socket", "Declaration of a node socket");
  RNA_def_struct_sdna(srna, "bNodeTreeInterfaceSocket");

  prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Name", "Socket name");
  RNA_def_struct_name_property(srna, prop);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "identifier", PROP_STRING, PROP_NONE);
  RNA_def_property_string_funcs(prop,
                                "rna_NodeTreeInterfaceSocket_identifier_get",
                                "rna_NodeTreeInterfaceSocket_identifier_length",
                                nullptr);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(prop, "Identifier", "Unique identifier for mapping sockets");

  prop = RNA_def_property(srna, "description", PROP_STRING, PROP_NONE);
  RNA_def_property_string_sdna(prop, nullptr, "description");
  RNA_def_property_ui_text(prop, "Description", "Socket description");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "data_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, DummyRNA_DEFAULT_items);
  RNA_def_property_enum_funcs(prop,
                              "rna_NodeTreeInterfaceSocket_data_type_get",
                              "rna_NodeTreeInterfaceSocket_data_type_set",
                              "rna_NodeTreeInterfaceSocket_data_type_itemf");
  RNA_def_property_ui_text(prop, "Data Type", "Socket data type");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "kind", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "kind");
  RNA_def_property_enum_items(prop, rna_enum_node_tree_interface_socket_kind_items);
  RNA_def_property_flag(prop, PROP_ENUM_FLAG);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Kind", "Whether the socket is an input or output");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "is_input", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "kind", NODE_INTERFACE_INPUT);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Is Input", "Whether the socket is an input");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "is_output", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "kind", NODE_INTERFACE_OUTPUT);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Is Output", "Whether the socket is an output");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");
}

static void rna_def_node_interface_panel(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "NodeTreeInterfacePanel", "NodeTreeInterfaceItem");
  RNA_def_struct_ui_text(srna, "Node Tree Interface Item", "Declaration of a node panel");
  RNA_def_struct_sdna(srna, "bNodeTreeInterfacePanel");

  prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Name", "Panel name");
  RNA_def_struct_name_property(srna, prop);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "interface_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items_array", "items_num");
  RNA_def_property_struct_type(prop, "NodeTreeInterfaceItem");
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(prop, "Items", "Items in the node panel");
}

static void rna_def_node_tree_interface_items_api(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;
  PropertyRNA *prop;
  PropertyRNA *parm;
  FunctionRNA *func;

  RNA_def_property_srna(cprop, "NodeTreeInterfaceItems");
  srna = RNA_def_struct(brna, "NodeTreeInterfaceItems", nullptr);
  RNA_def_struct_sdna(srna, "bNodeTreeInterface");
  RNA_def_struct_ui_text(
      srna, "Node Tree Interface Items", "Collection of items in a node tree interface");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  //  prop = RNA_def_property(srna, "active", PROP_POINTER, PROP_NONE);
  //  RNA_def_property_struct_type(prop, "NodeTreeInterfaceItem");
  //  RNA_def_property_flag(prop, PROP_EDITABLE);
  //  RNA_def_property_pointer_funcs(prop,
  //                                 "rna_NodeTreeInterfaceItems_active_get",
  //                                 "rna_NodeTreeInterfaceItems_active_set",
  //                                 nullptr,
  //                                 nullptr);
  //  RNA_def_property_ui_text(prop, "Active", "Active item");
  //  RNA_def_property_update(prop, NC_NODE, nullptr);

  func = RNA_def_function(srna, "new_socket", "rna_NodeTreeInterfaceItems_new_socket");
  RNA_def_function_ui_description(func, "Add a new socket to the interface");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN | FUNC_USE_REPORTS);
  parm = RNA_def_string(func, "name", nullptr, 0, "Name", "Name of the socket");
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  RNA_def_string(func, "description", nullptr, 0, "Description", "Description of the socket");
  parm = RNA_def_enum(
      func, "data_type", DummyRNA_DEFAULT_items, 0, "Data Type", "Socket data type");
  /* Note: itemf callback works for the function parameter, it does not require a data pointer. */
  RNA_def_property_enum_funcs(
      parm, nullptr, nullptr, "rna_NodeTreeInterfaceSocket_data_type_itemf");
  RNA_def_enum_flag(func,
                    "kind",
                    rna_enum_node_tree_interface_socket_kind_items,
                    NODE_INTERFACE_INPUT,
                    "Socket Kind",
                    "Input/output kind of the socket");
  /* return value */
  parm = RNA_def_pointer(func, "item", "NodeTreeInterfaceSocket", "Socket", "New socket");
  RNA_def_function_return(func, parm);

  func = RNA_def_function(srna, "new_panel", "rna_NodeTreeInterfaceItems_new_panel");
  RNA_def_function_ui_description(func, "Add a new panel to the interface");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN | FUNC_USE_REPORTS);
  parm = RNA_def_string(func, "name", nullptr, 0, "Name", "Name of the new panel");
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  /* return value */
  parm = RNA_def_pointer(func, "item", "NodeTreeInterfacePanel", "Panel", "New panel");
  RNA_def_function_return(func, parm);

  func = RNA_def_function(srna, "copy", "rna_NodeTreeInterfaceItems_copy");
  RNA_def_function_ui_description(func, "Add a copy of an item to the interface");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN | FUNC_USE_REPORTS);
  parm = RNA_def_pointer(func, "item", "NodeTreeInterfaceItem", "Item", "Item to copy");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, PARM_REQUIRED);
  /* return value */
  parm = RNA_def_pointer(
      func, "item_copy", "NodeTreeInterfaceItem", "Item Copy", "Copy of the item");
  RNA_def_function_return(func, parm);

  func = RNA_def_function(srna, "remove", "rna_NodeTreeInterfaceItems_remove");
  RNA_def_function_ui_description(func, "Remove an item from the interface");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN);
  parm = RNA_def_pointer(func, "item", "NodeTreeInterfaceItem", "Item", "The item to remove");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, PARM_REQUIRED);

  func = RNA_def_function(srna, "clear", "rna_NodeTreeInterfaceItems_clear");
  RNA_def_function_ui_description(func, "Remove all items from the interface");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN);

  func = RNA_def_function(srna, "move", "rna_NodeTreeInterfaceItems_move");
  RNA_def_function_ui_description(func, "Move an item to another position");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN);
  parm = RNA_def_pointer(func, "item", "NodeTreeInterfaceItem", "Item", "The item to remove");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, PARM_REQUIRED);
  parm = RNA_def_int(
      func, "to_index", -1, 0, INT_MAX, "To Index", "Target index for the item", 0, 10000);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
}

static void rna_def_node_tree_interface(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "NodeTreeInterface", nullptr);
  RNA_def_struct_ui_text(
      srna, "Node Tree Interface", "Declaration of sockets and ui panels of a node group");
  RNA_def_struct_sdna(srna, "bNodeTreeInterface");

  rna_def_node_tree_interface_items_api(brna, prop);
}

void RNA_def_node_tree_interface(BlenderRNA *brna)
{
  rna_def_node_interface_item(brna);
  rna_def_node_interface_socket(brna);
  rna_def_node_interface_panel(brna);
  rna_def_node_tree_interface(brna);
}

#endif
