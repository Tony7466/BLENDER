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

#ifdef RNA_RUNTIME

#  include "BKE_node.h"
#  include "BKE_node_tree_interface.hh"
#  include "BKE_node_tree_update.h"
#  include "ED_node.h"
#  include "WM_api.h"

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
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket *socket = reinterpret_cast<bNodeTreeInterfaceSocket *>(item);
      if (STREQ(socket->socket_type, "NodeSocketFloat")) {
        return &RNA_NodeTreeInterfaceSocketFloat;
      }
      else if (STREQ(socket->socket_type, "NodeSocketInt")) {
        return &RNA_NodeTreeInterfaceSocketInt;
      }
      else if (STREQ(socket->socket_type, "NodeSocketBool")) {
        return &RNA_NodeTreeInterfaceSocketBool;
      }
      else if (STREQ(socket->socket_type, "NodeSocketString")) {
        return &RNA_NodeTreeInterfaceSocketString;
      }
      else if (STREQ(socket->socket_type, "NodeSocketObject")) {
        return &RNA_NodeTreeInterfaceSocketObject;
      }
      else {
        return &RNA_NodeTreeInterfaceSocket;
      }
    }
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

static int rna_NodeTreeInterfaceSocket_socket_type_get(PointerRNA *ptr)
{
  bNodeTreeInterfaceSocket *socket = static_cast<bNodeTreeInterfaceSocket *>(ptr->data);
  return rna_node_socket_idname_to_enum(socket->socket_type);
}

static void rna_NodeTreeInterfaceSocket_socket_type_set(PointerRNA *ptr, int value)
{
  bNodeSocketType *typeinfo = rna_node_socket_type_from_enum(value);

  if (typeinfo) {
    bNodeTreeInterfaceSocket *socket = static_cast<bNodeTreeInterfaceSocket *>(ptr->data);
    socket->socket_type = BLI_strdup(typeinfo->idname);
  }
}

static bool is_socket_type_supported(bNodeTreeType *ntreetype, bNodeSocketType *socket_type)
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

static bNodeSocketType *find_supported_socket_type(bNodeTreeType *ntree_type)
{
  NODE_SOCKET_TYPES_BEGIN (socket_type) {
    if (is_socket_type_supported(ntree_type, socket_type)) {
      return socket_type;
    }
  }
  NODE_SOCKET_TYPES_END;
  return nullptr;
}

static bool rna_NodeTreeInterfaceSocket_socket_type_poll(void *userdata,
                                                         bNodeSocketType *socket_type)
{
  bNodeTreeType *ntreetype = static_cast<bNodeTreeType *>(userdata);
  return is_socket_type_supported(ntreetype, socket_type);
}

static const EnumPropertyItem *rna_NodeTreeInterfaceSocket_socket_type_itemf(
    bContext * /*C*/, PointerRNA *ptr, PropertyRNA * /*prop*/, bool *r_free)
{
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(ptr->owner_id);

  if (!ntree) {
    return DummyRNA_NULL_items;
  }

  return rna_node_socket_type_itemf(
      ntree->typeinfo, rna_NodeTreeInterfaceSocket_socket_type_poll, r_free);
}

static void rna_NodeTreeInterfaceSocketFloat_range(
    PointerRNA *ptr, float *min, float *max, float *softmin, float *softmax)
{
  bNodeTreeInterfaceSocketFloat *socket = static_cast<bNodeTreeInterfaceSocketFloat *>(ptr->data);

  *min = (socket->subtype == PROP_UNSIGNED ? 0.0f : -FLT_MAX);
  *max = FLT_MAX;
  *softmin = socket->min_value;
  *softmax = socket->max_value;
}

static void rna_NodeTreeInterfaceSocketInt_range(
    PointerRNA *ptr, int *min, int *max, int *softmin, int *softmax)
{
  bNodeTreeInterfaceSocketInt *socket = static_cast<bNodeTreeInterfaceSocketInt *>(ptr->data);

  *min = (socket->subtype == PROP_UNSIGNED ? 0 : INT_MIN);
  *max = INT_MAX;
  *softmin = socket->min_value;
  *softmax = socket->max_value;
}

static PointerRNA rna_NodeTreeInterfaceItems_active_get(PointerRNA *ptr)
{
  bNodeTreeInterface *interface = static_cast<bNodeTreeInterface *>(ptr->data);
  PointerRNA r_ptr;
  RNA_pointer_create(ptr->owner_id, &RNA_NodeTreeInterfaceItem, interface->active_item(), &r_ptr);
  return r_ptr;
}

static void rna_NodeTreeInterfaceItems_active_set(PointerRNA *ptr,
                                                  PointerRNA value,
                                                  ReportList * /*reports*/)
{
  bNodeTreeInterface *interface = static_cast<bNodeTreeInterface *>(ptr->data);
  bNodeTreeInterfaceItem *item = static_cast<bNodeTreeInterfaceItem *>(value.data);
  interface->active_item_set(item);
}

static bNodeTreeInterfaceSocket *rna_NodeTreeInterfaceItems_new_socket(
    ID *id,
    bNodeTreeInterface *interface,
    Main *bmain,
    ReportList *reports,
    const char *name,
    const char *description,
    int socket_type_enum,
    bNodeTreeInterfacePanel *parent)
{
  if (parent != nullptr && !interface->find_item(parent->item)) {
    BKE_report(reports, RPT_ERROR_INVALID_INPUT, "Parent is not part of the interface");
    return nullptr;
  }
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
  bNodeSocketType *typeinfo = rna_node_socket_type_from_enum(socket_type_enum);
  if (typeinfo == nullptr) {
    BKE_report(reports, RPT_ERROR_INVALID_INPUT, "Unknown socket type");
    return nullptr;
  }

  /* If data type is unsupported try to find a valid type. */
  if (!is_socket_type_supported(ntree->typeinfo, typeinfo)) {
    typeinfo = find_supported_socket_type(ntree->typeinfo);
    if (typeinfo == nullptr) {
      BKE_report(reports, RPT_ERROR, "Could not find supported socket type");
      return nullptr;
    }
  }
  const char *socket_type = typeinfo->idname;

  const eNodeTreeInterfaceSocketFlag default_flags = NODE_INTERFACE_SOCKET_INPUT;
  bNodeTreeInterfaceSocket *socket = interface->add_socket(
      name, description, socket_type, default_flags, parent);

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
    ID *id,
    bNodeTreeInterface *interface,
    Main *bmain,
    ReportList *reports,
    const char *name,
    bNodeTreeInterfacePanel *parent)
{
  if (parent != nullptr && !interface->find_item(parent->item)) {
    BKE_report(reports, RPT_ERROR_INVALID_INPUT, "Parent is not part of the interface");
    return nullptr;
  }

  bNodeTreeInterfacePanel *panel = interface->add_panel(name, parent);

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
                                                               bNodeTreeInterfaceItem *item,
                                                               bNodeTreeInterfacePanel *parent)
{
  if (parent != nullptr && !interface->find_item(parent->item)) {
    BKE_report(reports, RPT_ERROR_INVALID_INPUT, "Parent is not part of the interface");
    return nullptr;
  }

  if (parent == nullptr) {
    parent = &interface->root_panel;
  }
  const int index = parent->items().as_span().first_index_try(item);
  if (!parent->items().index_range().contains(index)) {
    return nullptr;
  }

  bNodeTreeInterfaceItem *item_copy = interface->insert_item_copy(*item, parent, index + 1);

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

  prop = RNA_def_property(srna, "socket_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, DummyRNA_DEFAULT_items);
  RNA_def_property_enum_funcs(prop,
                              "rna_NodeTreeInterfaceSocket_socket_type_get",
                              "rna_NodeTreeInterfaceSocket_socket_type_set",
                              "rna_NodeTreeInterfaceSocket_socket_type_itemf");
  RNA_def_property_ui_text(
      prop, "Socket Type", "Type of the socket generated by this interface item");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "is_input", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NODE_INTERFACE_SOCKET_INPUT);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Is Input", "Whether the socket is an input");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "is_output", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NODE_INTERFACE_SOCKET_OUTPUT);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Is Output", "Whether the socket is an output");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "hide_value", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NODE_INTERFACE_SOCKET_HIDE_VALUE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(
      prop, "Hide Value", "Hide the socket input value even when the socket is not connected");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "hide_in_modifier", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NODE_INTERFACE_SOCKET_HIDE_IN_MODIFIER);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop,
                           "Hide in Modifier",
                           "Don't show the input value in the geometry nodes modifier interface");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "attribute_domain", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_attribute_domain_items);
  RNA_def_property_ui_text(
      prop,
      "Attribute Domain",
      "Attribute domain used by the geometry nodes modifier to create an attribute output");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "default_attribute_name", PROP_STRING, PROP_NONE);
  RNA_def_property_string_sdna(prop, nullptr, "default_attribute_name");
  RNA_def_property_ui_text(prop,
                           "Default Attribute",
                           "The attribute name used by default when the node group is used by a "
                           "geometry nodes modifier");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");
}

static void rna_def_node_interface_socket_float(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  static EnumPropertyItem rna_enum_subtype_items[] = {
      {PROP_PIXEL, "PIXEL", 0, "Pixel", ""},
      {PROP_UNSIGNED, "UNSIGNED", 0, "Unsigned", ""},
      {PROP_PERCENTAGE, "PERCENTAGE", 0, "Percentage", ""},
      {PROP_FACTOR, "FACTOR", 0, "Factor", ""},
      {PROP_ANGLE, "ANGLE", 0, "Angle", ""},
      {PROP_TIME,
       "TIME",
       0,
       "Time (Scene Relative)",
       "Time specified in frames, converted to seconds based on scene frame rate"},
      {PROP_TIME_ABSOLUTE,
       "TIME_ABSOLUTE",
       0,
       "Time (Absolute)",
       "Time specified in seconds, independent of the scene"},
      {PROP_DISTANCE, "DISTANCE", 0, "Distance", ""},
      {PROP_DISTANCE_CAMERA, "DISTANCE_CAMERA", 0, "Camera Distance", ""},
      {PROP_POWER, "POWER", 0, "Power", ""},
      {PROP_TEMPERATURE, "TEMPERATURE", 0, "Temperature", ""},
      {0, NULL, 0, NULL, NULL}};

  srna = RNA_def_struct(brna, "NodeTreeInterfaceSocketFloat", "NodeTreeInterfaceSocket");
  RNA_def_struct_ui_text(srna,
                         "Node Tree Interface Socket Float",
                         "Declaration of a floating point value node socket");
  RNA_def_struct_sdna(srna, "bNodeTreeInterfaceSocketFloat");

  prop = RNA_def_property(srna, "subtype", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_subtype_items);
  RNA_def_property_ui_text(prop, "Subtype", "Subtype of the default value of the socket");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "default_value", PROP_FLOAT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_float_funcs(prop, nullptr, nullptr, "rna_NodeTreeInterfaceSocketFloat_range");
  RNA_def_property_ui_text(prop, "Default Value", "Input value used for unconnected socket");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "min_value", PROP_FLOAT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Minimum Value", "Minimum value");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "max_value", PROP_FLOAT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Maximum Value", "Maximum value");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");
}

static void rna_def_node_interface_socket_int(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  static EnumPropertyItem rna_enum_subtype_items[] = {
      {PROP_PIXEL, "PIXEL", 0, "Pixel", ""},
      {PROP_UNSIGNED, "UNSIGNED", 0, "Unsigned", ""},
      {PROP_PERCENTAGE, "PERCENTAGE", 0, "Percentage", ""},
      {PROP_ANGLE, "ANGLE", 0, "Angle", ""},
      {PROP_TIME,
       "TIME",
       0,
       "Time (Scene Relative)",
       "Time specified in frames, converted to seconds based on scene frame rate"},
      {PROP_TIME_ABSOLUTE,
       "TIME_ABSOLUTE",
       0,
       "Time (Absolute)",
       "Time specified in seconds, independent of the scene"},
      {PROP_DISTANCE, "DISTANCE", 0, "Distance", ""},
      {PROP_DISTANCE_CAMERA, "DISTANCE_CAMERA", 0, "Camera Distance", ""},
      {PROP_POWER, "POWER", 0, "Power", ""},
      {PROP_TEMPERATURE, "TEMPERATURE", 0, "Temperature", ""},
      {0, NULL, 0, NULL, NULL}};

  srna = RNA_def_struct(brna, "NodeTreeInterfaceSocketInt", "NodeTreeInterfaceSocket");
  RNA_def_struct_ui_text(
      srna, "Node Tree Int Socket Interface", "Declaration of an integer value node socket");
  RNA_def_struct_sdna(srna, "bNodeTreeInterfaceSocketInt");

  prop = RNA_def_property(srna, "subtype", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_subtype_items);
  RNA_def_property_ui_text(prop, "Subtype", "Subtype of the default value of the socket");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "default_value", PROP_INT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_int_funcs(prop, nullptr, nullptr, "rna_NodeTreeInterfaceSocketInt_range");
  RNA_def_property_ui_text(prop, "Default Value", "Input value used for unconnected socket");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "min_value", PROP_INT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Minimum Value", "Minimum value");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "max_value", PROP_INT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Maximum Value", "Maximum value");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");
}

static void rna_def_node_interface_socket_bool(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "NodeTreeInterfaceSocketBool", "NodeTreeInterfaceSocket");
  RNA_def_struct_ui_text(
      srna, "Node Tree Bool Socket Interface", "Declaration of a boolean value node socket");
  RNA_def_struct_sdna(srna, "bNodeTreeInterfaceSocketBool");

  prop = RNA_def_property(srna, "default_value", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Default Value", "Input value used for unconnected socket");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");
}

static void rna_def_node_interface_socket_string(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  static EnumPropertyItem rna_enum_subtype_items[] = {
      {PROP_FILEPATH, "FILE_PATH", 0, "File Path", ""},
      {PROP_DIRPATH, "DIR_PATH", 0, "Directory Path", ""},
      {PROP_FILENAME, "FILE_NAME", 0, "File Name", ""},
      {PROP_BYTESTRING, "BYTE_STRING", 0, "Byte String", ""},
      {PROP_PASSWORD, "PASSWORD", 0, "Password", "A string that is displayed hidden ('********')"},
      {0, NULL, 0, NULL, NULL}};

  srna = RNA_def_struct(brna, "NodeTreeInterfaceSocketString", "NodeTreeInterfaceSocket");
  RNA_def_struct_ui_text(
      srna, "Node Tree String Socket Interface", "Declaration of a string value node socket");
  RNA_def_struct_sdna(srna, "bNodeTreeInterfaceSocketString");

  prop = RNA_def_property(srna, "subtype", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_subtype_items);
  RNA_def_property_ui_text(prop, "Subtype", "Subtype of the default value of the socket");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");

  prop = RNA_def_property(srna, "default_value", PROP_STRING, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Default Value", "Input value used for unconnected socket");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");
}

static void rna_def_node_interface_socket_object(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "NodeTreeInterfaceSocketObject", "NodeTreeInterfaceSocket");
  RNA_def_struct_ui_text(
      srna, "Node Tree Object Socket Interface", "Declaration of an object node socket");
  RNA_def_struct_sdna(srna, "bNodeTreeInterfaceSocketObject");

  prop = RNA_def_property(srna, "default_value", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "Object");
  RNA_def_property_ui_text(prop, "Default Value", "Input value used for unconnected socket");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeTreeInterfaceItem_update");
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_ID_REFCOUNT);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
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

static void rna_def_node_tree_interface_items_api(StructRNA *srna)
{
  PropertyRNA *prop;
  PropertyRNA *parm;
  FunctionRNA *func;

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "NodeTreeInterfaceItem");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_pointer_funcs(prop,
                                 "rna_NodeTreeInterfaceItems_active_get",
                                 "rna_NodeTreeInterfaceItems_active_set",
                                 nullptr,
                                 nullptr);
  RNA_def_property_ui_text(prop, "Active", "Active item");
  RNA_def_property_update(prop, NC_NODE, nullptr);

  func = RNA_def_function(srna, "new_socket", "rna_NodeTreeInterfaceItems_new_socket");
  RNA_def_function_ui_description(func, "Add a new socket to the interface");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN | FUNC_USE_REPORTS);
  parm = RNA_def_string(func, "name", nullptr, 0, "Name", "Name of the socket");
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  RNA_def_string(func, "description", nullptr, 0, "Description", "Description of the socket");
  parm = RNA_def_enum(func,
                      "socket_type",
                      DummyRNA_DEFAULT_items,
                      0,
                      "Socket Type",
                      "Type of socket generated on nodes");
  /* Note: itemf callback works for the function parameter, it does not require a data pointer. */
  RNA_def_property_enum_funcs(
      parm, nullptr, nullptr, "rna_NodeTreeInterfaceSocket_socket_type_itemf");
  RNA_def_pointer(
      func, "parent", "NodeTreeInterfacePanel", "Parent", "Panel to add the socket in");
  /* return value */
  parm = RNA_def_pointer(func, "item", "NodeTreeInterfaceSocket", "Socket", "New socket");
  RNA_def_function_return(func, parm);

  func = RNA_def_function(srna, "new_panel", "rna_NodeTreeInterfaceItems_new_panel");
  RNA_def_function_ui_description(func, "Add a new panel to the interface");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN | FUNC_USE_REPORTS);
  parm = RNA_def_string(func, "name", nullptr, 0, "Name", "Name of the new panel");
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  RNA_def_pointer(func,
                  "parent",
                  "NodeTreeInterfacePanel",
                  "Parent",
                  "Add panel as a child of the parent panel");
  /* return value */
  parm = RNA_def_pointer(func, "item", "NodeTreeInterfacePanel", "Panel", "New panel");
  RNA_def_function_return(func, parm);

  func = RNA_def_function(srna, "copy", "rna_NodeTreeInterfaceItems_copy");
  RNA_def_function_ui_description(func, "Add a copy of an item to the interface");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN | FUNC_USE_REPORTS);
  parm = RNA_def_pointer(func, "item", "NodeTreeInterfaceItem", "Item", "Item to copy");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, PARM_REQUIRED);
  RNA_def_pointer(func, "parent", "NodeTreeInterfacePanel", "Parent", "Panel to add the item in");
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

  srna = RNA_def_struct(brna, "NodeTreeInterface", nullptr);
  RNA_def_struct_ui_text(
      srna, "Node Tree Interface", "Declaration of sockets and ui panels of a node group");
  RNA_def_struct_sdna(srna, "bNodeTreeInterface");

  rna_def_node_tree_interface_items_api(srna);
}

void RNA_def_node_tree_interface(BlenderRNA *brna)
{
  rna_def_node_interface_item(brna);
  rna_def_node_interface_socket(brna);
  rna_def_node_interface_socket_float(brna);
  rna_def_node_interface_socket_int(brna);
  rna_def_node_interface_socket_bool(brna);
  rna_def_node_interface_socket_string(brna);
  rna_def_node_interface_socket_object(brna);
  rna_def_node_interface_panel(brna);
  rna_def_node_tree_interface(brna);
}

#endif
