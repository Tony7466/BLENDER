/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_node_types.h"

#include "NOD_socket_items.hh"

struct BlendWriter;
struct BlendDataReader;

namespace blender::nodes {

/**
 * Makes it possible to use various functions (e.g. the ones in `NOD_socket_items.hh`) with
 * simulation items.
 */
struct SimulationItemsAccessor {
  using ItemT = NodeSimulationItem;
  static StructRNA *item_srna;
  static int node_type;
  static constexpr const char *node_idname = "GeometryNodeSimulationOutput";

  static socket_items::SocketItemsRef<NodeSimulationItem> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeGeometrySimulationOutput *>(node.storage);
    return {&storage->items, &storage->items_num, &storage->active_index};
  }
  static void copy_item(const NodeSimulationItem &src, NodeSimulationItem &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }
  static void destruct_item(NodeSimulationItem *item)
  {
    MEM_SAFE_FREE(item->name);
  }
  static void blend_write(BlendWriter *writer, const bNode &node);
  static void blend_read_data(BlendDataReader *reader, bNode &node);
  static short *get_socket_type(NodeSimulationItem &item)
  {
    return &item.socket_type;
  }
  static char **get_name(NodeSimulationItem &item)
  {
    return &item.name;
  }
  static bool supports_socket_type(const eNodeSocketDatatype socket_type)
  {
    return ELEM(socket_type,
                SOCK_FLOAT,
                SOCK_VECTOR,
                SOCK_RGBA,
                SOCK_BOOLEAN,
                SOCK_ROTATION,
                SOCK_INT,
                SOCK_STRING,
                SOCK_GEOMETRY);
  }
  static void init_with_socket_type_and_name(bNode &node,
                                             NodeSimulationItem &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeGeometrySimulationOutput *>(node.storage);
    item.socket_type = socket_type;
    item.identifier = storage->next_identifier++;
    socket_items::set_item_name_and_make_unique<SimulationItemsAccessor>(node, item, name);
  }
  static std::string socket_identifier_for_item(const NodeSimulationItem &item)
  {
    return "Item_" + std::to_string(item.identifier);
  }
};

/**
 * Makes it possible to use various functions (e.g. the ones in `NOD_socket_items.hh`) with
 * repeat items.
 */
struct RepeatItemsAccessor {
  using ItemT = NodeRepeatItem;
  static StructRNA *item_srna;
  static int node_type;
  static constexpr const char *node_idname = "GeometryNodeRepeatOutput";

  static socket_items::SocketItemsRef<NodeRepeatItem> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeGeometryRepeatOutput *>(node.storage);
    return {&storage->items, &storage->items_num, &storage->active_index};
  }
  static void copy_item(const NodeRepeatItem &src, NodeRepeatItem &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }
  static void destruct_item(NodeRepeatItem *item)
  {
    MEM_SAFE_FREE(item->name);
  }
  static void blend_write(BlendWriter *writer, const bNode &node);
  static void blend_read_data(BlendDataReader *reader, bNode &node);
  static short *get_socket_type(NodeRepeatItem &item)
  {
    return &item.socket_type;
  }
  static char **get_name(NodeRepeatItem &item)
  {
    return &item.name;
  }
  static bool supports_socket_type(const eNodeSocketDatatype socket_type)
  {
    return ELEM(socket_type,
                SOCK_FLOAT,
                SOCK_VECTOR,
                SOCK_RGBA,
                SOCK_BOOLEAN,
                SOCK_ROTATION,
                SOCK_INT,
                SOCK_STRING,
                SOCK_GEOMETRY,
                SOCK_OBJECT,
                SOCK_MATERIAL,
                SOCK_IMAGE,
                SOCK_COLLECTION);
  }
  static void init_with_socket_type_and_name(bNode &node,
                                             NodeRepeatItem &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeGeometryRepeatOutput *>(node.storage);
    item.socket_type = socket_type;
    item.identifier = storage->next_identifier++;
    socket_items::set_item_name_and_make_unique<RepeatItemsAccessor>(node, item, name);
  }
  static std::string socket_identifier_for_item(const NodeRepeatItem &item)
  {
    return "Item_" + std::to_string(item.identifier);
  }
};

struct ForEachInputItemsAccessor {
  using ItemT = NodeForEachInputItem;
  static StructRNA *item_srna;
  /* This refers to the node that stores the item array. */
  static int node_type;
  static constexpr const char *node_idname = "GeometryNodeForEachOutput";

  static socket_items::SocketItemsRef<NodeForEachInputItem> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeGeometryForEachOutput *>(node.storage);
    return {&storage->input_items, &storage->input_items_num, &storage->input_active_index};
  }
  static void copy_item(const NodeForEachInputItem &src, NodeForEachInputItem &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }
  static void destruct_item(NodeForEachInputItem *item)
  {
    MEM_SAFE_FREE(item->name);
  }
  static void blend_write(BlendWriter *, const bNode &node);
  static void blend_read_data(BlendDataReader *reader, bNode &node);
  static short *get_socket_type(NodeForEachInputItem &item)
  {
    return &item.socket_type;
  }
  static char **get_name(NodeForEachInputItem &item)
  {
    return &item.name;
  }
  static bool supports_socket_type(const eNodeSocketDatatype /*socket_type*/)
  {
    /* TODO */
    return true;
  }
  static void init_with_socket_type_and_name(bNode &node,
                                             NodeForEachInputItem &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeGeometryForEachOutput *>(node.storage);
    item.socket_type = socket_type;
    item.identifier = storage->input_next_identifier++;
    socket_items::set_item_name_and_make_unique<ForEachInputItemsAccessor>(node, item, name);
  }
  static std::string socket_identifier_for_item(const NodeForEachInputItem &item)
  {
    return "Item_" + std::to_string(item.identifier);
  }
};

struct ForEachOutputItemsAccessor {
  using ItemT = NodeForEachOutputItem;
  static StructRNA *item_srna;
  static int node_type;
  static constexpr const char *node_idname = "GeometryNodeForEachOutput";

  static socket_items::SocketItemsRef<NodeForEachOutputItem> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeGeometryForEachOutput *>(node.storage);
    return {&storage->output_items, &storage->output_items_num, &storage->output_active_index};
  }
  static void copy_item(const NodeForEachOutputItem &src, NodeForEachOutputItem &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }
  static void destruct_item(NodeForEachOutputItem *item)
  {
    MEM_SAFE_FREE(item->name);
  }
  static void blend_write(BlendWriter *, const bNode &node);
  static void blend_read_data(BlendDataReader *reader, bNode &node);
  static short *get_socket_type(NodeForEachOutputItem &item)
  {
    return &item.socket_type;
  }
  static char **get_name(NodeForEachOutputItem &item)
  {
    return &item.name;
  }
  static bool supports_socket_type(const eNodeSocketDatatype /*socket_type*/)
  {
    /* TODO */
    return true;
  }
  static void init_with_socket_type_and_name(bNode &node,
                                             NodeForEachOutputItem &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeGeometryForEachOutput *>(node.storage);
    item.socket_type = socket_type;
    item.identifier = storage->output_next_identifier++;
    socket_items::set_item_name_and_make_unique<ForEachOutputItemsAccessor>(node, item, name);
  }
  static std::string socket_identifier_for_item(const NodeForEachOutputItem &item)
  {
    return "Item_" + std::to_string(item.identifier);
  }
};

}  // namespace blender::nodes
