/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_node_types.h"

#include "NOD_item_arrays.hh"

namespace blender::nodes {

struct SimulationItemsAccessor {
  using ItemT = NodeSimulationItem;
  static StructRNA *srna;
  static int node_type;
  static constexpr const char *node_idname = "GeometryNodeSimulationOutput";

  static item_arrays::ItemArrayRef<NodeSimulationItem> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeGeometrySimulationOutput *>(node.storage);
    return {&storage->items, &storage->items_num, &storage->active_index};
  }
  static void destruct_item(NodeSimulationItem *item)
  {
    MEM_SAFE_FREE(item->name);
  }
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
    item_arrays::set_item_name<SimulationItemsAccessor>(node, item, name);
  }
};

struct RepeatItemsAccessor {
  using ItemT = NodeRepeatItem;
  static StructRNA *srna;
  static int node_type;
  static constexpr const char *node_idname = "GeometryNodeRepeatOutput";

  static item_arrays::ItemArrayRef<NodeRepeatItem> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeGeometryRepeatOutput *>(node.storage);
    return {&storage->items, &storage->items_num, &storage->active_index};
  }
  static void destruct_item(NodeRepeatItem *item)
  {
    MEM_SAFE_FREE(item->name);
  }
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
    item_arrays::set_item_name<RepeatItemsAccessor>(node, item, name);
  }
};

}  // namespace blender::nodes
