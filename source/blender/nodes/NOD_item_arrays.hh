/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_node_types.h"

namespace blender::nodes::item_arrays {

template<typename T> struct ItemArrayRef {
  T **items_p;
  int *items_num_p;
  int *active_index_p = nullptr;
};

struct SimulationItemsAccessors {
  using ItemT = NodeSimulationItem;
  static StructRNA *srna;
  static int node_type;
  static constexpr const char *node_idname = "GeometryNodeSimulationOutput";

  static ItemArrayRef<NodeSimulationItem> get_items_from_node(bNode &node)
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
};

struct RepeatItemsAccessors {
  using ItemT = NodeRepeatItem;
  static StructRNA *srna;
  static int node_type;
  static constexpr const char *node_idname = "GeometryNodeRepeatOutput";

  static ItemArrayRef<NodeRepeatItem> get_items_from_node(bNode &node)
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
};

template<typename Accessor>
inline bNode *find_node_by_item(bNodeTree &ntree, const typename Accessor::ItemT &item)
{
  ntree.ensure_topology_cache();
  for (bNode *node : ntree.nodes_by_type(Accessor::node_idname)) {
    ItemArrayRef array = Accessor::get_items_from_node(*node);
    if (&item >= *array.items_p && &item < *array.items_p + *array.items_num_p) {
      return node;
    }
  }
  return nullptr;
}

template<typename T>
inline void remove_item(T **items,
                        int *items_num,
                        int *active_index,
                        const int remove_index,
                        void (*destruct_item)(T *))
{
  static_assert(std::is_trivial_v<T>);
  BLI_assert(remove_index >= 0);
  BLI_assert(remove_index < *items_num);

  const int old_items_num = *items_num;
  const int new_items_num = old_items_num - 1;
  const int old_active_index = *active_index;

  T *old_items = *items;
  T *new_items = MEM_cnew_array<T>(new_items_num, __func__);

  std::copy_n(old_items, remove_index, new_items);
  std::copy_n(
      old_items + remove_index + 1, old_items_num - remove_index - 1, new_items + remove_index);

  destruct_item(&old_items[remove_index]);
  MEM_SAFE_FREE(old_items);

  const int new_active_index = std::max(
      0, old_active_index == new_items_num ? new_items_num - 1 : old_active_index);

  *items = new_items;
  *items_num = new_items_num;
  *active_index = new_active_index;
}

template<typename T>
inline void clear_items(T **items, int *items_num, int *active_index, void (*destruct_item)(T *))
{
  static_assert(std::is_trivial_v<T>);
  for (const int i : blender::IndexRange(*items_num)) {
    destruct_item(&(*items)[i]);
  }
  MEM_SAFE_FREE(*items);
  *items_num = 0;
  *active_index = 0;
}

template<typename T>
inline void move_item(T *items, const int items_num, const int from_index, const int to_index)
{
  static_assert(std::is_trivial_v<T>);
  BLI_assert(from_index >= 0);
  BLI_assert(from_index < items_num);
  BLI_assert(to_index >= 0);
  BLI_assert(to_index < items_num);
  UNUSED_VARS_NDEBUG(items_num);

  if (from_index == to_index) {
    return;
  }

  if (from_index < to_index) {
    const T tmp = items[from_index];
    for (int i = from_index; i < to_index; i++) {
      items[i] = items[i + 1];
    }
    items[to_index] = tmp;
  }
  else if (from_index > to_index) {
    const T tmp = items[from_index];
    for (int i = from_index; i > to_index; i--) {
      items[i] = items[i - 1];
    }
    items[to_index] = tmp;
  }
}

}  // namespace blender::nodes::item_arrays
