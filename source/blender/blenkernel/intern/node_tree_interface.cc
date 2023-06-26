/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_node_tree_interface.hh"

#include "BLI_array.hh"
#include "BLI_stack.hh"
#include "BLI_string.h"
#include "BLI_vector.hh"

#include "DNA_node_tree_interface_types.h"
#include "DNA_node_types.h"

#include <queue>

std::string bNodeTreeInterfaceSocket::socket_identifier() const
{
  return "Socket" + std::to_string(uid);
}

int bNodeTreeInterface::item_index(bNodeTreeInterfaceItem &item) const
{
  return items().first_index_try(&item);
}

void bNodeTreeInterface::add_item(bNodeTreeInterfaceItem &item)
{
  blender::MutableSpan<bNodeTreeInterfaceItem *> old_items = items();
  items_num++;
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);
  items().drop_back(1).copy_from(old_items);
  items().last() = &item;

  if (!old_items.is_empty()) {
    MEM_freeN(old_items.data());
  }
}

void bNodeTreeInterface::insert_item(bNodeTreeInterfaceItem &item, int index)
{
  BLI_assert(blender::IndexRange(items().size() + 1).contains(index));

  blender::MutableSpan<bNodeTreeInterfaceItem *> old_items = items();
  items_num++;
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);
  items().take_front(index).copy_from(old_items.take_front(index));
  items().drop_front(index + 1).copy_from(old_items.drop_front(index));
  items()[index] = &item;

  if (!old_items.is_empty()) {
    MEM_freeN(old_items.data());
  }
}

void bNodeTreeInterface::free_item(bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_PANEL: {
      auto &panel = item.get_as<bNodeTreeInterfacePanel>();
      MEM_SAFE_FREE(panel.name);
      break;
    }
    case NODE_INTERFACE_SOCKET: {
      auto &socket = item.get_as<bNodeTreeInterfaceSocket>();
      MEM_SAFE_FREE(socket.name);
      MEM_SAFE_FREE(socket.description);
      MEM_SAFE_FREE(socket.type);
      break;
    }
  }
}

bNodeTreeInterfaceSocket *bNodeTreeInterface::add_socket(blender::StringRef name,
                                                         blender::StringRef description,
                                                         blender::StringRef type,
                                                         const eNodeTreeInterfaceSocketKind kind)
{
  bNodeTreeInterfaceSocket *new_socket = MEM_cnew<bNodeTreeInterfaceSocket>(__func__);
  new_socket->name = BLI_strdup(name.data());
  new_socket->description = BLI_strdup(description.data());
  new_socket->type = BLI_strdup(type.data());
  new_socket->kind = kind;
  new_socket->uid = next_socket_uid++;

  add_item(new_socket->item);

  return new_socket;
}

bNodeTreeInterfaceSocket *bNodeTreeInterface::insert_socket(
    blender::StringRef name,
    blender::StringRef description,
    blender::StringRef type,
    const eNodeTreeInterfaceSocketKind kind,
    const int index)
{
  if (!blender::IndexRange(items().size() + 1).contains(index)) {
    return nullptr;
  }

  bNodeTreeInterfaceSocket *new_socket = MEM_cnew<bNodeTreeInterfaceSocket>(__func__);
  new_socket->name = BLI_strdup(name.data());
  new_socket->description = BLI_strdup(description.data());
  new_socket->type = BLI_strdup(type.data());
  new_socket->kind = kind;
  new_socket->uid = next_socket_uid++;

  insert_item(new_socket->item, index);

  return new_socket;
}

bNodeTreeInterfacePanel *bNodeTreeInterface::add_panel(blender::StringRef name)
{
  bNodeTreeInterfacePanel *new_panel = MEM_cnew<bNodeTreeInterfacePanel>(__func__);
  new_panel->name = BLI_strdup(name.data());

  add_item(new_panel->item);

  return new_panel;
}

bNodeTreeInterfacePanel *bNodeTreeInterface::insert_panel(blender::StringRef name, const int index)
{
  if (!blender::IndexRange(items().size() + 1).contains(index)) {
    return nullptr;
  }

  bNodeTreeInterfacePanel *new_panel = MEM_cnew<bNodeTreeInterfacePanel>(__func__);
  new_panel->name = BLI_strdup(name.data());

  insert_item(new_panel->item, index);

  return new_panel;
}

bool bNodeTreeInterface::remove_item(bNodeTreeInterfaceItem &item)
{
  const int index = item_index(item);
  if (!items().index_range().contains(index)) {
    return false;
  }

  blender::MutableSpan<bNodeTreeInterfaceItem *> old_items = items();
  items_num--;
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);
  items().take_front(index).copy_from(old_items.take_front(index));
  items().drop_front(index).copy_from(old_items.drop_front(index + 1));

  free_item(item);

  /* Guaranteed not empty, contains at least the removed item */
  MEM_freeN(old_items.data());

  return true;
}

void bNodeTreeInterface::clear_items()
{
  for (bNodeTreeInterfaceItem *item : items()) {
    free_item(*item);
  }

  MEM_SAFE_FREE(items_array);
  items_num = 0;
  items_array = nullptr;
}

bool bNodeTreeInterface::move_item(bNodeTreeInterfaceItem &item, const int new_index)
{
  const int old_index = item_index(item);
  if (!items().index_range().contains(old_index) || !items().index_range().contains(new_index)) {
    return false;
  }

  if (old_index == new_index) {
    /* Nothing changes. */
    return true;
  }
  else if (old_index < new_index) {
    const blender::Span<bNodeTreeInterfaceItem *> moved_items = items().slice(
        old_index + 1, new_index - old_index);
    bNodeTreeInterfaceItem *tmp = items()[old_index];
    std::copy(moved_items.begin(), moved_items.end(), items().drop_front(old_index).data());
    items()[new_index] = tmp;
  }
  else /* old_index > new_index */ {
    const blender::Span<bNodeTreeInterfaceItem *> moved_items = items().slice(
        new_index, old_index - new_index);
    bNodeTreeInterfaceItem *tmp = items()[old_index];
    std::copy_backward(
        moved_items.begin(), moved_items.end(), items().drop_front(old_index + 1).data());
    items()[new_index] = tmp;
  }

  return true;
}

#if 0
struct NodePanelSortState {
  int index;
  bNodePanel *panel;
  /* Forms a singly linked list of siblings of the same parent */
  NodePanelSortState *next_sibling = nullptr;
  /* First child panel */
  NodePanelSortState *first_child = nullptr;
};

void bNodeTreeInterface::update_order()
{
  /* Topological sorting for panels. */

  blender::Vector<bNodePanel *> sorted_panels;
  sorted_panels.reserve(panels().size());

  blender::Array<NodePanelSortState> panel_states(panels().size());
  blender::Stack<NodePanelSortState> ready_nodes;
  for (const int i : panels().index_range()) {
    bNodePanel *panel = panels()[i];
    panel_states[i] = {i, panel};

    //    /* Build sibling lists. */
    //    if (panel->parent) {
    //    }

    /* Initial panels with zero in-degree: all panels at depth 0 */
    if (!panel->parent) {
      ready_nodes.push(panel_states[i]);
    }
  }

  while (!ready_nodes.is_empty()) {
    NodePanelSortState state = ready_nodes.pop();

    sorted_panels.append(state.panel);

    /* Put all child panels before the next sibling. */
    /* XXX children should be cached */
    for (NodePanelSortState &child_state : panel_states) {
      if (child_state.panel->parent != state.panel->parent) {
        continue;
      }

      ready_nodes.push(child_state);
    }
  }

  /* Topological sorting for sockets. */
}
#endif

void bNodeTreeInterface::update_panels_order()
{
  //  /* Topological sorting for panels. */
  //  blender::Vector<bNodePanel *> sorted_panels;
  //  sorted_panels.reserve(panels().size());

  //  std::queue<bNodePanel *> ready_panels;
  //  for (const int i : panels().index_range()) {
  //    bNodePanel *panel = panels()[i];

  //    /* Initial panels with zero in-degree: all panels at depth 0 */
  //    if (!panel->parent) {
  //      ready_panels.emplace(panel);
  //    }
  //  }

  //  while (!ready_panels.empty()) {
  //    bNodePanel *panel = ready_panels.front();
  //    ready_panels.pop();

  //    sorted_panels.append(panel);

  //    /* Put all child panels before the next sibling. */
  //    /* XXX children should be cached:
  //     * - Build a simple linked list temporarily for sorting (O(n)).
  //     * - After sorting direct child panels can be returned as a span.
  //     */
  //    for (bNodePanel *child_panel : panels()) {
  //      if (child_panel->parent != panel->parent) {
  //        continue;
  //      }

  //      ready_panels.push(child_panel);
  //    }
  //  }

  //  /* Copy sorted panels. */
  //  for (const int i : blender::IndexRange(panels_num)) {
  //    panels_array[i] = sorted_panels[i];
  //  }
}

void bNodeTreeInterface::update_sockets_order()
{
  //  /* Topological sorting for sockets. */
  //  /* Warning: Panels must be sorted at this point. */

  //  blender::Vector<bNodeSocketDeclaration *> sorted_sockets;
  //  sorted_sockets.reserve(sockets().size());

  //  for (const int i : panels().index_range()) {
  //    bNodePanel *panel = panels()[i];

  //    /* Sort sockets by order of parents. */
  //    /* XXX children should be cached:
  //     * - Build a simple linked list temporarily for sorting (O(n)).
  //     * - After sorting direct child panels can be returned as a span.
  //     */
  //    for (bNodeSocketDeclaration *socket_decl : sockets()) {
  //      if (socket_decl->panel != panel) {
  //        continue;
  //      }

  //      sorted_sockets.append(socket_decl);
  //    }
  //  }

  //  /* Copy sorted sockets. */
  //  for (const int i : blender::IndexRange(sockets_num)) {
  //    sockets_array[i] = sorted_sockets[i];
  //  }
}

void bNodeTreeInterface::update_index()
{
  //  int index = 0;
  //  for (bNodePanel *panel : panels()) {
  //    panel->index = index++;
  //  }
}

void bNodeTreeInterface::update_order()
{
  update_panels_order();
  update_sockets_order();
}
