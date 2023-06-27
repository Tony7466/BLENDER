/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_node.h"
#include "BKE_node_tree_interface.hh"

#include "BLI_array.hh"
#include "BLI_stack.hh"
#include "BLI_string.h"
#include "BLI_vector.hh"

#include "BLO_read_write.h"

#include "DNA_node_tree_interface_types.h"
#include "DNA_node_types.h"

#include "RNA_access.h"
#include "RNA_prototypes.h"

#include <queue>

void bNodeTreeInterfaceItem::copy_data(const bNodeTreeInterfaceItem &src,
                                       std::optional<ParentMap> parent_map)
{
  BLI_assert(src.item_type == item_type);

  if (parent_map) {
    if (src.parent != nullptr) {
      parent = parent_map->lookup(src.parent);
    }
    else {
      parent = nullptr;
    }
  }

  switch (item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(*this);
      const bNodeTreeInterfaceSocket &socket_src =
          reinterpret_cast<const bNodeTreeInterfaceSocket &>(src);
      BLI_assert(socket_src.name != nullptr);
      BLI_assert(socket_src.data_type != nullptr);
      socket.name = BLI_strdup(socket_src.name);
      socket.description = socket_src.description ? BLI_strdup(socket_src.description) : nullptr;
      socket.data_type = BLI_strdup(socket_src.data_type);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(*this);
      const bNodeTreeInterfacePanel &panel_src = reinterpret_cast<const bNodeTreeInterfacePanel &>(
          src);
      BLI_assert(panel.name != nullptr);
      panel.name = BLI_strdup(panel_src.name);
      break;
    }
  }
}

void bNodeTreeInterfaceItem::free_data() {}

std::string bNodeTreeInterfaceSocket::socket_identifier() const
{
  return "Socket" + std::to_string(uid);
}

blender::ColorGeometry4f bNodeTreeInterfaceSocket::socket_color() const
{
  bNodeSocketType *typeinfo = nodeSocketTypeFind(data_type);
  if (typeinfo && typeinfo->draw_color) {
    /* XXX Warning! Color callbacks for sockets are overly general,
     * using instance pointers for potential context-based colors.
     * This is not used by the standard socket types, but might be used
     * by python nodes.
     *
     * There should be a simplified "base color" callback that does not
     * depend on context, for drawing socket type colors without any
     * actual socket instance. We just set pointers to null here and
     * hope for the best until the situation can be improved ...
     */
    //    bNode dummy_node{0};
    bNodeSocket dummy_socket{0};
    dummy_socket.typeinfo = typeinfo;
    PointerRNA socket_ptr, node_ptr;
    //    RNA_pointer_create(nullptr, &RNA_Node, &dummy_node, &node_ptr);
    node_ptr = PointerRNA_NULL;
    RNA_pointer_create(nullptr, &RNA_NodeSocket, &dummy_socket, &socket_ptr);
    bContext *C = nullptr;
    float color[4];
    typeinfo->draw_color(C, &socket_ptr, &node_ptr, color);
    return blender::ColorGeometry4f(color);
  }
  else {
    return blender::ColorGeometry4f(1.0f, 0.0f, 1.0f, 1.0f);
  }
}

void bNodeTreeInterface::copy_data(const bNodeTreeInterface &src)
{
  copy_items(src.items());
  this->active_index = src.active_index;
}

void bNodeTreeInterface::free_data()
{
  clear_items();
}

int bNodeTreeInterface::item_index(bNodeTreeInterfaceItem &item) const
{
  return items().first_index_try(&item);
}

blender::IndexRange bNodeTreeInterface::root_items_range() const
{
  return blender::IndexRange(root_items_num);
}

blender::Span<const bNodeTreeInterfaceItem *> bNodeTreeInterface::root_items() const
{
  return items().take_front(root_items_num);
}

blender::MutableSpan<bNodeTreeInterfaceItem *> bNodeTreeInterface::root_items()
{
  return items().take_front(root_items_num);
}

blender::IndexRange bNodeTreeInterface::item_children_range(
    const bNodeTreeInterfaceItem &item) const
{
  return blender::IndexRange(item.children_start, item.children_num);
}

blender::Span<const bNodeTreeInterfaceItem *> bNodeTreeInterface::item_children(
    const bNodeTreeInterfaceItem &item) const
{
  return items().slice(item.children_start, item.children_num);
}

blender::MutableSpan<bNodeTreeInterfaceItem *> bNodeTreeInterface::item_children(
    const bNodeTreeInterfaceItem &item)
{
  return items().slice(item.children_start, item.children_num);
}

bool bNodeTreeInterface::is_valid_parent(const bNodeTreeInterfaceItem &item,
                                         const bNodeTreeInterfacePanel *new_parent) const
{
  const bNodeTreeInterfacePanel *panel = new_parent;
  while (panel) {
    if (&panel->item == &item) {
      return false;
    }
    panel = panel->item.parent;
  }
  return true;
}

bool bNodeTreeInterface::parent_set(bNodeTreeInterfaceItem &item,
                                    bNodeTreeInterfacePanel *new_parent)
{
  if (!is_valid_parent(item, new_parent)) {
    return false;
  }
  item.parent = new_parent;
  update_order();
  return true;
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

bool bNodeTreeInterface::unlink_item(bNodeTreeInterfaceItem &item)
{
  bool changed = false;

  const bNodeTreeInterfacePanel *panel = nullptr;
  if (item.item_type == NODE_INTERFACE_PANEL) {
    panel = reinterpret_cast<bNodeTreeInterfacePanel *>(&item);
  }

  for (bNodeTreeInterfaceItem *titem : items()) {
    if (panel && titem->parent == panel) {
      titem->parent = nullptr;
      changed = true;
    }
  }

  return changed;
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
      MEM_SAFE_FREE(socket.data_type);
      break;
    }
  }

  MEM_freeN(&item);
}

static bNodeTreeInterfaceSocket *make_socket(const int uid,
                                             blender::StringRef name,
                                             blender::StringRef description,
                                             blender::StringRef data_type,
                                             const eNodeTreeInterfaceSocketKind kind)
{
  BLI_assert(name.data() != nullptr);
  BLI_assert(data_type.data() != nullptr);

  bNodeTreeInterfaceSocket *new_socket = MEM_cnew<bNodeTreeInterfaceSocket>(__func__);
  new_socket->uid = uid;
  new_socket->item.item_type = NODE_INTERFACE_SOCKET;
  new_socket->item.parent = nullptr;
  new_socket->name = BLI_strdup(name.data());
  new_socket->description = description.data() ? BLI_strdup(description.data()) : nullptr;
  new_socket->data_type = BLI_strdup(data_type.data());
  new_socket->kind = kind;
  return new_socket;
}

static bNodeTreeInterfacePanel *make_panel(blender::StringRef name)
{
  BLI_assert(name.data() != nullptr);

  bNodeTreeInterfacePanel *new_panel = MEM_cnew<bNodeTreeInterfacePanel>(__func__);
  new_panel->item.item_type = NODE_INTERFACE_PANEL;
  new_panel->item.parent = nullptr;
  new_panel->name = BLI_strdup(name.data());
  return new_panel;
}

bNodeTreeInterfaceSocket *bNodeTreeInterface::add_socket(blender::StringRef name,
                                                         blender::StringRef description,
                                                         blender::StringRef data_type,
                                                         const eNodeTreeInterfaceSocketKind kind)
{
  bNodeTreeInterfaceSocket *new_socket = make_socket(
      next_socket_uid++, name, description, data_type, kind);
  add_item(new_socket->item);

  update_order();
  return new_socket;
}

bNodeTreeInterfaceSocket *bNodeTreeInterface::insert_socket(
    blender::StringRef name,
    blender::StringRef description,
    blender::StringRef data_type,
    const eNodeTreeInterfaceSocketKind kind,
    const int index)
{
  if (!blender::IndexRange(items().size() + 1).contains(index)) {
    return nullptr;
  }

  bNodeTreeInterfaceSocket *new_socket = make_socket(
      next_socket_uid++, name, description, data_type, kind);
  insert_item(new_socket->item, index);

  update_order();
  return new_socket;
}

bNodeTreeInterfacePanel *bNodeTreeInterface::add_panel(blender::StringRef name)
{
  bNodeTreeInterfacePanel *new_panel = make_panel(name);
  add_item(new_panel->item);

  update_order();
  return new_panel;
}

bNodeTreeInterfacePanel *bNodeTreeInterface::insert_panel(blender::StringRef name, const int index)
{
  if (!blender::IndexRange(items().size() + 1).contains(index)) {
    return nullptr;
  }

  bNodeTreeInterfacePanel *new_panel = make_panel(name);
  insert_item(new_panel->item, index);

  update_order();
  return new_panel;
}

bNodeTreeInterfaceItem *bNodeTreeInterface::add_item_copy(const bNodeTreeInterfaceItem &item)
{
  BLI_assert(items().as_span().contains(const_cast<bNodeTreeInterfaceItem *>(&item)));

  bNodeTreeInterfaceItem *citem = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(&item));
  citem->copy_data(item, {});
  add_item(*citem);

  update_order();
  return citem;
}

bNodeTreeInterfaceItem *bNodeTreeInterface::insert_item_copy(const bNodeTreeInterfaceItem &item,
                                                             int index)
{
  BLI_assert(items().as_span().contains(const_cast<bNodeTreeInterfaceItem *>(&item)));

  if (!blender::IndexRange(items().size() + 1).contains(index)) {
    return nullptr;
  }

  bNodeTreeInterfaceItem *citem = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(&item));
  citem->copy_data(item, {});
  insert_item(*citem, index);

  update_order();
  return citem;
}

bool bNodeTreeInterface::remove_item(bNodeTreeInterfaceItem &item)
{
  const int index = item_index(item);
  if (!items().index_range().contains(index)) {
    return false;
  }

  unlink_item(item);

  blender::MutableSpan<bNodeTreeInterfaceItem *> old_items = items();
  items_num--;
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);
  items().take_front(index).copy_from(old_items.take_front(index));
  items().drop_front(index).copy_from(old_items.drop_front(index + 1));

  unlink_item(item);
  free_item(item);

  /* Guaranteed not empty, contains at least the removed item */
  MEM_freeN(old_items.data());

  update_order();
  return true;
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

  update_order();
  return true;
}

void bNodeTreeInterface::copy_items(const blender::Span<const bNodeTreeInterfaceItem *> items_src)
{
  bNodeTreeInterfaceItem::ParentMap parent_map;

  items_num = items_src.size();
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);

  /* Copy buffers and build pointer map. */
  for (const int i : items_src.index_range()) {
    const bNodeTreeInterfaceItem *item_src = items_src[i];
    bNodeTreeInterfaceItem *item_dst = items_array[i] = static_cast<bNodeTreeInterfaceItem *>(
        MEM_dupallocN(item_src));

    if (item_src->item_type == NODE_INTERFACE_PANEL) {
      const bNodeTreeInterfacePanel *panel_src = reinterpret_cast<const bNodeTreeInterfacePanel *>(
          item_src);
      bNodeTreeInterfacePanel *panel_dst = reinterpret_cast<bNodeTreeInterfacePanel *>(item_dst);
      parent_map.add(panel_src, panel_dst);
    }
  }

  /* Copy data and remap pointers. */
  for (const int i : items_src.index_range()) {
    const bNodeTreeInterfaceItem &item_src = *items_src[i];
    bNodeTreeInterfaceItem &item_dst = *items()[i];

    item_dst.copy_data(item_src, parent_map);
  }
}

void bNodeTreeInterface::clear_items()
{
  for (bNodeTreeInterfaceItem *item : items()) {
    /* No need to unlink from other items, everything is freed here. */
    free_item(*item);
  }

  MEM_SAFE_FREE(items_array);
  items_num = 0;
  items_array = nullptr;
}

void bNodeTreeInterface::update_order()
{
  /* Reset counters. */
  root_items_num = 0;
  for (bNodeTreeInterfaceItem *item : items()) {
    /* index >= 0 is also used to mark sorted hierarchy levels. */
    item->index = -1;
    item->children_start = 0;
    item->children_num = 0;
  }

  bNodeTreeInterfaceItem **sorted_items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num,
                                                                                         __func__);
  int sorted_i = 0;
  while (sorted_i < items_num) {
    const int prev_sorted_i = sorted_i;
    /* Offset indicates the next writing position for remaining inputs. */
    int input_offset = 0;
    /* Only the first N items still need sorting, input array is compressed in each pass. */
    for (const int input_i : items().index_range().drop_back(sorted_i)) {
      bNodeTreeInterfaceItem *item = items()[input_i];

      /* Item \a index variable is used to indicate
       * when all items of the same hierarchy level are sorted. */
      const bool item_is_ready = (item->parent == nullptr || item->parent->item.index >= 0);

      if (item_is_ready) {
        /* Insert item into the sorted output array. */
        sorted_items_array[sorted_i] = item;

        /* Update child info of the parent. */
        if (item->parent) {
          /* First child sets the parent's start index. */
          if (item->parent->item.children_num == 0) {
            item->parent->item.children_start = sorted_i;
          }
          ++item->parent->item.children_num;
        }
        else {
          ++root_items_num;
        }

        ++sorted_i;
        ++input_offset;
      }
      else {
        /* Compress input by reinserting remaining items in the gaps left by sorted items. */
        if (input_offset > 0) {
          const int write_i = input_i - input_offset;
          items_array[write_i] = items_array[input_i];
        }
      }
    }
    BLI_assert(prev_sorted_i < sorted_i);

    /* Set index for newly sorted items.
     * This also marks their children ready for sorting in the next pass. */
    for (const int i : items().index_range().slice(prev_sorted_i, sorted_i - prev_sorted_i)) {
      sorted_items_array[i]->index = i;
    }
  }

  MEM_SAFE_FREE(items_array);
  items_array = sorted_items_array;
}

void BKE_nodetree_interface_init(bNodeTreeInterface * /*interface*/) {}

void BKE_nodetree_interface_copy(bNodeTreeInterface *interface_dst,
                                 const bNodeTreeInterface *interface_src)
{
  interface_dst->copy_data(*interface_src);
}

void BKE_nodetree_interface_free(bNodeTreeInterface *interface)
{
  interface->free_data();
}

static void interface_item_write(BlendWriter *writer, bNodeTreeInterfaceItem *item)
{
  switch (item->item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket *socket = reinterpret_cast<bNodeTreeInterfaceSocket *>(item);
      BLO_write_struct(writer, bNodeTreeInterfaceSocket, socket);
      BLO_write_string(writer, socket->name);
      BLO_write_string(writer, socket->description);
      BLO_write_string(writer, socket->data_type);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel *panel = reinterpret_cast<bNodeTreeInterfacePanel *>(item);
      BLO_write_struct(writer, bNodeTreeInterfacePanel, panel);
      BLO_write_string(writer, panel->name);
      break;
    }
  }
}

static void interface_item_read_data(BlendDataReader *reader, bNodeTreeInterfaceItem *item)
{
  BLO_read_data_address(reader, &item->parent);

  switch (item->item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket *socket = reinterpret_cast<bNodeTreeInterfaceSocket *>(item);
      BLO_read_data_address(reader, &socket->name);
      BLO_read_data_address(reader, &socket->description);
      BLO_read_data_address(reader, &socket->data_type);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel *panel = reinterpret_cast<bNodeTreeInterfacePanel *>(item);
      BLO_read_data_address(reader, &panel->name);
      break;
    }
  }
}

static void interface_item_read_lib(BlendLibReader * /*reader*/,
                                    ID * /*id*/,
                                    bNodeTreeInterfaceItem * /*item*/)
{
}

static void interface_item_read_expand(BlendExpander * /*reader*/, bNodeTreeInterfaceItem *
                                       /*item*/)
{
}

void BKE_nodetree_interface_write(BlendWriter *writer, bNodeTreeInterface *interface)
{
  BLO_write_struct(writer, bNodeTreeInterface, interface);

  BLO_write_pointer_array(writer, interface->items_num, interface->items_array);
  for (bNodeTreeInterfaceItem *item : interface->items()) {
    interface_item_write(writer, item);
  }
}

void BKE_nodetree_interface_read_data(BlendDataReader *reader, bNodeTreeInterface *interface)
{
  BLO_read_pointer_array(reader, reinterpret_cast<void **>(&interface->items_array));

  for (const int i : blender::IndexRange(interface->items_num)) {
    BLO_read_data_address(reader, &interface->items_array[i]);
    interface_item_read_data(reader, interface->items_array[i]);
  }
}

void BKE_nodetree_interface_read_lib(BlendLibReader *reader, ID *id, bNodeTreeInterface *interface)
{
  for (bNodeTreeInterfaceItem *item : interface->items()) {
    interface_item_read_lib(reader, id, item);
  }
}

void BKE_nodetree_interface_read_expand(BlendExpander *expander, bNodeTreeInterface *interface)
{
  for (bNodeTreeInterfaceItem *item : interface->items()) {
    interface_item_read_expand(expander, item);
  }
}
