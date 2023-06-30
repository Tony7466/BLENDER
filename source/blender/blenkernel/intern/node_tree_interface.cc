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

const char *bNodeTreeInterfaceSocketFloat::socket_type_static = "NodeSocketFloat";
const char *bNodeTreeInterfaceSocketInt::socket_type_static = "NodeSocketInt";
const char *bNodeTreeInterfaceSocketBool::socket_type_static = "NodeSocketBool";
const char *bNodeTreeInterfaceSocketString::socket_type_static = "NodeSocketString";
const char *bNodeTreeInterfaceSocketObject::socket_type_static = "NodeSocketObject";

namespace {

void copy_socket_base_data(bNodeTreeInterfaceSocket &dst, const bNodeTreeInterfaceSocket &src)
{
  BLI_assert(src.name != nullptr);
  BLI_assert(src.data_type != nullptr);
  dst.name = BLI_strdup(src.name);
  dst.description = src.description ? BLI_strdup(src.description) : nullptr;
  dst.data_type = BLI_strdup(src.data_type);
}

void copy_item_data(bNodeTreeInterfacePanel &dst, const bNodeTreeInterfacePanel &src)
{
  BLI_assert(src.name != nullptr);
  dst.name = BLI_strdup(src.name);
}

void copy_item_data(bNodeTreeInterfaceSocketFloat &dst, const bNodeTreeInterfaceSocketFloat &src)
{
  copy_socket_base_data(dst.base, src.base);
}

void copy_item_data(bNodeTreeInterfaceSocketInt &dst, const bNodeTreeInterfaceSocketInt &src)
{
  copy_socket_base_data(dst.base, src.base);
}

void copy_item_data(bNodeTreeInterfaceSocketBool &dst, const bNodeTreeInterfaceSocketBool &src)
{
  copy_socket_base_data(dst.base, src.base);
}

void copy_item_data(bNodeTreeInterfaceSocketString &dst, const bNodeTreeInterfaceSocketString &src)
{
  copy_socket_base_data(dst.base, src.base);
}

void copy_item_data(bNodeTreeInterfaceSocketObject &dst, const bNodeTreeInterfaceSocketObject &src)
{
  copy_socket_base_data(dst.base, src.base);
}

void free_item(bNodeTreeInterfaceItem &item)
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

}  // namespace

// struct CopyItemOp {
//  using ParentMap = bNodeTreeInterfaceItem::ParentMap;

//  bNodeTreeInterfaceItem &dst;
//  const bNodeTreeInterfaceItem &src;
//  std::optional<ParentMap> parent_map;

//  template<typename ItemT> void operator()()
//  {
//    BLI_assert(src.item_type == dst.item_type);

//    if (parent_map) {
//      if (src.parent != nullptr) {
//        dst.parent = parent_map->lookup(src.parent);
//      }
//      else {
//        dst.parent = nullptr;
//      }
//    }

//    copy_item_data(dst.get_as<ItemT>(), src.get_as<ItemT>());
//  }
//};

void bNodeTreeInterfaceItem::copy_data(const bNodeTreeInterfaceItem & /*src*/)
{
  //  apply_typed_operator(CopyItemOp{*this, src, parent_map});
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
  root_panel.item.copy_data(src.root_panel.item);
  this->active_index = src.active_index;
}

void bNodeTreeInterface::free_data()
{
  clear_items(nullptr);
}

blender::IndexRange bNodeTreeInterfacePanel::items_range() const
{
  return blender::IndexRange(items_num);
}

blender::Span<const bNodeTreeInterfaceItem *> bNodeTreeInterfacePanel::items() const
{
  return blender::Span(items_array, items_num);
}

blender::MutableSpan<bNodeTreeInterfaceItem *> bNodeTreeInterfacePanel::items()
{
  return blender::MutableSpan(items_array, items_num);
}

void bNodeTreeInterfacePanel::add_item(bNodeTreeInterfaceItem &item)
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

void bNodeTreeInterfacePanel::insert_item(bNodeTreeInterfaceItem &item, int index)
{
  index = std::min(std::max(index, 0), items_num);

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

bool bNodeTreeInterfacePanel::remove_item(bNodeTreeInterfaceItem &item, bool free)
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

  /* Guaranteed not empty, contains at least the removed item */
  MEM_freeN(old_items.data());

  if (free) {
    free_item(item);
  }

  return true;
}

void bNodeTreeInterfacePanel::clear_items()
{
  for (bNodeTreeInterfaceItem *item : items()) {
    free_item(*item);
  }
  MEM_SAFE_FREE(items_array);
  items_array = nullptr;
  items_num = 0;
}

bool bNodeTreeInterfacePanel::move_item(bNodeTreeInterfaceItem &item, const int new_index)
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
  new_panel->name = BLI_strdup(name.data());
  return new_panel;
}

int bNodeTreeInterfacePanel::item_index(const bNodeTreeInterfaceItem &item) const
{
  return items().first_index_try(&item);
}

void bNodeTreeInterfacePanel::copy_items(
    const blender::Span<const bNodeTreeInterfaceItem *> items_src)
{
  items_num = items_src.size();
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);

  /* Copy buffers. */
  for (const int i : items_src.index_range()) {
    const bNodeTreeInterfaceItem *item_src = items_src[i];
    items_array[i] = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(item_src));
  }

  /* Copy data and remap pointers. */
  for (const int i : items_src.index_range()) {
    const bNodeTreeInterfaceItem &item_src = *items_src[i];
    bNodeTreeInterfaceItem &item_dst = *items()[i];

    item_dst.copy_data(item_src);
  }
}

bNodeTreeInterfaceSocket *bNodeTreeInterface::add_socket(blender::StringRef name,
                                                         blender::StringRef description,
                                                         blender::StringRef data_type,
                                                         const eNodeTreeInterfaceSocketKind kind,
                                                         bNodeTreeInterfacePanel *parent)
{
  bNodeTreeInterfaceSocket *new_socket = make_socket(
      next_socket_uid++, name, description, data_type, kind);
  parent->add_item(new_socket->item);
  return new_socket;
}

bNodeTreeInterfaceSocket *bNodeTreeInterface::insert_socket(
    blender::StringRef name,
    blender::StringRef description,
    blender::StringRef data_type,
    const eNodeTreeInterfaceSocketKind kind,
    bNodeTreeInterfacePanel *parent,
    const int index)
{
  bNodeTreeInterfaceSocket *new_socket = make_socket(
      next_socket_uid++, name, description, data_type, kind);
  parent->insert_item(new_socket->item, index);
  return new_socket;
}

bNodeTreeInterfacePanel *bNodeTreeInterface::add_panel(blender::StringRef name,
                                                       bNodeTreeInterfacePanel *parent)
{
  bNodeTreeInterfacePanel *new_panel = make_panel(name);
  parent->add_item(new_panel->item);
  return new_panel;
}

bNodeTreeInterfacePanel *bNodeTreeInterface::insert_panel(blender::StringRef name,
                                                          bNodeTreeInterfacePanel *parent,
                                                          const int index)
{
  bNodeTreeInterfacePanel *new_panel = make_panel(name);
  parent->insert_item(new_panel->item, index);
  return new_panel;
}

bNodeTreeInterfaceItem *bNodeTreeInterface::add_item_copy(const bNodeTreeInterfaceItem &item,
                                                          bNodeTreeInterfacePanel *parent)
{
  BLI_assert(contains_item(item));
  BLI_assert(parent == nullptr || contains_item(*parent));

  if (parent == nullptr) {
    parent = &root_panel;
  }

  bNodeTreeInterfaceItem *citem = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(&item));
  citem->copy_data(item);
  parent->add_item(*citem);

  return citem;
}

bNodeTreeInterfaceItem *bNodeTreeInterface::insert_item_copy(const bNodeTreeInterfaceItem &item,
                                                             bNodeTreeInterfacePanel *parent,
                                                             int index)
{
  BLI_assert(contains_item(item));
  BLI_assert(parent == nullptr || contains_item(*parent));

  if (parent == nullptr) {
    parent = &root_panel;
  }

  bNodeTreeInterfaceItem *citem = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(&item));
  citem->copy_data(item);
  parent->insert_item(*citem, index);

  return citem;
}

bool bNodeTreeInterface::remove_item(bNodeTreeInterfaceItem &item, bNodeTreeInterfacePanel *parent)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  if (parent->remove_item(item, true)) {
    return true;
  }
  return false;
}

void bNodeTreeInterface::clear_items(bNodeTreeInterfacePanel *parent)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  parent->clear_items();
}

bool bNodeTreeInterface::move_item(bNodeTreeInterfaceItem &item,
                                   bNodeTreeInterfacePanel *parent,
                                   const int new_index)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  return parent->move_item(item, new_index);
}

bool bNodeTreeInterface::move_item_to_parent(bNodeTreeInterfaceItem &item,
                                             bNodeTreeInterfacePanel *parent,
                                             bNodeTreeInterfacePanel *new_parent,
                                             int new_index)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  if (parent->remove_item(item, false)) {
    new_parent->insert_item(item, new_index);
    return true;
  }
  return false;
}

bool bNodeTreeInterface::contains_item(const bNodeTreeInterfaceItem &item) const
{
  bool result = false;
  foreach_item([item, &result](const bNodeTreeInterfaceItem &titem) {
    if (&titem == &item) {
      result = true;
      return false;
    }
    return true;
  });
  return result;
}

// void bNodeTreeInterface::update_order()
//{
//  /* Reset counters. */
//  root_items_num = 0;
//  for (bNodeTreeInterfaceItem *item : items()) {
//    /* index >= 0 is also used to mark sorted hierarchy levels. */
//    item->index = -1;
//    item->children_start = 0;
//    item->children_num = 0;
//  }

//  bNodeTreeInterfaceItem **sorted_items_array = MEM_cnew_array<bNodeTreeInterfaceItem
//  *>(items_num,
//                                                                                         __func__);
//  int sorted_i = 0;
//  while (sorted_i < items_num) {
//    const int prev_sorted_i = sorted_i;
//    /* Offset indicates the next writing position for remaining inputs. */
//    int input_offset = 0;
//    /* Only the first N items still need sorting, input array is compressed in each pass. */
//    for (const int input_i : items().index_range().drop_back(sorted_i)) {
//      bNodeTreeInterfaceItem *item = items()[input_i];

//      /* Item \a index variable is used to indicate
//       * when all items of the same hierarchy level are sorted. */
//      const bool item_is_ready = (item->parent == nullptr || item->parent->item.index >= 0);

//      if (item_is_ready) {
//        /* Insert item into the sorted output array. */
//        sorted_items_array[sorted_i] = item;

//        /* Update child info of the parent. */
//        if (item->parent) {
//          /* First child sets the parent's start index. */
//          if (item->parent->item.children_num == 0) {
//            item->parent->item.children_start = sorted_i;
//          }
//          ++item->parent->item.children_num;
//        }
//        else {
//          ++root_items_num;
//        }

//        ++sorted_i;
//        ++input_offset;
//      }
//      else {
//        /* Compress input by reinserting remaining items in the gaps left by sorted items. */
//        if (input_offset > 0) {
//          const int write_i = input_i - input_offset;
//          items_array[write_i] = items_array[input_i];
//        }
//      }
//    }
//    BLI_assert(prev_sorted_i < sorted_i);

//    /* Set index for newly sorted items.
//     * This also marks their children ready for sorting in the next pass. */
//    for (const int i : items().index_range().slice(prev_sorted_i, sorted_i - prev_sorted_i)) {
//      sorted_items_array[i]->index = i;
//    }
//  }

//  MEM_SAFE_FREE(items_array);
//  items_array = sorted_items_array;
//}

namespace blender::bke {

void bNodeTreeInterfaceCache::rebuild(bNodeTreeInterface &interface)
{
  /* Rebuild draw-order list of interface items for linear access. */
  items.clear();

  /* DFS over all items */
  Stack<bNodeTreeInterfacePanel *> parent_stack;
  parent_stack.push(&interface.root_panel);
  while (!parent_stack.is_empty()) {
    bNodeTreeInterfacePanel *parent = parent_stack.pop();
    for (bNodeTreeInterfaceItem *item : parent->items()) {
      items.append(item);
      if (bNodeTreeInterfacePanel *panel = item->get_as_ptr<bNodeTreeInterfacePanel>()) {
        parent_stack.push(panel);
      }
    }
  }
}

}  // namespace blender::bke

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
      BLO_write_pointer_array(writer, panel->items_num, panel->items_array);
      for (bNodeTreeInterfaceItem *item : panel->items()) {
        interface_item_write(writer, item);
      }
      break;
    }
  }
}

static void interface_item_read_data(BlendDataReader *reader, bNodeTreeInterfaceItem *item)
{
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
      BLO_read_pointer_array(reader, reinterpret_cast<void **>(&panel->items_array));
      for (const int i : blender::IndexRange(panel->items_num)) {
        BLO_read_data_address(reader, &panel->items_array[i]);
        interface_item_read_data(reader, panel->items_array[i]);
      }
      break;
    }
  }
}

static void interface_item_read_lib(BlendLibReader *reader, ID *id, bNodeTreeInterfaceItem *item)
{
  switch (item->item_type) {
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel *panel = reinterpret_cast<bNodeTreeInterfacePanel *>(item);
      for (bNodeTreeInterfaceItem *item : panel->items()) {
        interface_item_read_lib(reader, id, item);
      }
      break;
    }
  }
}

static void interface_item_read_expand(BlendExpander *expander, bNodeTreeInterfaceItem *item)
{
  switch (item->item_type) {
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel *panel = reinterpret_cast<bNodeTreeInterfacePanel *>(item);
      for (bNodeTreeInterfaceItem *item : panel->items()) {
        interface_item_read_expand(expander, item);
      }
      break;
    }
  }
}

void BKE_nodetree_interface_write(BlendWriter *writer, bNodeTreeInterface *interface)
{
  BLO_write_struct(writer, bNodeTreeInterface, interface);
  interface_item_write(writer, &interface->root_panel.item);
}

void BKE_nodetree_interface_read_data(BlendDataReader *reader, bNodeTreeInterface *interface)
{
  interface_item_read_data(reader, &interface->root_panel.item);
}

void BKE_nodetree_interface_read_lib(BlendLibReader *reader, ID *id, bNodeTreeInterface *interface)
{
  interface_item_read_lib(reader, id, &interface->root_panel.item);
}

void BKE_nodetree_interface_read_expand(BlendExpander *expander, bNodeTreeInterface *interface)
{
  interface_item_read_expand(expander, &interface->root_panel.item);
}
