/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_node.h"
#include "BKE_node_tree_interface.hh"

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

struct CopyItemOperator {
  bNodeTreeInterfaceItem &dst;
  const bNodeTreeInterfaceItem &src;

  template<typename T> void operator()() const
  {
    reinterpret_cast<T &>(dst).copy_impl(reinterpret_cast<const T &>(src));
  }

  void operator()() const
  {
    dst.copy_impl(src);
  }
};

void copy_item(bNodeTreeInterfaceItem &dst, const bNodeTreeInterfaceItem &src)
{
  dst.to_static_type<blender::bke::node_interface::AllItemTypes>(CopyItemOperator{dst, src});
}

struct ItemFreeOperator {
  bNodeTreeInterfaceItem &item;

  template<typename T> void operator()() const
  {
    reinterpret_cast<T &>(item).free_impl();
    MEM_freeN(&item);
  }

  void operator()() const
  {
    item.free_impl();
    MEM_freeN(&item);
  }
};

void free_item(bNodeTreeInterfaceItem &item)
{
  item.to_static_type<blender::bke::node_interface::AllItemTypes>(ItemFreeOperator{item});
}

}  // namespace

void bNodeTreeInterfaceItem::copy_impl(const bNodeTreeInterfaceItem & /*src*/) {}

void bNodeTreeInterfaceItem::free_impl() {}

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

void bNodeTreeInterfaceSocket::copy_impl(const bNodeTreeInterfaceSocket &src)
{
  item.copy_impl(src.item);

  BLI_assert(src.name != nullptr);
  BLI_assert(src.data_type != nullptr);
  name = BLI_strdup(src.name);
  description = src.description ? BLI_strdup(src.description) : nullptr;
  data_type = BLI_strdup(src.data_type);
}
void bNodeTreeInterfaceSocket::free_impl()
{
  MEM_SAFE_FREE(name);
  MEM_SAFE_FREE(description);
  MEM_SAFE_FREE(data_type);

  item.free_impl();
}

void bNodeTreeInterfaceSocketFloat::copy_impl(const bNodeTreeInterfaceSocketFloat & /*src*/) {}

void bNodeTreeInterfaceSocketFloat::free_impl() {}

void bNodeTreeInterfaceSocketInt::copy_impl(const bNodeTreeInterfaceSocketInt & /*src*/) {}

void bNodeTreeInterfaceSocketInt::free_impl() {}

void bNodeTreeInterfaceSocketBool::copy_impl(const bNodeTreeInterfaceSocketBool & /*src*/) {}

void bNodeTreeInterfaceSocketBool::free_impl() {}

void bNodeTreeInterfaceSocketString::copy_impl(const bNodeTreeInterfaceSocketString & /*src*/) {}

void bNodeTreeInterfaceSocketString::free_impl() {}

void bNodeTreeInterfaceSocketObject::copy_impl(const bNodeTreeInterfaceSocketObject & /*src*/) {}

void bNodeTreeInterfaceSocketObject::free_impl() {}

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

int bNodeTreeInterfacePanel::item_index(const bNodeTreeInterfaceItem &item) const
{
  return items().first_index_try(&item);
}

bool bNodeTreeInterfacePanel::contains_item(const bNodeTreeInterfaceItem &item) const
{
  return items().contains(&item);
}

bool bNodeTreeInterfacePanel::find_item(const bNodeTreeInterfaceItem &item) const
{
  bool is_child = false;
  /* Have to capture item address here instead of just a reference,
   * otherwise pointer comparison will not work. */
  foreach_item([&item, &is_child](const bNodeTreeInterfaceItem &titem) {
    if (&titem == &item) {
      is_child = true;
      return false;
    }
    return true;
  });
  return is_child;
}

bool bNodeTreeInterfacePanel::find_item_parent(const bNodeTreeInterfaceItem &item,
                                               bNodeTreeInterfacePanel *&r_parent)
{
  std::queue<bNodeTreeInterfacePanel *> queue;

  if (contains_item(item)) {
    r_parent = this;
    return true;
  }
  queue.push(this);

  while (!queue.empty()) {
    bNodeTreeInterfacePanel *parent = queue.front();
    queue.pop();

    for (bNodeTreeInterfaceItem *titem : parent->items()) {
      if (titem->item_type != NODE_INTERFACE_PANEL) {
        continue;
      }

      bNodeTreeInterfacePanel *tpanel = titem->get_as_ptr<bNodeTreeInterfacePanel>();
      if (tpanel->contains_item(*titem)) {
        r_parent = tpanel;
        return true;
      }
      queue.push(tpanel);
    }
  }

  r_parent = nullptr;
  return false;
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

void bNodeTreeInterfacePanel::copy_impl(const bNodeTreeInterfacePanel &src)
{
  item.copy_impl(src.item);

  BLI_assert(src.name != nullptr);
  name = BLI_strdup(src.name);

  copy_items(src.items());
}

void bNodeTreeInterfacePanel::free_impl()
{
  MEM_SAFE_FREE(name);
  clear_items();

  item.free_impl();
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

void bNodeTreeInterfacePanel::copy_items(
    const blender::Span<const bNodeTreeInterfaceItem *> items_src)
{
  MEM_SAFE_FREE(items_array);

  items_num = items_src.size();
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);

  /* Copy buffers. */
  for (const int i : items_src.index_range()) {
    const bNodeTreeInterfaceItem *item_src = items_src[i];
    items_array[i] = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(item_src));
    copy_item(*items_array[i], *item_src);
  }
}

void bNodeTreeInterface::copy_data(const bNodeTreeInterface &src)
{
  root_panel.copy_items(src.root_panel.items());
  this->active_index = src.active_index;
}

void bNodeTreeInterface::free_data()
{
  root_panel.clear_items();
}

bNodeTreeInterfaceSocket *bNodeTreeInterface::add_socket(blender::StringRef name,
                                                         blender::StringRef description,
                                                         blender::StringRef data_type,
                                                         const eNodeTreeInterfaceSocketKind kind,
                                                         bNodeTreeInterfacePanel *parent)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(parent->item));

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
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(parent->item));

  bNodeTreeInterfaceSocket *new_socket = make_socket(
      next_socket_uid++, name, description, data_type, kind);
  parent->insert_item(new_socket->item, index);
  return new_socket;
}

bNodeTreeInterfacePanel *bNodeTreeInterface::add_panel(blender::StringRef name,
                                                       bNodeTreeInterfacePanel *parent)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(parent->item));

  bNodeTreeInterfacePanel *new_panel = make_panel(name);
  parent->add_item(new_panel->item);
  return new_panel;
}

bNodeTreeInterfacePanel *bNodeTreeInterface::insert_panel(blender::StringRef name,
                                                          bNodeTreeInterfacePanel *parent,
                                                          const int index)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(parent->item));

  bNodeTreeInterfacePanel *new_panel = make_panel(name);
  parent->insert_item(new_panel->item, index);
  return new_panel;
}

bNodeTreeInterfaceItem *bNodeTreeInterface::add_item_copy(const bNodeTreeInterfaceItem &item,
                                                          bNodeTreeInterfacePanel *parent)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(item));
  BLI_assert(find_item(parent->item));

  if (parent == nullptr) {
    parent = &root_panel;
  }

  bNodeTreeInterfaceItem *citem = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(&item));
  copy_item(*citem, item);
  parent->add_item(*citem);

  return citem;
}

bNodeTreeInterfaceItem *bNodeTreeInterface::insert_item_copy(const bNodeTreeInterfaceItem &item,
                                                             bNodeTreeInterfacePanel *parent,
                                                             int index)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(item));
  BLI_assert(find_item(parent->item));

  bNodeTreeInterfaceItem *citem = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(&item));
  copy_item(*citem, item);
  parent->insert_item(*citem, index);

  return citem;
}

bool bNodeTreeInterface::remove_item(bNodeTreeInterfaceItem &item)
{
  bNodeTreeInterfacePanel *parent;
  if (!find_item_parent(item, parent)) {
    return false;
  }
  if (parent->remove_item(item, true)) {
    return true;
  }
  return false;
}

void bNodeTreeInterface::clear_items()
{
  root_panel.clear_items();
}

bool bNodeTreeInterface::move_item(bNodeTreeInterfaceItem &item, const int new_index)
{
  bNodeTreeInterfacePanel *parent;
  if (!find_item_parent(item, parent)) {
    return false;
  }
  return parent->move_item(item, new_index);
}

bool bNodeTreeInterface::move_item_to_parent(bNodeTreeInterfaceItem &item,
                                             bNodeTreeInterfacePanel *new_parent,
                                             int new_index)
{
  bNodeTreeInterfacePanel *parent;
  if (!find_item_parent(item, parent)) {
    return false;
  }
  if (parent->remove_item(item, false)) {
    new_parent->insert_item(item, new_index);
    return true;
  }
  return false;
}

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
      bNodeTreeInterfaceSocket *socket = item->get_as_ptr<bNodeTreeInterfaceSocket>();
      BLO_write_struct(writer, bNodeTreeInterfaceSocket, socket);
      BLO_write_string(writer, socket->name);
      BLO_write_string(writer, socket->description);
      BLO_write_string(writer, socket->data_type);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel *panel = item->get_as_ptr<bNodeTreeInterfacePanel>();
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
      bNodeTreeInterfaceSocket *socket = item->get_as_ptr<bNodeTreeInterfaceSocket>();
      BLO_read_data_address(reader, &socket->name);
      BLO_read_data_address(reader, &socket->description);
      BLO_read_data_address(reader, &socket->data_type);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel *panel = item->get_as_ptr<bNodeTreeInterfacePanel>();
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
      bNodeTreeInterfacePanel *panel = item->get_as_ptr<bNodeTreeInterfacePanel>();
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
      bNodeTreeInterfacePanel *panel = item->get_as_ptr<bNodeTreeInterfacePanel>();
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
