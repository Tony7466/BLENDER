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

namespace detail {

static void item_write_struct(BlendWriter *writer, bNodeTreeInterfaceItem &item);

template<typename SocketT>
static bNodeTreeInterfaceSocket *try_alloc_socket_type(blender::StringRef socket_type)
{
  if (socket_type == SocketT::socket_type_static) {
    return reinterpret_cast<bNodeTreeInterfaceSocket *>(MEM_cnew<SocketT>(__func__));
  }
  return nullptr;
}

static bNodeTreeInterfaceSocket *alloc_socket_type(blender::StringRef socket_type)
{
  if (bNodeTreeInterfaceSocket *result = try_alloc_socket_type<bNodeTreeInterfaceSocketFloat>(
          socket_type))
  {
    return result;
  }
  if (bNodeTreeInterfaceSocket *result = try_alloc_socket_type<bNodeTreeInterfaceSocketInt>(
          socket_type))
  {
    return result;
  }
  if (bNodeTreeInterfaceSocket *result = try_alloc_socket_type<bNodeTreeInterfaceSocketBool>(
          socket_type))
  {
    return result;
  }
  if (bNodeTreeInterfaceSocket *result = try_alloc_socket_type<bNodeTreeInterfaceSocketString>(
          socket_type))
  {
    return result;
  }
  if (bNodeTreeInterfaceSocket *result = try_alloc_socket_type<bNodeTreeInterfaceSocketObject>(
          socket_type))
  {
    return result;
  }
  /* Unhandled socket type. */
  BLI_assert_unreachable();
  return nullptr;
}

static void item_copy(bNodeTreeInterfaceItem &dst, const bNodeTreeInterfaceItem &src)
{
  switch (dst.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &dst_socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(dst);
      const bNodeTreeInterfaceSocket &src_socket =
          reinterpret_cast<const bNodeTreeInterfaceSocket &>(src);
      BLI_assert(src_socket.name != nullptr);
      BLI_assert(src_socket.socket_type != nullptr);

      dst_socket.name = BLI_strdup(src_socket.name);
      dst_socket.description = src_socket.description ? BLI_strdup(src_socket.description) :
                                                        nullptr;
      dst_socket.socket_type = BLI_strdup(src_socket.socket_type);
      dst_socket.default_attribute_name = src_socket.default_attribute_name ?
                                              BLI_strdup(src_socket.default_attribute_name) :
                                              nullptr;
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &dst_panel = reinterpret_cast<bNodeTreeInterfacePanel &>(dst);
      const bNodeTreeInterfacePanel &src_panel = reinterpret_cast<const bNodeTreeInterfacePanel &>(
          src);
      BLI_assert(src_panel.name != nullptr);

      dst_panel.name = BLI_strdup(src_panel.name);
      dst_panel.copy_items(src_panel.items());
      break;
    }
  }
}

static void item_free(bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);

      MEM_SAFE_FREE(socket.name);
      MEM_SAFE_FREE(socket.description);
      MEM_SAFE_FREE(socket.socket_type);
      MEM_SAFE_FREE(socket.default_attribute_name);
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);

      panel.clear_items();
      MEM_SAFE_FREE(panel.name);
      break;
    }
  }

  MEM_freeN(&item);
}

static void item_write_data(BlendWriter *writer, bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);
      BLO_write_string(writer, socket.name);
      BLO_write_string(writer, socket.description);
      BLO_write_string(writer, socket.socket_type);
      BLO_write_string(writer, socket.default_attribute_name);

      if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketString::socket_type_static)) {
        bNodeTreeInterfaceSocketString &socket_string =
            reinterpret_cast<bNodeTreeInterfaceSocketString &>(socket);
        BLO_write_string(writer, socket_string.default_value);
      }
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);
      BLO_write_string(writer, panel.name);
      BLO_write_pointer_array(writer, panel.items_num, panel.items_array);
      for (bNodeTreeInterfaceItem *child_item : panel.items()) {
        item_write_struct(writer, *child_item);
      }
      break;
    }
  }
}

static void item_write_struct(BlendWriter *writer, bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);

      if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketFloat::socket_type_static)) {
        BLO_write_struct(writer, bNodeTreeInterfaceSocketFloat, &socket);
      }
      else if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketInt::socket_type_static)) {
        BLO_write_struct(writer, bNodeTreeInterfaceSocketInt, &socket);
      }
      else if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketBool::socket_type_static)) {
        BLO_write_struct(writer, bNodeTreeInterfaceSocketBool, &socket);
      }
      else if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketString::socket_type_static)) {
        BLO_write_struct(writer, bNodeTreeInterfaceSocketString, &socket);
      }
      else if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketObject::socket_type_static)) {
        BLO_write_struct(writer, bNodeTreeInterfaceSocketObject, &socket);
      }
      else {
        BLO_write_struct(writer, bNodeTreeInterfaceSocket, &socket);
      }
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);

      BLO_write_struct(writer, bNodeTreeInterfacePanel, &panel);
      break;
    }
  }

  item_write_data(writer, item);
}

static void item_read_data(BlendDataReader *reader, bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);
      BLO_read_data_address(reader, &socket.name);
      BLO_read_data_address(reader, &socket.description);
      BLO_read_data_address(reader, &socket.socket_type);
      BLO_read_data_address(reader, &socket.default_attribute_name);

      if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketString::socket_type_static)) {
        bNodeTreeInterfaceSocketString &socket_string =
            reinterpret_cast<bNodeTreeInterfaceSocketString &>(socket);
        BLO_read_data_address(reader, &socket_string.default_value);
      }
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);
      BLO_read_data_address(reader, &panel.name);
      BLO_read_pointer_array(reader, reinterpret_cast<void **>(&panel.items_array));
      for (const int i : blender::IndexRange(panel.items_num)) {
        BLO_read_data_address(reader, &panel.items_array[i]);
        item_read_data(reader, *panel.items_array[i]);
      }
      break;
    }
  }
}

static void item_read_lib(BlendLibReader *reader, ID *id, bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);

      if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketObject::socket_type_static)) {
        bNodeTreeInterfaceSocketObject &socket_object =
            reinterpret_cast<bNodeTreeInterfaceSocketObject &>(socket);
        BLO_read_id_address(reader, id, &socket_object.default_value);
      }
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);
      for (bNodeTreeInterfaceItem *item : panel.items()) {
        item_read_lib(reader, id, *item);
      }
      break;
    }
  }
}

static void item_read_expand(BlendExpander *expander, bNodeTreeInterfaceItem &item)
{
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      bNodeTreeInterfaceSocket &socket = reinterpret_cast<bNodeTreeInterfaceSocket &>(item);

      if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketObject::socket_type_static)) {
        bNodeTreeInterfaceSocketObject &socket_object =
            reinterpret_cast<bNodeTreeInterfaceSocketObject &>(socket);
        BLO_expand(expander, socket_object.default_value);
      }
      break;
    }
    case NODE_INTERFACE_PANEL: {
      bNodeTreeInterfacePanel &panel = reinterpret_cast<bNodeTreeInterfacePanel &>(item);
      for (bNodeTreeInterfaceItem *item : panel.items()) {
        item_read_expand(expander, *item);
      }
      break;
    }
  }
}

}  // namespace detail

std::string bNodeTreeInterfaceSocket::socket_identifier() const
{
  return "Socket" + std::to_string(uid);
}

blender::ColorGeometry4f bNodeTreeInterfaceSocket::socket_color() const
{
  bNodeSocketType *typeinfo = nodeSocketTypeFind(socket_type);
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
    detail::item_free(item);
  }

  return true;
}

void bNodeTreeInterfacePanel::clear_items()
{
  for (bNodeTreeInterfaceItem *item : items()) {
    detail::item_free(*item);
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
                                             blender::StringRef socket_type,
                                             const eNodeTreeInterfaceSocketFlag flag)
{
  BLI_assert(name.data() != nullptr);
  BLI_assert(socket_type.data() != nullptr);

  bNodeTreeInterfaceSocket *new_socket = detail::alloc_socket_type(socket_type);
  BLI_assert(new_socket);

  /* Init common socket properties. */
  new_socket->uid = uid;
  new_socket->item.item_type = NODE_INTERFACE_SOCKET;
  new_socket->name = BLI_strdup(name.data());
  new_socket->description = description.data() ? BLI_strdup(description.data()) : nullptr;
  new_socket->socket_type = BLI_strdup(socket_type.data());
  new_socket->flag = flag;

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
  items_num = items_src.size();
  items_array = MEM_cnew_array<bNodeTreeInterfaceItem *>(items_num, __func__);

  /* Copy buffers. */
  for (const int i : items_src.index_range()) {
    const bNodeTreeInterfaceItem *item_src = items_src[i];
    items_array[i] = static_cast<bNodeTreeInterfaceItem *>(MEM_dupallocN(item_src));
    detail::item_copy(*items_array[i], *item_src);
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
                                                         blender::StringRef socket_type,
                                                         const eNodeTreeInterfaceSocketFlag flag,
                                                         bNodeTreeInterfacePanel *parent)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(parent->item));

  bNodeTreeInterfaceSocket *new_socket = make_socket(
      next_socket_uid++, name, description, socket_type, flag);
  parent->add_item(new_socket->item);
  return new_socket;
}

bNodeTreeInterfaceSocket *bNodeTreeInterface::insert_socket(
    blender::StringRef name,
    blender::StringRef description,
    blender::StringRef socket_type,
    const eNodeTreeInterfaceSocketFlag flag,
    bNodeTreeInterfacePanel *parent,
    const int index)
{
  if (parent == nullptr) {
    parent = &root_panel;
  }
  BLI_assert(find_item(parent->item));

  bNodeTreeInterfaceSocket *new_socket = make_socket(
      next_socket_uid++, name, description, socket_type, flag);
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
  detail::item_copy(*citem, item);
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
  detail::item_copy(*citem, item);
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

void BKE_nodetree_interface_write(BlendWriter *writer, bNodeTreeInterface *interface)
{
  BLO_write_struct(writer, bNodeTreeInterface, interface);
  /* Don't write the root panel struct itself, it's nested in the interface struct. */
  detail::item_write_data(writer, interface->root_panel.item);
}

void BKE_nodetree_interface_read_data(BlendDataReader *reader, bNodeTreeInterface *interface)
{
  detail::item_read_data(reader, interface->root_panel.item);
}

void BKE_nodetree_interface_read_lib(BlendLibReader *reader, ID *id, bNodeTreeInterface *interface)
{
  detail::item_read_lib(reader, id, interface->root_panel.item);
}

void BKE_nodetree_interface_read_expand(BlendExpander *expander, bNodeTreeInterface *interface)
{
  detail::item_read_expand(expander, interface->root_panel.item);
}
