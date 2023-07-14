/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_node_tree_interface_types.h"

#include "BLI_map.hh"
#include "BLI_vector.hh"

#ifdef __cplusplus
#  include <queue>
#  include <type_traits>
#endif

#ifdef __cplusplus

namespace blender::bke {

/* Runtime topology cache for linear access to items. */
typedef struct bNodeTreeInterfaceCache {
  blender::Vector<bNodeTreeInterfaceItem *> items;

  void rebuild(bNodeTreeInterface &interface);
} bNodeTreeInterfaceCache;

}  // namespace blender::bke

inline bNodeTreeInterfaceItem *bNodeTreeInterface::active_item()
{
  bNodeTreeInterfaceItem *active = nullptr;
  int count = active_index;
  foreach_item([this, &active, &count](bNodeTreeInterfaceItem &item) {
    /* Skip root panel. */
    if (&item == &root_panel.item) {
      return true;
    }
    if (count == 0) {
      active = &item;
      return false;
    }
    --count;
    return true;
  });
  return active;
}

inline const bNodeTreeInterfaceItem *bNodeTreeInterface::active_item() const
{
  const bNodeTreeInterfaceItem *active = nullptr;
  int count = active_index;
  foreach_item([this, &active, &count](const bNodeTreeInterfaceItem &item) {
    /* Skip root panel. */
    if (&item == &root_panel.item) {
      return true;
    }
    if (count == 0) {
      active = &item;
      return false;
    }
    --count;
    return true;
  });
  return active;
}

inline void bNodeTreeInterface::active_item_set(bNodeTreeInterfaceItem *item)
{
  active_index = 0;
  int count = 0;
  foreach_item([this, item, &count](bNodeTreeInterfaceItem &titem) {
    /* Skip root panel. */
    if (&titem == &root_panel.item) {
      return true;
    }
    if (&titem == item) {
      active_index = count;
      return false;
    }
    ++count;
    return true;
  });
}

namespace blender::bke::node_interface::detail {

template<typename T> static bool item_is_type(const bNodeTreeInterfaceItem &item)
{
  bool match = false;
  switch (item.item_type) {
    case NODE_INTERFACE_SOCKET: {
      match |= std::is_same<T, bNodeTreeInterfaceSocket>::value;

      const bNodeTreeInterfaceSocket &socket = reinterpret_cast<const bNodeTreeInterfaceSocket &>(
          item);
      if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketFloat::socket_type_static)) {
        match |= std::is_same<T, bNodeTreeInterfaceSocketFloat>::value;
      }
      else if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketInt::socket_type_static)) {
        match |= std::is_same<T, bNodeTreeInterfaceSocketInt>::value;
      }
      else if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketBool::socket_type_static)) {
        match |= std::is_same<T, bNodeTreeInterfaceSocketBool>::value;
      }
      else if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketString::socket_type_static)) {
        match |= std::is_same<T, bNodeTreeInterfaceSocketString>::value;
      }
      else if (STREQ(socket.socket_type, bNodeTreeInterfaceSocketObject::socket_type_static)) {
        match |= std::is_same<T, bNodeTreeInterfaceSocketObject>::value;
      }
      else {
        /* Unhandled socket type. */
        BLI_assert_unreachable();
      }

      break;
    }
    case NODE_INTERFACE_PANEL: {
      match |= std::is_same<T, bNodeTreeInterfacePanel>::value;
    }
  }
  return match;
}

}  // namespace blender::bke::node_interface::detail

template<typename T> T &bNodeTreeInterfaceItem::get_as()
{
  BLI_assert(blender::bke::node_interface::detail::item_is_type<T>(*this));
  return reinterpret_cast<T &>(*this);
}

template<typename T> const T &bNodeTreeInterfaceItem::get_as() const
{
  BLI_assert(blender::bke::node_interface::detail::item_is_type<T>(*this));
  return reinterpret_cast<const T &>(*this);
}

// namespace detail {

// template<typename T> struct ItemMatchFunc {
//  bool &match;

//  template<typename U> void operator()()
//  {
//    match = std::is_same<T, U>::value;
//  }
//};

//}  // namespace detail

template<typename T> T *bNodeTreeInterfaceItem::get_as_ptr()
{
  //  bool match = false;

  //  /* Handle base types first, since there is no way to detect nested C type "inheritance" based
  //  on
  //   * types alone. */
  //  switch (eNodeTreeInterfaceItemType(item_type)) {
  //    case NODE_INTERFACE_SOCKET: {
  //      if (std::is_same<T, bNodeTreeInterfaceSocket>::value) {
  //        match = true;
  //        break;
  //      }

  //      /* Use static type dispatcher to find the final socket type */
  //      detail::ItemMatchFunc<T> func{match};
  //      to_static_type<blender::bke::node_interface::SocketItemTypes>(func);
  //      break;
  //    }
  //    case NODE_INTERFACE_PANEL: {
  //      match = std::is_same<T, bNodeTreeInterfacePanel>::value;
  //      break;
  //    }
  //  }
  //  return match ? reinterpret_cast<T *>(this) : nullptr;
  if (blender::bke::node_interface::detail::item_is_type<T>(*this)) {
    return reinterpret_cast<T *>(this);
  }
  return nullptr;
}

template<typename T> const T *bNodeTreeInterfaceItem::get_as_ptr() const
{
  //  return const_cast<bNodeTreeInterfaceItem *>(this)->get_as_ptr<T>();
  if (blender::bke::node_interface::detail::item_is_type<T>(*this)) {
    return reinterpret_cast<const T *>(this);
  }
  return nullptr;
}

#  if 0
namespace detail {

/* Build a lookup table to avoid having to compare the socket data type with every
 * type name one after another. */
template<typename Func, typename... Types>
static auto build_callback_map(const std::tuple<Types...> & /*dummy*/)
{
  using Callback = void (*)(Func &);
  using CallbackMap = blender::Map<std::string, Callback>;

  CallbackMap callback_map;
  /* This adds an entry in the map for every type in #Types. */
  (callback_map.add_new(Types::socket_type_static,
                        [](Func &func) {
                          /* Call the templated `operator()` of the given function
                           * object. */
                          func.template operator()<Types>();
                        }),
   ...);
  return callback_map;
};

template<typename Func, typename... Types>
static void to_static_type_impl(eNodeTreeInterfaceItemType item_type,
                                blender::StringRef socket_type,
                                Func func)
{
  static auto callback_map = build_callback_map<Func>(
      blender::bke::node_interface::SocketItemTypes());

  switch (item_type) {
    case NODE_INTERFACE_PANEL: {
      func.template operator()<bNodeTreeInterfacePanel>();
      break;
    }
    case NODE_INTERFACE_SOCKET: {
      const auto callback = callback_map.lookup_default(socket_type, nullptr);
      if (callback != nullptr) {
        callback(func);
      }
      else {
        func.template operator()<bNodeTreeInterfaceSocket>();
      }
      break;
    }
  }
}

template<typename Func, typename... Types>
static void to_static_type_resolve(std::tuple<Types...> /*dummy*/,
                                   eNodeTreeInterfaceItemType item_type,
                                   blender::StringRef socket_type,
                                   Func func)
{
  to_static_type_impl<Func, Types...>(item_type, socket_type, func);
}

}  // namespace detail

template<typename Types, typename Func>
void bNodeTreeInterfaceItem::to_static_type(eNodeTreeInterfaceItemType item_type,
                                            blender::StringRef socket_type,
                                            Func func)
{
  detail::to_static_type_resolve(Types(), item_type, socket_type, func);
}

template<typename Types, typename Func> void bNodeTreeInterfaceItem::to_static_type(Func func)
{
  const char *socket_type = (item_type == NODE_INTERFACE_SOCKET) ?
                                reinterpret_cast<bNodeTreeInterfaceSocket *>(this)->socket_type :
                                "";
  detail::to_static_type_resolve(
      Types(), eNodeTreeInterfaceItemType(item_type), socket_type, func);
}

template<typename Types, typename Func>
void bNodeTreeInterfaceItem::to_static_type(Func func) const
{
  const char *socket_type =
      (item_type == NODE_INTERFACE_SOCKET) ?
          reinterpret_cast<const bNodeTreeInterfaceSocket *>(this)->socket_type :
          "";
  detail::to_static_type_resolve(
      Types(), eNodeTreeInterfaceItemType(item_type), socket_type, func);
}
#  endif

template<typename Func> void bNodeTreeInterfacePanel::foreach_item(Func op)
{
  std::queue<bNodeTreeInterfacePanel *> queue;

  if (op(this->item) == false) {
    return;
  }
  queue.push(this);

  while (!queue.empty()) {
    bNodeTreeInterfacePanel *parent = queue.front();
    queue.pop();

    for (bNodeTreeInterfaceItem *item : parent->items()) {
      if (op(*item) == false) {
        return;
      }

      if (item->item_type == NODE_INTERFACE_PANEL) {
        bNodeTreeInterfacePanel *panel = reinterpret_cast<bNodeTreeInterfacePanel *>(item);
        queue.push(panel);
      }
    }
  }
}

template<typename Func> void bNodeTreeInterfacePanel::foreach_item(Func op) const
{
  std::queue<const bNodeTreeInterfacePanel *> queue;

  if (op(this->item) == false) {
    return;
  }
  queue.push(this);

  while (!queue.empty()) {
    const bNodeTreeInterfacePanel *parent = queue.front();
    queue.pop();

    for (const bNodeTreeInterfaceItem *item : parent->items()) {
      if (op(*item) == false) {
        return;
      }

      if (item->item_type == NODE_INTERFACE_PANEL) {
        const bNodeTreeInterfacePanel *panel = reinterpret_cast<const bNodeTreeInterfacePanel *>(
            item);
        queue.push(panel);
      }
    }
  }
}

#endif

#ifdef __cplusplus
extern "C" {
#endif

void BKE_nodetree_interface_init(struct bNodeTreeInterface *interface);
void BKE_nodetree_interface_copy(struct bNodeTreeInterface *interface_dst,
                                 const struct bNodeTreeInterface *interface_src);
void BKE_nodetree_interface_free(struct bNodeTreeInterface *interface);

/* .blend file I/O */

void BKE_nodetree_interface_write(struct BlendWriter *writer,
                                  struct bNodeTreeInterface *interface);
void BKE_nodetree_interface_read_data(struct BlendDataReader *reader,
                                      struct bNodeTreeInterface *interface);
void BKE_nodetree_interface_read_lib(struct BlendLibReader *reader,
                                     struct ID *id,
                                     struct bNodeTreeInterface *interface);
void BKE_nodetree_interface_read_expand(struct BlendExpander *expander,
                                        struct bNodeTreeInterface *interface);

#ifdef __cplusplus
}
#endif
