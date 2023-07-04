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

/* XXX This doesn't work because there is no actual inheritance between C structs.
 * A handler function for a base type will not be called if the final type is
 * a "derived" class (extended C struct).
 */
// template<typename ExpectedT> struct IsExpectedTypeOperator {
//  bool &result;

//  template<typename ItemT> void operator()()
//  {
//    result = std::is_same<ItemT, ExpectedT>::value;
//  }

//  void operator()()
//  {
//    result = std::is_same<bNodeTreeInterfaceSocket, ExpectedT>::value;
//  }
//};

// template<typename ItemT> bool is_expected_type(const bNodeTreeInterfaceItem &item);
// template<> bool is_expected_type<bNodeTreeInterfaceItem>(const bNodeTreeInterfaceItem &item)
//{
//  /* No naked item type exists. */
//  return false;
//}
// template<> bool is_expected_type<bNodeTreeInterfacePanel>(const bNodeTreeInterfaceItem &item)
//{
//  return item.item_type == NODE_INTERFACE_PANEL;
//}
// template<> bool is_expected_type<bNodeTreeInterfaceSocket>(const bNodeTreeInterfaceItem &item)
//{
//  return item.item_type == NODE_INTERFACE_SOCKET;
//}
// template<> bool is_expected_type<bNodeTreeInterfaceSocketFloat>(const bNodeTreeInterfaceItem
// &item)
//{
//  if (item.item_type == NODE_INTERFACE_SOCKET) {
//    const bNodeTreeInterfaceSocket &socket = reinterpret_cast<const bNodeTreeInterfaceSocket &>(
//        item);
//    return STREQ(socket.data_type, bNodeTreeInterfaceSocketFloat::socket_type_static);
//  }
//  return false;
//}

template<typename T> T &bNodeTreeInterfaceItem::get_as()
{
#  ifndef NDEBUG
//  bool is_valid = false;
//  to_static_type(IsExpectedTypeOperator<T>{is_valid});
//  BLI_assert(is_valid);
#  endif

  return *reinterpret_cast<T *>(this);
}

template<typename T> const T &bNodeTreeInterfaceItem::get_as() const
{
#  ifndef NDEBUG
//  bool is_valid = false;
//  to_static_type(IsExpectedTypeOperator<T>{is_valid});
//  BLI_assert(is_valid);
#  endif

  return *reinterpret_cast<const T *>(this);
}

template<typename ExpectedT> struct CastToTypeOperator {
  bNodeTreeInterfaceItem *item;
  ExpectedT *&result;

  template<typename ItemT> void operator()()
  {
    check_type(reinterpret_cast<ItemT *>(item));
  }

  void operator()()
  {
    result = nullptr;
  }

  template<typename ItemT> bool check_exact_match()
  {
    if (std::is_same<ItemT, ExpectedT>::value) {
      result = reinterpret_cast<ExpectedT *>(item);
      return true;
    }
    else {
      result = nullptr;
      return false;
    }
  }

  void check_type(const bNodeTreeInterfacePanel *)
  {
    check_exact_match<bNodeTreeInterfacePanel>();
  }
  void check_type(const bNodeTreeInterfaceSocket *)
  {
    check_exact_match<bNodeTreeInterfaceSocket>();
  }
  void check_type(const bNodeTreeInterfaceSocketFloat *)
  {
    if (check_exact_match<bNodeTreeInterfaceSocketFloat>()) {
      return;
    }
    check_exact_match<bNodeTreeInterfaceSocket>();
  }
  void check_type(const bNodeTreeInterfaceSocketInt *)
  {
    if (check_exact_match<bNodeTreeInterfaceSocketInt>()) {
      return;
    }
    check_exact_match<bNodeTreeInterfaceSocket>();
  }
  void check_type(const bNodeTreeInterfaceSocketBool *)
  {
    if (check_exact_match<bNodeTreeInterfaceSocketBool>()) {
      return;
    }
    check_exact_match<bNodeTreeInterfaceSocket>();
  }
  void check_type(const bNodeTreeInterfaceSocketString *)
  {
    if (check_exact_match<bNodeTreeInterfaceSocketString>()) {
      return;
    }
    check_exact_match<bNodeTreeInterfaceSocket>();
  }
  void check_type(const bNodeTreeInterfaceSocketObject *)
  {
    if (check_exact_match<bNodeTreeInterfaceSocketObject>()) {
      return;
    }
    check_exact_match<bNodeTreeInterfaceSocket>();
  }
};

template<typename T> T *bNodeTreeInterfaceItem::get_as_ptr()
{
  T *result = nullptr;
  to_static_type<blender::bke::node_interface::AllItemTypes>(CastToTypeOperator<T>{this, result});
  return result;
}

template<typename T> const T *bNodeTreeInterfaceItem::get_as_ptr() const
{
  const T *result = nullptr;
  to_static_type<blender::bke::node_interface::AllItemTypes>(CastToTypeOperator<T>{this, result});
  return result;
}

/* Build a lookup table to avoid having to compare the socket data type with every type name
 * one after another. */
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
static void to_static_type_impl(Func func,
                                eNodeTreeInterfaceItemType item_type,
                                blender::StringRef data_type)
{
  static auto callback_map = build_callback_map<Func>(
      blender::bke::node_interface::SocketItemTypes());

  switch (item_type) {
    case NODE_INTERFACE_PANEL: {
      func.template operator()<bNodeTreeInterfacePanel>();
      break;
    }
    case NODE_INTERFACE_SOCKET: {
      const auto callback = callback_map.lookup_default(data_type, nullptr);
      if (callback != nullptr) {
        callback(func);
      }
      else {
        func.template operator()<bNodeTreeInterfaceSocket>();
      }
      break;
    }
    default:
      /* Call the non-templated `operator()` of the given function object. */
      func();
      break;
  }
}

template<typename Func, typename... Types>
static void to_static_type_resolve(std::tuple<Types...> /*dummy*/,
                                   Func func,
                                   eNodeTreeInterfaceItemType item_type,
                                   blender::StringRef data_type)
{
  to_static_type_impl<Func, Types...>(func, item_type, data_type);
}

template<typename TypeArgs, typename Func>
void bNodeTreeInterfaceItem::to_static_type(Func func,
                                            eNodeTreeInterfaceItemType item_type,
                                            blender::StringRef data_type)
{
  to_static_type_resolve(TypeArgs(), func, item_type, data_type);
}

template<typename TypeArgs, typename Func> void bNodeTreeInterfaceItem::to_static_type(Func func)
{
  const char *data_type = nullptr;
  if (item_type == NODE_INTERFACE_SOCKET) {
    data_type = reinterpret_cast<bNodeTreeInterfaceSocket *>(this)->data_type;
  }
  to_static_type<TypeArgs>(func, eNodeTreeInterfaceItemType(item_type), data_type);
}

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
