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
  foreach_item([&active, &count](bNodeTreeInterfaceItem &item) {
    --count;
    if (count == 0) {
      active = &item;
      return false;
    }
    return true;
  });
  return active;
}

inline const bNodeTreeInterfaceItem *bNodeTreeInterface::active_item() const
{
  const bNodeTreeInterfaceItem *active = nullptr;
  int count = active_index;
  foreach_item([&active, &count](const bNodeTreeInterfaceItem &item) {
    --count;
    if (count == 0) {
      active = &item;
      return false;
    }
    return true;
  });
  return active;
}

inline void bNodeTreeInterface::active_item_set(bNodeTreeInterfaceItem *item)
{
  active_index = 0;
  int count = 0;
  foreach_item([item, &count, this](bNodeTreeInterfaceItem &titem) {
    if (&titem == item) {
      active_index = count;
      return false;
    }
    ++count;
    return true;
  });
}

template<typename ExpectedT> struct IsExpectedTypeOperator {
  bool &result;

  template<typename ItemT> void operator()()
  {
    result = std::is_same<ItemT, ExpectedT>::value;
  }

  void operator()()
  {
    result = std::is_same<bNodeTreeInterfaceSocket, ExpectedT>::value;
  }
};

template<typename T> T &bNodeTreeInterfaceItem::get_as()
{
#  ifndef NDEBUG
  bool is_valid = false;
  to_static_type(IsExpectedTypeOperator<T>{is_valid});
  BLI_assert(is_valid);
#  endif

  return *reinterpret_cast<T *>(this);
}

template<typename T> const T &bNodeTreeInterfaceItem::get_as() const
{
#  ifndef NDEBUG
  bool is_valid = false;
  to_static_type(IsExpectedTypeOperator<T>{is_valid});
  BLI_assert(is_valid);
#  endif

  return *reinterpret_cast<const T *>(this);
}

template<typename ExpectedT> struct CastToTypeOperator {
  bNodeTreeInterfaceItem *item;
  ExpectedT *&result;

  template<typename ItemT> void operator()()
  {
    if (std::is_same<ItemT, ExpectedT>::value) {
      result = reinterpret_cast<ExpectedT *>(item);
    }
    else {
      result = nullptr;
    }
  }

  void operator()()
  {
    if (std::is_same<bNodeTreeInterfaceItem, ExpectedT>::value) {
      result = reinterpret_cast<ExpectedT *>(item);
    }
    else {
      result = nullptr;
    }
  }
};

template<typename T> T *bNodeTreeInterfaceItem::get_as_ptr()
{
  T *result = nullptr;
  to_static_type(CastToTypeOperator<T>{this, result});
  return result;
}

template<typename T> const T *bNodeTreeInterfaceItem::get_as_ptr() const
{
  const T *result = nullptr;
  to_static_type(CastToTypeOperator<T>{this, result});
  return result;
}

template<typename... Types, typename Func>
void bNodeTreeInterfaceItem::to_static_type(Func func,
                                            eNodeTreeInterfaceItemType item_type,
                                            blender::StringRef data_type)
{
  using Callback = void (*)(const Func &func);

  /* Build a lookup table to avoid having to compare the socket data type with every type name
   * one after another. */
  static const blender::Map<std::string, Callback> callback_map = []() {
    blender::Map<std::string, Callback> callback_map;
    /* This adds an entry in the map for every type in #Types. */
    (callback_map.add_new(Types::socket_type_static,
                          [](const Func &func) {
                            /* Call the templated `operator()` of the given function
                             * object. */
                            func.template operator()<Types>();
                          }),
     ...);
    return callback_map;
  }();

  switch (item_type) {
    case NODE_INTERFACE_PANEL: {
      func.template operator()<bNodeTreeInterfacePanel>();
      break;
    }
    case NODE_INTERFACE_SOCKET: {
      const Callback callback = callback_map.lookup_default(data_type, nullptr);
      if (callback != nullptr) {
        callback(func);
      }
      else {
        /* Call the non-templated `operator()` of the given function object. */
        func();
      }
      break;
    }
  }
}

template<typename... Types, typename Func> void bNodeTreeInterfaceItem::to_static_type(Func func)
{
  const char *data_type = nullptr;
  if (item_type == NODE_INTERFACE_SOCKET) {
    data_type = reinterpret_cast<bNodeTreeInterfaceSocket *>(this)->data_type;
  }
  to_static_type(func, eNodeTreeInterfaceItemType(item_type), data_type);
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
