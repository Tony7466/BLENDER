/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_node_tree_interface_types.h"

#include "BLI_span.hh"

#ifdef __cplusplus
#  include <queue>
#  include <type_traits>
#endif

#ifdef __cplusplus

inline blender::Span<const bNodeTreeInterfaceItem *> bNodeTreeInterface::items() const
{
  return blender::Span(items_array, items_num);
}

inline blender::MutableSpan<bNodeTreeInterfaceItem *> bNodeTreeInterface::items()
{
  return blender::MutableSpan(items_array, items_num);
}

inline bNodeTreeInterfaceItem *bNodeTreeInterface::active_item()
{
  if (items().index_range().contains(active_index)) {
    return items()[active_index];
  }
  return nullptr;
}

inline const bNodeTreeInterfaceItem *bNodeTreeInterface::active_item() const
{
  if (items().index_range().contains(active_index)) {
    return items()[active_index];
  }
  return nullptr;
}

inline void bNodeTreeInterface::active_item_set(bNodeTreeInterfaceItem *item)
{
  active_index = items().as_span().first_index_try(item);
}

template<typename ExpectedT> struct IsExpectedTypeOp {
  bool result = false;

  template<typename ItemT> void operator()()
  {
    result = std::is_same<ItemT, ExpectedT>::value;
  }
};

template<typename T> T &bNodeTreeInterfaceItem::get_as()
{
#  ifndef NDEBUG
  IsExpectedTypeOp<T> op;
  apply_typed_operator(op);
  BLI_assert(op.result);
#  endif

  return *reinterpret_cast<T *>(this);
}

template<typename T> const T &bNodeTreeInterfaceItem::get_as() const
{
#  ifndef NDEBUG
  IsExpectedTypeOp<T> op;
  apply_typed_operator(op);
  BLI_assert(op.result);
#  endif

  return *reinterpret_cast<const T *>(this);
}

template<typename T> T *bNodeTreeInterfaceItem::get_as_ptr()
{
  switch (item_type) {
    case NODE_INTERFACE_PANEL: {
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfacePanel>::value;
      return is_valid_type ? reinterpret_cast<T *>(this) : nullptr;
    }
    case NODE_INTERFACE_SOCKET: {
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfaceSocket>::value;
      return is_valid_type ? reinterpret_cast<T *>(this) : nullptr;
    }
    default:
      return nullptr;
  }
}

template<typename T> const T *bNodeTreeInterfaceItem::get_as_ptr() const
{
  switch (item_type) {
    case NODE_INTERFACE_PANEL: {
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfacePanel>::value;
      return is_valid_type ? reinterpret_cast<const T *>(this) : nullptr;
    }
    default:
    case NODE_INTERFACE_SOCKET:
      constexpr bool is_valid_type = std::is_same<T, bNodeTreeInterfaceSocket>::value;
      return is_valid_type ? reinterpret_cast<const T *>(this) : nullptr;
  }
}

template<typename OpT>
void apply_typed_operator(const eNodeTreeInterfaceItemType item_type,
                          blender::StringRef socket_data_type,
                          OpT op)
{
  switch (item_type) {
    case NODE_INTERFACE_PANEL:
      op.template operator()<bNodeTreeInterfacePanel>();
      break;
    case NODE_INTERFACE_SOCKET: {
      if (socket_data_type == bNodeTreeInterfaceSocketFloat::socket_type_static) {
        op.template operator()<bNodeTreeInterfaceSocketFloat>();
      }
      else if (socket_data_type == bNodeTreeInterfaceSocketInt::socket_type_static) {
        op.template operator()<bNodeTreeInterfaceSocketInt>();
      }
      else if (socket_data_type == bNodeTreeInterfaceSocketBool::socket_type_static) {
        op.template operator()<bNodeTreeInterfaceSocketBool>();
      }
      else if (socket_data_type == bNodeTreeInterfaceSocketString::socket_type_static) {
        op.template operator()<bNodeTreeInterfaceSocketString>();
      }
      else if (socket_data_type == bNodeTreeInterfaceSocketObject::socket_type_static) {
        op.template operator()<bNodeTreeInterfaceSocketObject>();
      }
      break;
    }
  }
}

template<typename OpT> void bNodeTreeInterfaceItem::apply_typed_operator(OpT op) const
{
  const blender::StringRef data_type =
      (item_type == NODE_INTERFACE_SOCKET ?
           reinterpret_cast<const bNodeTreeInterfaceSocket *>(this)->data_type :
           "");
  return ::apply_typed_operator<OpT>(eNodeTreeInterfaceItemType(item_type), data_type, op);
}

template<typename OpT> void bNodeTreeInterface::foreach_item(OpT op)
{
  std::queue<bNodeTreeInterfacePanel *> queue;

  if (op(root_panel) == false) {
    return;
  }
  queue.push(&root_panel);

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

template<typename OpT> void bNodeTreeInterface::foreach_item(OpT op) const
{
  std::queue<const bNodeTreeInterfacePanel *> queue;

  if (op(root_panel) == false) {
    return;
  }
  queue.push(&root_panel);

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
