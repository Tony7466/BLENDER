/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_node_tree_interface_types.h"
#include "DNA_node_types.h"

#include "BLI_parameter_pack_utils.hh"
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
      break;
    }
    case NODE_INTERFACE_PANEL: {
      match |= std::is_same<T, bNodeTreeInterfacePanel>::value;
      break;
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

template<typename T> T *bNodeTreeInterfaceItem::get_as_ptr()
{
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

namespace blender::bke::node_interface {

namespace socket_types {

constexpr const char *node_socket_data_float = "NodeSocketFloat";
constexpr const char *node_socket_data_int = "NodeSocketInt";
constexpr const char *node_socket_data_bool = "NodeSocketBoolean";
constexpr const char *node_socket_data_rotation = "NodeSocketRotation";
constexpr const char *node_socket_data_vector = "NodeSocketVector";
constexpr const char *node_socket_data_color = "NodeSocketColor";
constexpr const char *node_socket_data_string = "NodeSocketString";
constexpr const char *node_socket_data_object = "NodeSocketObject";
constexpr const char *node_socket_data_image = "NodeSocketImage";
constexpr const char *node_socket_data_collection = "NodeSocketCollection";
constexpr const char *node_socket_data_texture = "NodeSocketTexture";
constexpr const char *node_socket_data_material = "NodeSocketMaterial";

template<typename Fn> void socket_data_to_static_type(const char *socket_type, const Fn &fn)
{
  if (STREQ(socket_type, socket_types::node_socket_data_float)) {
    fn.template operator()<bNodeSocketValueFloat>();
  }
  else if (STREQ(socket_type, socket_types::node_socket_data_int)) {
    fn.template operator()<bNodeSocketValueInt>();
  }
  else if (STREQ(socket_type, socket_types::node_socket_data_bool)) {
    fn.template operator()<bNodeSocketValueBoolean>();
  }
  else if (STREQ(socket_type, socket_types::node_socket_data_rotation)) {
    fn.template operator()<bNodeSocketValueRotation>();
  }
  else if (STREQ(socket_type, socket_types::node_socket_data_vector)) {
    fn.template operator()<bNodeSocketValueVector>();
  }
  else if (STREQ(socket_type, socket_types::node_socket_data_color)) {
    fn.template operator()<bNodeSocketValueRGBA>();
  }
  else if (STREQ(socket_type, socket_types::node_socket_data_string)) {
    fn.template operator()<bNodeSocketValueString>();
  }
  else if (STREQ(socket_type, socket_types::node_socket_data_object)) {
    fn.template operator()<bNodeSocketValueObject>();
  }
  else if (STREQ(socket_type, socket_types::node_socket_data_image)) {
    fn.template operator()<bNodeSocketValueImage>();
  }
  else if (STREQ(socket_type, socket_types::node_socket_data_collection)) {
    fn.template operator()<bNodeSocketValueCollection>();
  }
  else if (STREQ(socket_type, socket_types::node_socket_data_texture)) {
    fn.template operator()<bNodeSocketValueTexture>();
  }
  else if (STREQ(socket_type, socket_types::node_socket_data_material)) {
    fn.template operator()<bNodeSocketValueMaterial>();
  }
  else {
    /* Unhandled type */
    BLI_assert_unreachable();
  }
}

namespace detail {

template<typename Fn> struct TypeTagExecutor {
  const Fn &fn;

  TypeTagExecutor(const Fn &fn_) : fn(fn_) {}

  template<typename T> void operator()() const
  {
    fn(TypeTag<T>{});
  }
};

}  // namespace detail

template<typename Fn> void socket_data_to_static_type_tag(const char *socket_type, const Fn &fn)
{
  detail::TypeTagExecutor executor{fn};
  socket_data_to_static_type(socket_type, executor);
}

}  // namespace socket_types

template<typename T> bool socket_data_is_type(const char *socket_type)
{
  bool match = false;
  socket_types::socket_data_to_static_type_tag(socket_type, [&match](auto type_tag) {
    using SocketDataType = typename decltype(type_tag)::type;
    match |= std::is_same<T, SocketDataType>::value;
  });
  return match;
}

}  // namespace blender::bke::node_interface

template<typename T> T &bNodeTreeInterfaceSocket::get_data()
{
  BLI_assert(blender::bke::node_interface::socket_data_is_type<T>(socket_type));
  return *static_cast<T *>(socket_data);
}

template<typename T> const T &bNodeTreeInterfaceSocket::get_data() const
{
  BLI_assert(blender::bke::node_interface::socket_data_is_type<T>(socket_type));
  return *static_cast<const T *>(socket_data);
}

namespace blender::bke::node_interface::detail {

template<typename Func, typename ReturnT> struct PanelForeachExecutor {
  void operator()(bNodeTreeInterfacePanel &panel, Func op)
  {
    std::queue<bNodeTreeInterfacePanel *> queue;

    if (op(panel.item) == false) {
      return;
    }
    queue.push(&panel);

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
};

template<typename Func> struct PanelForeachExecutor<Func, void> {
  void operator()(bNodeTreeInterfacePanel &panel, Func op)
  {
    std::queue<bNodeTreeInterfacePanel *> queue;

    op(panel.item);
    queue.push(&panel);

    while (!queue.empty()) {
      bNodeTreeInterfacePanel *parent = queue.front();
      queue.pop();

      for (bNodeTreeInterfaceItem *item : parent->items()) {
        op(*item);

        if (item->item_type == NODE_INTERFACE_PANEL) {
          bNodeTreeInterfacePanel *panel = reinterpret_cast<bNodeTreeInterfacePanel *>(item);
          queue.push(panel);
        }
      }
    }
  }
};

}  // namespace blender::bke::node_interface::detail

template<typename Func> void bNodeTreeInterfacePanel::foreach_item(Func op)
{
  using ResultT = std::result_of<Func(bNodeTreeInterfaceItem &)>;
  blender::bke::node_interface::detail::PanelForeachExecutor<Func, ResultT> executor;
  executor(*this, op);
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
                                     struct bNodeTreeInterface *interface) ATTR_NONNULL(1);
void BKE_nodetree_interface_read_expand(struct BlendExpander *expander,
                                        struct bNodeTreeInterface *interface);

#ifdef __cplusplus
}
#endif
