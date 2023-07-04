/* SPDX-FileCopyrightText: 2005 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "BLI_utildefines.h"

#ifdef __cplusplus
#  include "BLI_color.hh"
#  include "BLI_span.hh"
#  include "BLI_string_ref.hh"

#  include <memory>
#endif

struct bNodeTreeInterfaceItem;
struct bNodeTreeInterfacePanel;
struct bNodeTreeInterfaceSocket;
struct bNodeTreeInterfaceSocketFloat;
struct bNodeTreeInterfaceSocketInt;
struct bNodeTreeInterfaceSocketBool;
struct bNodeTreeInterfaceSocketString;
struct bNodeTreeInterfaceSocketObject;

/** Type of interface item. */
typedef enum eNodeTreeInterfaceItemType {
  NODE_INTERFACE_PANEL = 0,
  NODE_INTERFACE_SOCKET = 1,
} eNodeTreeInterfaceItemType;

#ifdef __cplusplus

namespace blender::bke::node_interface {

/* List of final item types. */
using SocketItemTypes = std::tuple<bNodeTreeInterfaceSocketFloat,
                                   bNodeTreeInterfaceSocketInt,
                                   bNodeTreeInterfaceSocketBool,
                                   bNodeTreeInterfaceSocketString,
                                   bNodeTreeInterfaceSocketObject>;
/* List of final item types. */
using FinalItemTypes = std::tuple<bNodeTreeInterfacePanel,
                                  bNodeTreeInterfaceSocketFloat,
                                  bNodeTreeInterfaceSocketInt,
                                  bNodeTreeInterfaceSocketBool,
                                  bNodeTreeInterfaceSocketString,
                                  bNodeTreeInterfaceSocketObject>;
/* List of all item types, including intermediate classes. */
using AllItemTypes = std::tuple<bNodeTreeInterfaceItem,
                                bNodeTreeInterfacePanel,
                                bNodeTreeInterfaceSocket,
                                bNodeTreeInterfaceSocketFloat,
                                bNodeTreeInterfaceSocketInt,
                                bNodeTreeInterfaceSocketBool,
                                bNodeTreeInterfaceSocketString,
                                bNodeTreeInterfaceSocketObject>;
}  // namespace blender::bke::node_interface
#endif

/** Describes a socket and all necessary details for a node declaration. */
typedef struct bNodeTreeInterfaceItem {
  /* eNodeTreeInterfaceItemType */
  char item_type;
  char _pad[7];

#ifdef __cplusplus
  template<typename T> T &get_as();
  template<typename T> const T &get_as() const;

  template<typename T> T *get_as_ptr();
  template<typename T> const T *get_as_ptr() const;

  /* Call function with the static item type. */
  template<typename TypeArgs, typename Func>
  static void to_static_type(Func func,
                             eNodeTreeInterfaceItemType item_type,
                             blender::StringRef data_type);
  /* Call function with the static type of the item instance. */
  template<typename TypeArgs, typename Func> void to_static_type(Func func);

  /* Overloaded by subclasses */
  void copy_impl(const bNodeTreeInterfaceItem &src);
  /* Overloaded by subclasses */
  void free_impl();
#endif
} bNodeTreeInterfaceItem;

/** Socket side (input/output). */
typedef enum eNodeTreeInterfaceSocketKind {
  NODE_INTERFACE_INPUT = 1 << 0,
  NODE_INTERFACE_OUTPUT = 1 << 1,
} eNodeTreeInterfaceSocketKind;
ENUM_OPERATORS(eNodeTreeInterfaceSocketKind, NODE_INTERFACE_OUTPUT);

typedef struct bNodeTreeInterfaceSocket {
  bNodeTreeInterfaceItem item;

  char *name;
  char *description;
  char *data_type;
  /* eNodeTreeInterfaceSocketKind */
  int kind;

  /* Unique id for constructing socket identifiers. */
  int uid;

#ifdef __cplusplus
  std::string socket_identifier() const;
  blender::ColorGeometry4f socket_color() const;

  void copy_impl(const bNodeTreeInterfaceSocket &src);
  void free_impl();
#endif
} bNodeTreeInterfaceSocket;

typedef struct bNodeTreeInterfaceSocketFloat {
  bNodeTreeInterfaceSocket base;

#ifdef __cplusplus
  using ValueType = float;
  using SocketValueType = struct bNodeSocketValueFloat;

  static const char *socket_type_static;

  void copy_impl(const bNodeTreeInterfaceSocketFloat &src);
  void free_impl();
#endif
} bNodeTreeInterfaceSocketFloat;

typedef struct bNodeTreeInterfaceSocketInt {
  bNodeTreeInterfaceSocket base;

#ifdef __cplusplus
  using ValueType = int;
  using SocketValueType = struct bNodeSocketValueInt;

  static const char *socket_type_static;

  void copy_impl(const bNodeTreeInterfaceSocketInt &src);
  void free_impl();
#endif
} bNodeTreeInterfaceSocketInt;

typedef struct bNodeTreeInterfaceSocketBool {
  bNodeTreeInterfaceSocket base;

#ifdef __cplusplus
  using ValueType = bool;
  using SocketValueType = struct bNodeSocketValueBoolean;

  static const char *socket_type_static;

  void copy_impl(const bNodeTreeInterfaceSocketBool &src);
  void free_impl();
#endif
} bNodeTreeInterfaceSocketBool;

typedef struct bNodeTreeInterfaceSocketString {
  bNodeTreeInterfaceSocket base;

#ifdef __cplusplus
  using ValueType = std::string;
  using SocketValueType = struct bNodeSocketValueString;

  static const char *socket_type_static;

  void copy_impl(const bNodeTreeInterfaceSocketString &src);
  void free_impl();
#endif
} bNodeTreeInterfaceSocketString;

typedef struct bNodeTreeInterfaceSocketObject {
  bNodeTreeInterfaceSocket base;

#ifdef __cplusplus
  using ValueType = struct Object *;
  using SocketValueType = struct bNodeSocketValueObject;

  static const char *socket_type_static;

  void copy_impl(const bNodeTreeInterfaceSocketObject &src);
  void free_impl();
#endif
} bNodeTreeInterfaceSocketObject;

typedef struct bNodeTreeInterfacePanel {
  bNodeTreeInterfaceItem item;

  char *name;

  bNodeTreeInterfaceItem **items_array;
  int items_num;
  char _pad[4];

#ifdef __cplusplus
  static bNodeTreeInterfaceSocket *create(blender::StringRef name);

  blender::IndexRange items_range() const;
  blender::Span<const bNodeTreeInterfaceItem *> items() const;
  blender::MutableSpan<bNodeTreeInterfaceItem *> items();

  int item_index(const bNodeTreeInterfaceItem &item) const;
  bool contains_item(const bNodeTreeInterfaceItem &item) const;

  /**
   * Search for an item in the interface.
   * \return True if the item was found.
   */
  bool find_item(const bNodeTreeInterfaceItem &item) const;
  /**
   * Search for an item and its parent in the interface.
   * \param r_parent: Parent containing the item.
   * \return True if the item was found in any panel.
   */
  bool find_item_parent(const bNodeTreeInterfaceItem &item, bNodeTreeInterfacePanel *&r_parent);

  /**
   * Apply an operator to every item in the panel.
   * The items are visited in drawing order from top to bottom.
   * The operator should have the following signature:
   *
   *   bool MyOperator(bNodeTreeInterfaceItem &item);
   *
   * If the operator returns false for any item the iteration stops.
   */
  template<typename Func> void foreach_item(Func op);

  /**
   * Apply an operator to every item in the panel.
   * The items are visited in drawing order from top to bottom.
   * The operator should have the following signature:
   *
   *   bool MyOperator(const bNodeTreeInterfaceItem &item);
   *
   * If the operator returns false for any item the iteration stops.
   */
  template<typename Func> void foreach_item(Func op) const;

  void copy_impl(const bNodeTreeInterfacePanel &src);
  void free_impl();

 protected:
  void copy_items(blender::Span<const bNodeTreeInterfaceItem *> items_src);

  void add_item(bNodeTreeInterfaceItem &item);
  void insert_item(bNodeTreeInterfaceItem &item, int index);
  bool remove_item(bNodeTreeInterfaceItem &item, bool free);
  void clear_items();
  bool move_item(bNodeTreeInterfaceItem &item, int new_index);

  friend struct bNodeTreeInterface;
#endif
} bNodeTreeInterfacePanel;

typedef struct bNodeTreeInterface {
  bNodeTreeInterfacePanel root_panel;

  int active_index;
  int next_socket_uid;

#ifdef __cplusplus
  void copy_data(const bNodeTreeInterface &src);
  void free_data();

  bNodeTreeInterfaceItem *active_item();
  const bNodeTreeInterfaceItem *active_item() const;
  void active_item_set(bNodeTreeInterfaceItem *item);

  /**
   * Search for an item in the interface.
   * \return True if the item was found.
   */
  bool find_item(const bNodeTreeInterfaceItem &item) const
  {
    return root_panel.find_item(item);
  }
  /**
   * Search for an item and its parent in the interface.
   * \param r_parent: Parent containing the item.
   * \return True if the item was found in any panel.
   */
  bool find_item_parent(const bNodeTreeInterfaceItem &item, bNodeTreeInterfacePanel *&r_parent)
  {
    return root_panel.find_item_parent(item, r_parent);
  }

  /**
   * Add a new socket at the end of the items list.
   * \param parent: Panel in which to add the socket. If parent is null the socket is added in the
   * root panel.
   */
  bNodeTreeInterfaceSocket *add_socket(blender::StringRef name,
                                       blender::StringRef description,
                                       blender::StringRef data_type,
                                       eNodeTreeInterfaceSocketKind in_out,
                                       bNodeTreeInterfacePanel *parent);
  /**
   * Insert a new socket.
   * \param parent: Panel in which to add the socket. If parent is null the socket is added in the
   * root panel.
   * \param index: Position of the socket within the parent panel.
   */
  bNodeTreeInterfaceSocket *insert_socket(blender::StringRef name,
                                          blender::StringRef description,
                                          blender::StringRef data_type,
                                          eNodeTreeInterfaceSocketKind in_out,
                                          bNodeTreeInterfacePanel *parent,
                                          int index);

  /**
   * Add a new panel at the end of the items list.
   * \param parent: Panel in which the new panel is aded as a child. If parent is null the new
   * panel is made a child of the root panel.
   */
  bNodeTreeInterfacePanel *add_panel(blender::StringRef name, bNodeTreeInterfacePanel *parent);
  /**
   * Insert a new panel.
   * \param parent: Panel in which the new panel is aded as a child. If parent is null the new
   * panel is made a child of the root panel.
   * \param index: Position of the child panel within the parent panel.
   */
  bNodeTreeInterfacePanel *insert_panel(blender::StringRef name,
                                        bNodeTreeInterfacePanel *parent,
                                        int index);

  /**
   * Add a copy of an item at the end of the items list.
   * \param parent: Add the item inside the parent panel. If parent is null the item is made a
   * child of the root panel.
   */
  bNodeTreeInterfaceItem *add_item_copy(const bNodeTreeInterfaceItem &item,
                                        bNodeTreeInterfacePanel *parent);
  /**
   * Insert a copy of an item.
   * \param parent: Add the item inside the parent panel. If parent is null the item is made a
   * child of the root panel.
   * \param index: Position of the item within the parent panel.
   */
  bNodeTreeInterfaceItem *insert_item_copy(const bNodeTreeInterfaceItem &item,
                                           bNodeTreeInterfacePanel *parent,
                                           int index);

  bool remove_item(bNodeTreeInterfaceItem &item);
  void clear_items();

  /**
   * Move an item to a new position.
   * \param new_index: New index of the item in the parent panel.
   */
  bool move_item(bNodeTreeInterfaceItem &item, int new_index);
  /**
   * Move an item to a new panel and/or position.
   * \param new_parent: Panel that the item is moved to. If null the item is added to the root
   * panel.
   * \param new_index: New index of the item in the parent panel.
   */
  bool move_item_to_parent(bNodeTreeInterfaceItem &item,
                           bNodeTreeInterfacePanel *new_parent,
                           int new_index);

  /**
   * Apply an operator to every item in the interface.
   * The items are visited in drawing order from top to bottom.
   * The operator should have the following signature:
   *
   *   bool MyOperator(bNodeTreeInterfaceItem &item);
   *
   * If the operator returns false for any item the iteration stops.
   */
  template<typename Func> void foreach_item(Func op)
  {
    root_panel.foreach_item(op);
  }

  /**
   * Apply an operator to every item in the interface.
   * The items are visited in drawing order from top to bottom.
   * The operator should have the following signature:
   *
   *   bool MyOperator(const bNodeTreeInterfaceItem &item);
   *
   * If the operator returns false for any item the iteration stops.
   */
  template<typename Func> void foreach_item(Func op) const
  {
    root_panel.foreach_item(op);
  }

#endif
} bNodeTreeInterface;
