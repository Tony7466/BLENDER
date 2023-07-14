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
#endif
} bNodeTreeInterfaceItem;

/* Socket interface flags */
typedef enum eNodeTreeInterfaceSocketFlag {
  NODE_INTERFACE_SOCKET_INPUT = 1 << 0,
  NODE_INTERFACE_SOCKET_OUTPUT = 1 << 1,
  NODE_INTERFACE_SOCKET_HIDE_VALUE = 1 << 2,
  NODE_INTERFACE_SOCKET_HIDE_IN_MODIFIER = 1 << 3,
} eNodeTreeInterfaceSocketFlag;
ENUM_OPERATORS(eNodeTreeInterfaceSocketFlag, NODE_INTERFACE_SOCKET_HIDE_IN_MODIFIER);

typedef struct bNodeTreeInterfaceSocket {
  bNodeTreeInterfaceItem item;

  char *name;
  char *description;
  char *socket_type;
  /* eNodeTreeInterfaceSocketFlag */
  int flag;

  /* eAttrDomain */
  int attribute_domain;
  char *default_attribute_name;

  /* Unique id for constructing socket identifiers. */
  int uid;
  char _pad[4];

#ifdef __cplusplus
  std::string socket_identifier() const;
  blender::ColorGeometry4f socket_color() const;
#endif
} bNodeTreeInterfaceSocket;

typedef struct bNodeTreeInterfaceSocketFloat {
  bNodeTreeInterfaceSocket base;

  int subtype;
  float default_value;
  float min_value;
  float max_value;

#ifdef __cplusplus
  static const char *socket_type_static;
#endif
} bNodeTreeInterfaceSocketFloat;

typedef struct bNodeTreeInterfaceSocketInt {
  bNodeTreeInterfaceSocket base;

  int subtype;
  int default_value;
  int min_value;
  int max_value;

#ifdef __cplusplus
  static const char *socket_type_static;
#endif
} bNodeTreeInterfaceSocketInt;

typedef struct bNodeTreeInterfaceSocketBool {
  bNodeTreeInterfaceSocket base;

  int subtype;
  char default_value;
  char _pad[3];

#ifdef __cplusplus
  static const char *socket_type_static;
#endif
} bNodeTreeInterfaceSocketBool;

typedef struct bNodeTreeInterfaceSocketString {
  bNodeTreeInterfaceSocket base;

  int subtype;
  char _pad[4];
  char *default_value;

#ifdef __cplusplus
  static const char *socket_type_static;
#endif
} bNodeTreeInterfaceSocketString;

typedef struct bNodeTreeInterfaceSocketObject {
  bNodeTreeInterfaceSocket base;

  int subtype;
  char _pad[4];
  struct Object *default_value;

#ifdef __cplusplus
  static const char *socket_type_static;
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

  void copy_items(blender::Span<const bNodeTreeInterfaceItem *> items_src);
  void clear_items();

  void add_item(bNodeTreeInterfaceItem &item);
  void insert_item(bNodeTreeInterfaceItem &item, int index);
  bool remove_item(bNodeTreeInterfaceItem &item, bool free);
  bool move_item(bNodeTreeInterfaceItem &item, int new_index);

  /**
   * Apply an operator to every item in the panel.
   * The items are visited in drawing order from top to bottom.
   *
   * Handle all items:
   *   void MyOperator(bNodeTreeInterfaceItem &item);
   * Stops if the operator return false:
   *   bool MyOperator(bNodeTreeInterfaceItem &item);
   */
  template<typename Func> void foreach_item(Func op);

  /**
   * Apply an operator to every item in the panel.
   * The items are visited in drawing order from top to bottom.
   *
   * Handle all items:
   *   void MyOperator(bNodeTreeInterfaceItem &item);
   * Stops if the operator return false:
   *   bool MyOperator(bNodeTreeInterfaceItem &item);
   */
  template<typename Func> void foreach_item(Func op) const;
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
                                       blender::StringRef socket_type,
                                       eNodeTreeInterfaceSocketFlag flag,
                                       bNodeTreeInterfacePanel *parent);
  /**
   * Insert a new socket.
   * \param parent: Panel in which to add the socket. If parent is null the socket is added in the
   * root panel.
   * \param index: Position of the socket within the parent panel.
   */
  bNodeTreeInterfaceSocket *insert_socket(blender::StringRef name,
                                          blender::StringRef description,
                                          blender::StringRef socket_type,
                                          eNodeTreeInterfaceSocketFlag flag,
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
   *
   * Handle all items:
   *   void MyOperator(bNodeTreeInterfaceItem &item);
   * Stops if the operator return false:
   *   bool MyOperator(bNodeTreeInterfaceItem &item);
   */
  template<typename Func> void foreach_item(Func op)
  {
    root_panel.foreach_item(op);
  }

  /**
   * Apply an operator to every item in the interface.
   * The items are visited in drawing order from top to bottom.
   *
   * Handle all items:
   *   void MyOperator(bNodeTreeInterfaceItem &item);
   * Stops if the operator return false:
   *   bool MyOperator(bNodeTreeInterfaceItem &item);
   */
  template<typename Func> void foreach_item(Func op) const
  {
    root_panel.foreach_item(op);
  }

#endif
} bNodeTreeInterface;
