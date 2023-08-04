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
#  include "BLI_function_ref.hh"
#  include "BLI_span.hh"
#  include "BLI_string_ref.hh"

#  include <memory>
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct bContext;
struct bNodeSocket;
struct bNodeSocketType;
struct bNodeTreeInterfaceItem;
struct bNodeTreeInterfacePanel;
struct bNodeTreeInterfaceSocket;
struct IDProperty;
struct LibraryForeachIDData;
struct PointerRNA;
struct uiLayout;

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
  /* String identifier for backwards compatibility, not used for new sockets. */
  char *uid_compat;

  void *socket_data;

  IDProperty *prop;

#ifdef __cplusplus
  std::string socket_identifier() const;
  bNodeSocketType *socket_typeinfo() const;
  blender::ColorGeometry4f socket_color() const;

  bool set_socket_type(const char *new_socket_type);

  void init_from_socket_instance(const bNodeSocket *socket);
#endif
} bNodeTreeInterfaceSocket;

typedef struct bNodeTreeInterfacePanel {
  bNodeTreeInterfaceItem item;

  char *name;

  bNodeTreeInterfaceItem **items_array;
  int items_num;

  /* Internal unique identifier for validating panel states. */
  int uid;

#ifdef __cplusplus
  blender::IndexRange items_range() const;
  blender::Span<const bNodeTreeInterfaceItem *> items() const;
  blender::MutableSpan<bNodeTreeInterfaceItem *> items();

  /**
   * Get the position of an item in this panel.
   * \return Position relative to the start of the panel items or -1 if the item is not in the
   * panel.
   */
  int item_position(const bNodeTreeInterfaceItem &item) const;
  /**
   * Check if the item is a direct child of the panel.
   */
  bool contains_item(const bNodeTreeInterfaceItem &item) const;

  /**
   * Search for an item in the interface.
   * \return True if the item was found.
   */
  bool find_item(const bNodeTreeInterfaceItem &item) const;
  /**
   * Get the index of the item in the interface.
   * \return Index if the item was found or -1 otherwise.
   */
  int find_item_index(const bNodeTreeInterfaceItem &item) const;
  /**
   * Get the item at the given index of the interface draw list.
   */
  const bNodeTreeInterfaceItem *get_item_at_index(int index) const;
  /**
   * Search for an item and its parent in the interface.
   * \param r_parent: Parent containing the item.
   * \return True if the item was found in any panel.
   */
  bool find_item_parent(const bNodeTreeInterfaceItem &item, bNodeTreeInterfacePanel *&r_parent);

  void copy_items(blender::Span<const bNodeTreeInterfaceItem *> items_src, int flag);
  void clear_items(bool do_id_user);

  void add_item(bNodeTreeInterfaceItem &item);
  void insert_item(bNodeTreeInterfaceItem &item, int position);
  bool remove_item(bNodeTreeInterfaceItem &item, bool free);
  bool move_item(bNodeTreeInterfaceItem &item, int new_position);

  /**
   * Apply a function to every item in the panel, including child panels.
   * \note: The items are visited in drawing order from top to bottom.
   *
   * \param fn: Function to execute for each item, iterations stops if false is returned.
   * \param include_root: Include the panel itself in the iteration.
   */
  void foreach_item(blender::FunctionRef<bool(bNodeTreeInterfaceItem &item)> fn,
                    bool include_self = false);
  /**
   * Apply a function to every item in the panel, including child panels.
   * \note: The items are visited in drawing order from top to bottom.
   *
   * \param fn: Function to execute for each item, iterations stops if false is returned.
   * \param include_root: Include the panel itself in the iteration.
   */
  void foreach_item(blender::FunctionRef<bool(const bNodeTreeInterfaceItem &item)> fn,
                    bool include_self = false) const;
#endif
} bNodeTreeInterfacePanel;

typedef struct bNodeTreeInterface {
  bNodeTreeInterfacePanel root_panel;

  int active_index;
  int next_uid;

#ifdef __cplusplus
  void copy_data(const bNodeTreeInterface &src, int flag);
  void free_data();

  bNodeTreeInterfaceItem *active_item();
  const bNodeTreeInterfaceItem *active_item() const;
  void active_item_set(bNodeTreeInterfaceItem *item);

  /**
   * Get the index of the item in the interface.
   * \return Index if the item was found or -1 otherwise.
   */
  int find_item_index(const bNodeTreeInterfaceItem &item) const
  {
    return root_panel.find_item_index(item);
  }
  /**
   * Search for an item in the interface.
   * \return True if the item was found.
   */
  bool find_item(const bNodeTreeInterfaceItem &item) const
  {
    return root_panel.find_item(item);
  }
  /**
   * Get the item at the given index of the interface draw list.
   */
  const bNodeTreeInterfaceItem *get_item_at_index(int index) const
  {
    return root_panel.get_item_at_index(index);
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
   * \param position: Position of the socket within the parent panel.
   */
  bNodeTreeInterfaceSocket *insert_socket(blender::StringRef name,
                                          blender::StringRef description,
                                          blender::StringRef socket_type,
                                          eNodeTreeInterfaceSocketFlag flag,
                                          bNodeTreeInterfacePanel *parent,
                                          int position);

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
   * \param position: Position of the child panel within the parent panel.
   */
  bNodeTreeInterfacePanel *insert_panel(blender::StringRef name,
                                        bNodeTreeInterfacePanel *parent,
                                        int position);

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
   * \param position: Position of the item within the parent panel.
   */
  bNodeTreeInterfaceItem *insert_item_copy(const bNodeTreeInterfaceItem &item,
                                           bNodeTreeInterfacePanel *parent,
                                           int position);

  bool remove_item(bNodeTreeInterfaceItem &item);
  void clear_items();

  /**
   * Move an item to a new position.
   * \param new_position: New position of the item in the parent panel.
   */
  bool move_item(bNodeTreeInterfaceItem &item, int new_position);
  /**
   * Move an item to a new panel and/or position.
   * \param new_parent: Panel that the item is moved to. If null the item is added to the root
   * panel.
   * \param new_position: New position of the item in the parent panel.
   */
  bool move_item_to_parent(bNodeTreeInterfaceItem &item,
                           bNodeTreeInterfacePanel *new_parent,
                           int new_position);

  /**
   * Apply a function to every item in the interface.
   * \note: The items are visited in drawing order from top to bottom.
   *
   * \param fn: Function to execute for each item, iterations stops if false is returned.
   * \param include_root: Include the root panel in the iteration.
   */
  void foreach_item(blender::FunctionRef<bool(bNodeTreeInterfaceItem &item)> fn,
                    bool include_root = false)
  {
    root_panel.foreach_item(fn, /*include_self=*/include_root);
  }
  /**
   * Apply a function to every item in the interface.
   * \note: The items are visited in drawing order from top to bottom.
   *
   * \param fn: Function to execute for each item, iterations stops if false is returned.
   * \param include_root: Include the root panel in the iteration.
   */
  void foreach_item(blender::FunctionRef<bool(const bNodeTreeInterfaceItem &item)> fn,
                    bool include_root = false) const
  {
    root_panel.foreach_item(fn, /*include_self=*/include_root);
  }

  void foreach_id(LibraryForeachIDData *cb);

#endif
} bNodeTreeInterface;

#ifdef __cplusplus
}
#endif
