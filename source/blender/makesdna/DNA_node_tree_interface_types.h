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
#  include "BLI_map.hh"
#  include "BLI_span.hh"
#  include "BLI_string_ref.hh"

#  include <memory>
#endif

struct bNodeTreeInterfacePanel;
struct bNodeTreeInterfaceSocket;

/** Type of interface item. */
typedef enum eNodeTreeInterfaceItemType {
  NODE_INTERFACE_SOCKET = 0,
  NODE_INTERFACE_PANEL = 1,
} eNodeTreeInterfaceItemType;

/** Describes a socket and all necessary details for a node declaration. */
typedef struct bNodeTreeInterfaceItem {
  /* eNodeTreeInterfaceItemType */
  char item_type;
  char _pad[3];

  /* Index in final item sequence. */
  int index;
  int children_start;
  int children_num;

  /* Panel in which to display the item. */
  struct bNodeTreeInterfacePanel *parent;

#ifdef __cplusplus
  template<typename T> T &get_as();
  template<typename T> const T &get_as() const;

  template<typename T> T *get_as_ptr();
  template<typename T> const T *get_as_ptr() const;

  using ParentMap = blender::Map<const bNodeTreeInterfacePanel *, bNodeTreeInterfacePanel *>;

  void copy_data(const bNodeTreeInterfaceItem &src, std::optional<ParentMap> parent_map);
  void free_data();

  template<typename OpT> void apply_typed_operator(OpT op) const;
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
#endif
} bNodeTreeInterfaceSocket;

typedef struct bNodeTreeInterfaceSocketFloat {
  bNodeTreeInterfaceSocket base;

#ifdef __cplusplus
  using ValueType = float;
  using SocketValueType = struct bNodeSocketValueFloat;

  static const char *socket_type_static;
#endif
} bNodeTreeInterfaceSocketFloat;

typedef struct bNodeTreeInterfaceSocketInt {
  bNodeTreeInterfaceSocket base;

#ifdef __cplusplus
  using ValueType = int;
  using SocketValueType = struct bNodeSocketValueInt;

  static const char *socket_type_static;
#endif
} bNodeTreeInterfaceSocketInt;

typedef struct bNodeTreeInterfaceSocketBool {
  bNodeTreeInterfaceSocket base;

#ifdef __cplusplus
  using ValueType = bool;
  using SocketValueType = struct bNodeSocketValueBoolean;

  static const char *socket_type_static;
#endif
} bNodeTreeInterfaceSocketBool;

typedef struct bNodeTreeInterfaceSocketString {
  bNodeTreeInterfaceSocket base;

#ifdef __cplusplus
  using ValueType = std::string;
  using SocketValueType = struct bNodeSocketValueString;

  static const char *socket_type_static;
#endif
} bNodeTreeInterfaceSocketString;

typedef struct bNodeTreeInterfaceSocketObject {
  bNodeTreeInterfaceSocket base;

#ifdef __cplusplus
  using ValueType = struct Object *;
  using SocketValueType = struct bNodeSocketValueObject;

  static const char *socket_type_static;
#endif
} bNodeTreeInterfaceSocketObject;

typedef struct bNodeTreeInterfacePanel {
  bNodeTreeInterfaceItem item;

  char *name;

#ifdef __cplusplus
  static bNodeTreeInterfaceSocket *create(blender::StringRef name);
#endif
} bNodeTreeInterfacePanel;

typedef struct bNodeTreeInterface {
  bNodeTreeInterfaceItem **items_array;
  int items_num;
  int active_index;
  int next_socket_uid;

  /* Root items range. */
  int root_items_num;

#ifdef __cplusplus
  void copy_data(const bNodeTreeInterface &src);
  void free_data();

  blender::Span<const bNodeTreeInterfaceItem *> items() const;
  blender::MutableSpan<bNodeTreeInterfaceItem *> items();
  int item_index(bNodeTreeInterfaceItem &item) const;

  blender::IndexRange root_items_range() const;
  blender::Span<const bNodeTreeInterfaceItem *> root_items() const;
  blender::MutableSpan<bNodeTreeInterfaceItem *> root_items();

  blender::IndexRange item_children_range(const bNodeTreeInterfaceItem &item) const;
  blender::Span<const bNodeTreeInterfaceItem *> item_children(
      const bNodeTreeInterfaceItem &item) const;
  blender::MutableSpan<bNodeTreeInterfaceItem *> item_children(const bNodeTreeInterfaceItem &item);

  bNodeTreeInterfaceItem *active_item();
  const bNodeTreeInterfaceItem *active_item() const;
  void active_item_set(bNodeTreeInterfaceItem *item);

  bool is_valid_parent(const bNodeTreeInterfaceItem &item,
                       const bNodeTreeInterfacePanel *new_parent) const;
  bool parent_set(bNodeTreeInterfaceItem &item, bNodeTreeInterfacePanel *new_parent);

  bNodeTreeInterfaceSocket *add_socket(blender::StringRef name,
                                       blender::StringRef description,
                                       blender::StringRef data_type,
                                       eNodeTreeInterfaceSocketKind in_out);
  bNodeTreeInterfaceSocket *insert_socket(blender::StringRef name,
                                          blender::StringRef description,
                                          blender::StringRef data_type,
                                          eNodeTreeInterfaceSocketKind in_out,
                                          int index);

  bNodeTreeInterfacePanel *add_panel(blender::StringRef name);
  bNodeTreeInterfacePanel *insert_panel(blender::StringRef name, int index);

  bNodeTreeInterfaceItem *add_item_copy(const bNodeTreeInterfaceItem &item);
  bNodeTreeInterfaceItem *insert_item_copy(const bNodeTreeInterfaceItem &item, int index);

  bool remove_item(bNodeTreeInterfaceItem &item);
  void clear_item_type(eNodeTreeInterfaceItemType type);
  bool move_item(bNodeTreeInterfaceItem &item, int new_index);

  void clear_items();

 protected:
  void copy_items(blender::Span<const bNodeTreeInterfaceItem *> items_src);

  void add_item(bNodeTreeInterfaceItem &item);
  void insert_item(bNodeTreeInterfaceItem &item, int index);
  bool unlink_item(bNodeTreeInterfaceItem &item);
  void free_item(bNodeTreeInterfaceItem &item);

  /**
   * Topologial stable sorting method that keeps items grouped by parent.
   * Direct children of a panel remain grouped together, so children can be access as a span.
   */
  void update_order();

#endif
} bNodeTreeInterface;
