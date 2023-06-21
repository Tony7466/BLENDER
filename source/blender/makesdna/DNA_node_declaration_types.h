/* SPDX-FileCopyrightText: 2005 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "BLI_utildefines.h"

#ifdef __cplusplus
#  include "BLI_span.hh"
#  include "BLI_string_ref.hh"

#  include <memory>
#endif

struct bNodePanel;

/** Socket side (input/output). */
typedef enum eNodeSocketDeclarationInOut {
  SOCKDECL_IN = 1 << 0,
  SOCKDECL_OUT = 1 << 1,
} eNodeSocketDeclarationInOut;
ENUM_OPERATORS(eNodeSocketDeclarationInOut, SOCKDECL_OUT);

/** Describes a socket and all necessary details for a node declaration. */
typedef struct bNodeSocketDeclaration {
  char *name;
  char *description;
  /* eNodeSocketDeclarationInOut */
  int in_out;
  char _pad[4];

  /* Panel in which to display the socket. */
  struct bNodePanel *panel;
} bNodeSocketDeclaration;

typedef struct bNodeTreeInterface {
  bNodeSocketDeclaration **sockets_array;
  int sockets_num;
  char _pad[4];

  struct bNodePanel **panels_array;
  int panels_num;
  char _pad2[4];

#ifdef __cplusplus
  blender::Span<const bNodeSocketDeclaration *> sockets() const;
  blender::MutableSpan<bNodeSocketDeclaration *> sockets();

  int socket_index(bNodeSocketDeclaration &socket_decl) const;
  bNodeSocketDeclaration *add_socket(blender::StringRef name, eNodeSocketDeclarationInOut in_out);
  bNodeSocketDeclaration *insert_socket(blender::StringRef name,
                                        eNodeSocketDeclarationInOut in_out,
                                        int index);
  bool remove_socket(bNodeSocketDeclaration &socket_decl);
  void clear_sockets();
  bool move_socket(bNodeSocketDeclaration &socket_decl, int new_index);

  blender::Span<const bNodePanel *> panels() const;
  blender::MutableSpan<bNodePanel *> panels();

  int panel_index(bNodePanel &panel) const;
  bNodePanel *add_panel(blender::StringRef name);
  bNodePanel *insert_panel(blender::StringRef name, int index);
  bool remove_panel(bNodePanel &panel);
  void clear_panels();
  bool move_panel(bNodePanel &panel, int new_index);

 protected:
  void update_order();

 private:
  void update_panels_order();
  void update_sockets_order();

#endif
} bNodeTreeInterface;
