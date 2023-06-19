/* SPDX-FileCopyrightText: 2005 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "BLI_utildefines.h"

#include "DNA_node_types.h"

#ifdef __cplusplus
#  include <memory>
#endif

/** Describes a socket and all necessary details for a node declaration. */
typedef struct bNodeSocketDeclaration {
  char *name;
  char *description;
  /* eNodeSocketInOut */
  int in_out;
  char _pad[4];
} bNodeSocketDeclaration;

typedef struct bNodeTreeDeclaration {
  bNodeSocketDeclaration **sockets_array;
  int sockets_num;
  char _pad[4];

  bNodePanel **panels_array;
  int panels_num;
  char _pad2[4];

#ifdef __cplusplus
  blender::Span<const bNodeSocketDeclaration *> sockets() const;
  blender::MutableSpan<bNodeSocketDeclaration *> sockets_for_write();

  blender::Span<const bNodePanel *> panels() const;
  blender::MutableSpan<bNodePanel *> panels_for_write();
#endif
} bNodeTreeDeclaration;
