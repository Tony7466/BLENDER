/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_node_declaration_types.h"

#include "BLI_span.hh"

#ifdef __cplusplus

inline blender::Span<const bNodeSocketDeclaration *> bNodeTreeDeclaration::sockets() const
{
  return blender::Span(sockets_array, sockets_num);
}

inline blender::MutableSpan<bNodeSocketDeclaration *> bNodeTreeDeclaration::sockets()
{
  return blender::MutableSpan(sockets_array, sockets_num);
}

inline blender::Span<const bNodePanel *> bNodeTreeDeclaration::panels() const
{
  return blender::Span(panels_array, panels_num);
}

inline blender::MutableSpan<bNodePanel *> bNodeTreeDeclaration::panels()
{
  return blender::MutableSpan(panels_array, panels_num);
}

#endif
