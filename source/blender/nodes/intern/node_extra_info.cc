/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#include "NOD_node_extra_info.hh"

namespace blender::nodes {

NodeTooltipTextLine NodeTooltipTextLine::empty_space()
{
  NodeTooltipTextLine empty_line;
  empty_line.type = NodeTooltipTextLine::Type::Space;
  return empty_line;
}

}  // namespace blender::nodes
