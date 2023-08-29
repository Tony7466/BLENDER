/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem InvertNodeParser::compute()
{
  NodeItem fac = get_input_value("Fac");
  NodeItem color = get_input_value("Color");
  return fac.blend(color, fac.val(1.0f) - color);
}

}  // namespace blender::nodes::materialx
