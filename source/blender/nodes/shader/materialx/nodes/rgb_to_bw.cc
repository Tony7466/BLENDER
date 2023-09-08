/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem RGBToBWNodeParser::compute()
{
  NodeItem color = get_input_value("Color", NodeItem::Type::Color4);

  NodeItem res = create_node("luminance", NodeItem::Type::Color4);
  res.set_input("in", color);
  return res;
}

}  // namespace blender::nodes::materialx
