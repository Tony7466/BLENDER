/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem TexCheckerNodeParser::compute()
{
  NodeItem vector = get_input_link("Vector", NodeItem::Type::Vector2);
  NodeItem color1 = get_input_value("Color1", NodeItem::Type::Color3);
  NodeItem color2 = get_input_value("Color2", NodeItem::Type::Color3);
  NodeItem scale = get_input_value("Scale", NodeItem::Type::Float);

  if (!vector) {
    vector = create_node("texcoord", NodeItem::Type::Vector2);
  }
  vector = (vector * scale) % val(2.0f);
  NodeItem mix = (vector.extract(0).floor() + vector.extract(1).floor())
                     .if_else(NodeItem::CompareOp::Eq, val(1.0f), val(1.0f), val(0.0f));
  NodeItem res = create_node("mix", NodeItem::Type::Color3);
  res.set_input("fg", color1);
  res.set_input("bg", color2);
  res.set_input("mix", mix);
  return res;
}

}  // namespace blender::nodes::materialx
