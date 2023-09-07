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
    vector = create_node("texcoord", "vector2");
  }
  vector = (vector * scale) % value(2.0f);
  NodeItem mix = (vector.extract(0).floor() + vector.extract(1).floor())
                     .if_else(NodeItem::CompareOp::Eq, value(1.0f), value(1.0f), value(0.0f));
  NodeItem res = create_node("mix", "color3");
  res.set_input("fg", color1);
  res.set_input("bg", color2);
  res.set_input("mix", mix);
  return res;
}

}  // namespace blender::nodes::materialx
