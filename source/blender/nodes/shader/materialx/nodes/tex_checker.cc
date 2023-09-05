/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem TexCheckerNodeParser::compute()
{
  NodeItem vector = get_input_link("Vector");
  NodeItem color1 = get_input_value("Color1");
  NodeItem color2 = get_input_value("Color2");
  NodeItem scale = get_input_value("Scale");

  if (!vector) {
    vector = create_node("texcoord", "vector2");
  }
  vector = vector * scale;

  NodeItem separate = create_node("separate2", "multioutput");
  separate.set_input("in", vector, NodeItem::Type::Vector2);
  separate.add_output("outx", NodeItem::Type::Float);
  separate.add_output("outy", NodeItem::Type::Float);

  NodeItem modulo_x = create_node("modulo", "float");
  modulo_x.set_input("in1", separate, "outx");
  modulo_x.set_input("in2", value(2.0f));

  NodeItem modulo_y = create_node("modulo", "float");
  modulo_y.set_input("in1", separate, "outy");
  modulo_y.set_input("in2", value(2.0f));

  NodeItem ifequal = (modulo_x.floor() + modulo_y.floor())
                         .if_else(NodeItem::CompareOp::Eq, value(1.0f), value(0.0f), value(1.0f));

  NodeItem res = create_node("mix", "color3");
  res.set_input("bg", color1, NodeItem::Type::Color3);
  res.set_input("fg", color2, NodeItem::Type::Color3);
  res.set_input("mix", ifequal);
  return res;
}

}  // namespace blender::nodes::materialx
