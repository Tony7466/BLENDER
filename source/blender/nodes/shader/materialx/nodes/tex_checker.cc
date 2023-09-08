/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem TexCheckerNodeParser::compute()
{
  NodeItem vector = get_input_link("Vector", NodeItem::Type::Vector2);
  if (!vector) {
    vector = texcoord_node();
  }
  NodeItem value1 = val(1.0f);
  NodeItem value2 = val(0.0f);
  if (STREQ(socket_out_->name, "Color")) {
    value1 = get_input_value("Color1", NodeItem::Type::Color4);
    value2 = get_input_value("Color2", NodeItem::Type::Color4);
  }
  NodeItem scale = get_input_value("Scale", NodeItem::Type::Float);

  vector = (vector * scale) % val(2.0f);
  return (vector.extract(0).floor() + vector.extract(1).floor())
      .if_else(NodeItem::CompareOp::Eq, val(1.0f), value1, value2);
}

}  // namespace blender::nodes::materialx
