/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem SeparateXYZNodeParser::compute()
{
  NodeItem vector = get_input_value("Vector", NodeItem::Type::Vector3);
  int index = STREQ(socket_out_->name, "X") ? 0 : STREQ(socket_out_->name, "Y") ? 1 : 2;
  return vector.extract(index);
}

NodeItem CombineXYZNodeParser::compute()
{
  NodeItem x = get_input_value("X", NodeItem::Type::Float);
  NodeItem y = get_input_value("Y", NodeItem::Type::Float);
  NodeItem z = get_input_value("Z", NodeItem::Type::Float);
  NodeItem res = create_node("combine3", NodeItem::Type::Vector3);
  res.set_input("in1", x);
  res.set_input("in2", y);
  res.set_input("in3", z);
  return res;
}

}  // namespace blender::nodes::materialx
