/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem ClampNodeParser::compute()
{
  auto type = node_->custom1;
  NodeItem value = get_input_value("Value", NodeItem::Type::Float);
  NodeItem min = get_input_value("Min", NodeItem::Type::Float);
  NodeItem max = get_input_value("Max", NodeItem::Type::Float);

  NodeItem res = empty();
  if (type == NODE_CLAMP_RANGE) {
    res = min.if_else(
        NodeItem::CompareOp::Less, max, value.clamp(min, max), value.clamp(max, min));
  }
  else {
    res = value.clamp(min, max);
  }
  return res;
}

}  // namespace blender::nodes::materialx
