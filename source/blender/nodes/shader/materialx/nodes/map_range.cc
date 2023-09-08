/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem MapRangeNodeParser::compute()
{
  /* Interpolation isn't supported by MaterialX. */
  const NodeMapRange *map_range = static_cast<NodeMapRange *>(node_->storage);

  NodeItem::Type type;
  NodeItem value = empty();
  NodeItem from_min = empty();
  NodeItem from_max = empty();
  NodeItem to_min = empty();
  NodeItem to_max = empty();
  switch (map_range->data_type) {
    case CD_PROP_FLOAT:
      type = NodeItem::Type::Float;
      value = get_input_value("Value", type);
      from_min = get_input_value(1, type);
      from_max = get_input_value(2, type);
      to_min = get_input_value(3, type);
      to_max = get_input_value(4, type);
      break;
    case CD_PROP_FLOAT3:
      type = NodeItem::Type::Vector3;
      value = get_input_value("Vector", type);
      from_min = get_input_value(7, type);
      from_max = get_input_value(8, type);
      to_min = get_input_value(9, type);
      to_max = get_input_value(10, type);
      break;
    default:
      BLI_assert_unreachable();
  }

  NodeItem res = create_node("range", type);
  res.set_input("in", value);
  res.set_input("inlow", from_min);
  res.set_input("inhigh", from_max);
  res.set_input("outlow", to_min);
  res.set_input("outhigh", to_max);
  res.set_input("doclamp", val(bool(map_range->clamp)));
  return res;
}

}  // namespace blender::nodes::materialx
