/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem HueSatValNodeParser::compute()
{
  /* TODO: implement fac, see do_hue_sat_fac in
   * source\blender\nodes\texture\nodes\node_texture_hueSatVal.cc */
  NodeItem hue = get_input_value("Hue", NodeItem::Type::Float);
  NodeItem saturation = get_input_value("Saturation", NodeItem::Type::Float);
  NodeItem value = get_input_value("Value", NodeItem::Type::Float);
  NodeItem fac = get_input_value("Fac", NodeItem::Type::Float);
  NodeItem color = get_input_value("Color", NodeItem::Type::Color3);

  /* Modifier to follow Cycles result */
  hue = hue - val(0.5f);

  NodeItem combine = create_node("combine3", NodeItem::Type::Vector3);
  combine.set_input("in1", hue);
  combine.set_input("in2", saturation);
  combine.set_input("in3", value);

  NodeItem res = create_node("hsvadjust", NodeItem::Type::Color3);
  res.set_input("in", color);
  res.set_input("amount", combine);
  return res;
}

}  // namespace blender::nodes::materialx
