/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem HueSatValNodeParser::compute()
{
  /* TODO: implement fac, see do_hue_sat_fac in
   * source\blender\nodes\texture\nodes\node_texture_hueSatVal.cc */
  NodeItem hue = get_input_value("Hue");
  NodeItem saturation = get_input_value("Saturation");
  NodeItem val = get_input_value("Value");
  NodeItem fac = get_input_value("Fac");
  NodeItem color = get_input_value("Color");

  /* Modifier to follow Cycles result */
  hue = hue - value(0.5f);

  NodeItem combine = create_node("combine3", "vector3");
  combine.set_input("in1", hue);
  combine.set_input("in2", saturation);
  combine.set_input("in3", val);

  NodeItem res = create_node("hsvadjust", "color3");
  res.set_input("in", color, NodeItem::Type::Color3);
  res.set_input("amount", combine);
  return res;
}

}  // namespace blender::nodes::materialx
