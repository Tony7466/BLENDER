/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "../material.h"
#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem SeparateColorNodeParser::compute()
{
  int mode = static_cast<NodeCombSepColor *>(node_->storage)->mode;
  NodeItem color = get_input_value("Color", NodeItem::Type::Color3);

  NodeItem convert = empty();

  switch (mode) {
    case NODE_COMBSEP_COLOR_RGB:
      break;
    case NODE_COMBSEP_COLOR_HSV:
      convert = create_node("rgbtohsv", "color3");
      convert.set_input("in", color);
      break;
    case NODE_COMBSEP_COLOR_HSL:
      CLOG_WARN(LOG_MATERIALX_SHADER, "Unsupported color model, using HSV instead: %d", mode);
      convert = create_node("rgbtohsv", "color3");
      convert.set_input("in", color);
      break;
    default:
      BLI_assert_unreachable();
  }

  int index = STREQ(socket_out_->name, "Red") ? 0 : STREQ(socket_out_->name, "Green") ? 1 : 2;

  NodeItem res = convert ? convert : color;
  return res.extract(index);
}

NodeItem CombineColorNodeParser::compute()
{
  int mode = static_cast<NodeCombSepColor *>(node_->storage)->mode;
  NodeItem red = get_input_value("Red", NodeItem::Type::Float);
  NodeItem green = get_input_value("Green", NodeItem::Type::Float);
  NodeItem blue = get_input_value("Blue", NodeItem::Type::Float);

  NodeItem convert = empty();
  NodeItem combine = create_node("combine3", "color3");
  combine.set_input("in1", red);
  combine.set_input("in2", green);
  combine.set_input("in3", blue);

  switch (mode) {
    case NODE_COMBSEP_COLOR_RGB:
      break;
    case NODE_COMBSEP_COLOR_HSV:
      convert = create_node("hsvtorgb", "color3");
      convert.set_input("in", combine);
      break;
    case NODE_COMBSEP_COLOR_HSL:
      CLOG_WARN(LOG_MATERIALX_SHADER, "Unsupported color model, using HSV instead: %d", mode);
      convert = create_node("hsvtorgb", "color3");
      convert.set_input("in", combine);
      break;
    default:
      BLI_assert_unreachable();
  }

  NodeItem res = convert ? convert : combine;
  return res;
}

}  // namespace blender::nodes::materialx
