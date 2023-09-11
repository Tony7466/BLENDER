/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem CurvesRGBNodeParser::compute()
{
  /* TODO: implement */
  return get_input_value("Color", NodeItem::Type::Color4);
}

NodeItem CurvesFloatNodeParser::compute()
{
  /* TODO: implement */
  return get_input_value("Value", NodeItem::Type::Float);
}

}  // namespace blender::nodes::materialx
