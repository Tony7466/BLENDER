/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem GammaNodeParser::compute()
{
  NodeItem color = get_input_value("Color", NodeItem::Type::Color4);
  NodeItem gamma = get_input_value("Gamma", NodeItem::Type::Float);

  return color ^ gamma;
}

}  // namespace blender::nodes::materialx
