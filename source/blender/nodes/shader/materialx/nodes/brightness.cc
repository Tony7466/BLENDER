/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem BrightContrastNodeParser::compute()
{
  NodeItem color = get_input_value("Color", NodeItem::Type::Color3);
  NodeItem bright = get_input_value("Bright", NodeItem::Type::Float);
  NodeItem contrast = get_input_value("Contrast", NodeItem::Type::Float);

  /* This formula was given from OSL shader code in Cycles. */
  return (bright + color * (contrast + value(1.0f)) - contrast * value(0.5f)).max(value(0.0f));
}

}  // namespace blender::nodes::materialx
