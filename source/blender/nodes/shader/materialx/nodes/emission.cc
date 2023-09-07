/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem EmissionNodeParser::compute()
{
  if (shader_type_ != NodeItem::Type::EDF) {
    return empty();
  }

  NodeItem color = get_input_value("Color", NodeItem::Type::Color3);
  NodeItem strength = get_input_value("Strength", NodeItem::Type::Float);

  NodeItem res = create_node("uniform_edf", "EDF");
  res.set_input("color", color * strength);
  return res;
}

}  // namespace blender::nodes::materialx
