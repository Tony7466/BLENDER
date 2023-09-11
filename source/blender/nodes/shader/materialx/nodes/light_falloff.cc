/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem LightFalloffNodeParser::compute()
{
  NodeItem strength = get_input_value("Strength", NodeItem::Type::Float);
  NodeItem smooth = get_input_value("Smooth", NodeItem::Type::Float);

  /* This node isn't supported by MaterialX. This formula was given from OSL shader code in Cycles
   * node_light_falloff.osl. Considered ray_length=1.0f. */
  strength = strength * val(1.0f) / (smooth + val(1.0f));

  return strength;
}

}  // namespace blender::nodes::materialx
