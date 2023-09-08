/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem BlackbodyNodeParser::compute()
{
  /* This node doesn't have an implementation in MaterialX 1.38.6.
   * It's added in MaterialX 1.38.8. Uncomment this code after switching to 1.38.8.
   *
   * NodeItem temperature = get_input_value("Temperature", NodeItem::Type::Float);

   * NodeItem res = create_node("blackbody", NodeItem::Type::Color3);
   * res.set_input("temperature", temperature);
   * return res; */
  return empty();
}

}  // namespace blender::nodes::materialx
