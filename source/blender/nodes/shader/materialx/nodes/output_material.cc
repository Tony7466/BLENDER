/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem OutputMaterialNodeParser::compute()
{
  NodeItem node = empty();
  NodeItem surface = get_input_link("Surface");
  if (surface) {
    node = create_node("surfacematerial", "material", false);
    node.set_input("surfaceshader", surface);
  }
  return node;
}

}  // namespace blender::nodes::materialx
