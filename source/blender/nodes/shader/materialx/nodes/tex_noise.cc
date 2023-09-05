/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem TexNoiseNodeParser::compute()
{
  NodeItem scale = get_input_value("Scale");
  NodeItem detail = get_input_value("Detail");
  NodeItem lacunarity = get_input_value("Lacunarity");

  if (detail.value && detail.type() == NodeItem::Type::Float) {
    detail = value(int(detail.value->asA<float>()));
  }

  NodeItem position = create_node("position", "vector3");
  position = position * scale;

  NodeItem res = create_node("fractal3d", "color3");
  res.set_input("position", position, NodeItem::Type::Vector3);
  res.set_input("octaves", detail);
  res.set_input("lacunarity", lacunarity);
  return res;
}

}  // namespace blender::nodes::materialx
