/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem TexNoiseNodeParser::compute()
{
  NodeItem scale = get_input_value("Scale", NodeItem::Type::Float);
  NodeItem detail = get_input_value("Detail", NodeItem::Type::Float);
  NodeItem lacunarity = get_input_value("Lacunarity", NodeItem::Type::Float);

  if (detail.value && detail.type() == NodeItem::Type::Float) {
    detail = val(int(detail.value->asA<float>()));
  }

  NodeItem position = create_node("position", NodeItem::Type::Vector3);
  position = position * scale;

  NodeItem res = create_node("fractal3d", NodeItem::Type::Color3);
  res.set_input("position", position);
  res.set_input("octaves", detail);
  res.set_input("lacunarity", lacunarity);
  return res;
}

}  // namespace blender::nodes::materialx
