/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem BSDFDiffuseNodeParser::compute()
{
  if (shader_type_ != NodeItem::Type::BSDF) {
    return empty();
  }

  NodeItem color = get_input_value("Color", NodeItem::Type::Color3);
  NodeItem roughness = get_input_value("Roughness", NodeItem::Type::Float);
  NodeItem normal = get_input_link("Normal", NodeItem::Type::Vector3);

  NodeItem res = create_node("oren_nayar_diffuse_bsdf", NodeItem::Type::BSDF);
  res.set_input("color", color);
  res.set_input("roughness", roughness);
  if (normal) {
    res.set_input("normal", normal);
  }
  return res;
}

}  // namespace blender::nodes::materialx
