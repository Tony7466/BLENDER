/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "../material.h"
#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem NormalMapNodeParser::compute()
{
  std::string default_space = "object";
  NodeShaderNormalMap *normal_map_node = static_cast<NodeShaderNormalMap *>(node_->storage);
  NodeItem color = get_input_value("Color");
  NodeItem strength = get_input_value("Strength");

  NodeItem res = create_node("normalmap", "vector3");
  res.set_input("in", color, NodeItem::Type::Color3);
  res.set_input("scale", strength);

  switch (normal_map_node->space) {
    case SHD_SPACE_TANGENT:
      res.set_input("space", value(std::string("tangent")));
      break;
    case SHD_SPACE_OBJECT:
      res.set_input("space", value(std::string("tangent")));
      break;
    default:
      res.set_input("space", value(default_space));
      CLOG_WARN(LOG_MATERIALX_SHADER,
                "Ignoring unsupported Space: %d %s (%d), %s will be used",
                normal_map_node->space,
                node_->name,
                node_->type,
                default_space);
  }
  return res;
}

}  // namespace blender::nodes::materialx
