/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem MixShaderNodeParser::compute()
{
  NodeItem res = empty();
  switch (shader_type_) {
    case NodeItem::Type::BSDF:
    case NodeItem::Type::EDF: {
      NodeItem fac = get_input_value(0, NodeItem::Type::Float);
      NodeItem shader1 = get_input_shader(1, shader_type_);
      NodeItem shader2 = get_input_shader(2, shader_type_);

      if (shader1 && !shader2) {
        res = shader1 * (value(1.0f) - fac);
      }
      else if (!shader1 && shader2) {
        res = shader2 * fac;
      }
      else if (shader1 && shader2) {
        res = create_node("mix", NodeItem::type(shader_type_));
        res.set_input("fg", shader1);
        res.set_input("bg", shader2);
        res.set_input("mix", fac);
      }
      break;
    }
    case NodeItem::Type::SurfaceShader: {
      res = get_input_shader(1, shader_type_);
      if (!res) {
        res = get_input_shader(2, shader_type_);
      }
      break;
    }
    default:
      BLI_assert_unreachable();
  }
  return res;
}

}  // namespace blender::nodes::materialx
