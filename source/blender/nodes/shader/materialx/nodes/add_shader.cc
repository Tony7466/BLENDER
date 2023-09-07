/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem AddShaderNodeParser::compute()
{
  NodeItem res = empty();
  switch (shader_type_) {
    case NodeItem::Type::BSDF:
    case NodeItem::Type::EDF: {
      NodeItem shader1 = get_input_shader(0, shader_type_);
      NodeItem shader2 = get_input_shader(1, shader_type_);

      if (shader1 && !shader2) {
        res = shader1;
      }
      else if (!shader1 && shader2) {
        res = shader2;
      }
      else if (shader1 && shader2) {
        res = shader1 + shader2;
      }
      break;
    }
    case NodeItem::Type::SurfaceShader: {
      res = get_input_shader(0, shader_type_);
      if (!res) {
        res = get_input_shader(1, shader_type_);
      }
      break;
    }
    default:
      BLI_assert_unreachable();
  }
  return res;
}

}  // namespace blender::nodes::materialx
