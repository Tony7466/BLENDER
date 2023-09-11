/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

namespace blender::nodes::materialx {

NodeItem LightPathNodeParser::compute()
{
  /* This node isn't supported by MaterialX. Only default values returned. */
  if (STREQ(socket_out_->name, "Is Camera Ray")) {
    return val(1.0f);
  }
  if (STREQ(socket_out_->name, "Ray Length")) {
    return val(1.0f);
  }
  return val(0.0f);
}

}  // namespace blender::nodes::materialx
