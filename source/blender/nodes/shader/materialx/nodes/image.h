/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "node.h"

namespace blender::nodes::materialx {

class MaterialXTexImageNode : public MaterialXNode {
 protected:
  /* Following Cycles color for wrong Texture nodes. */
  static const MaterialX::Color3 texture_error_color_;

 public:
  MaterialXTexImageNode(MaterialX::DocumentPtr doc,
                        const Depsgraph *depsgraph,
                        const Material *material,
                        const bNode *node);
  MaterialX::NodePtr convert() override;
};

}  // namespace blender::nodes::materialx
