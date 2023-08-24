/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "node.h"

namespace blender::nodes::materialx {

class MaterialXPrincipledBSDFNode : public MaterialXNode {
 protected:
  static const MaterialX::Color3 default_white_color_;

 public:
  MaterialXPrincipledBSDFNode(MaterialX::DocumentPtr doc,
                              const Depsgraph *depsgraph,
                              const Material *material,
                              const bNode *node);
  MaterialX::NodePtr convert() override;
};

}  // namespace blender::nodes::materialx
