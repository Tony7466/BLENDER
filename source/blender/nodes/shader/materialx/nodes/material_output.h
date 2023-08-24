/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "node.h"

namespace blender::nodes::materialx {

class MaterialXMaterialOutputNode : public MaterialXNode {
 public:
  MaterialXMaterialOutputNode(MaterialX::DocumentPtr doc,
                              const Depsgraph *depsgraph,
                              const Material *material,
                              const bNode *node);
  MaterialX::NodePtr convert() override;
};

}  // namespace blender::nodes::materialx
