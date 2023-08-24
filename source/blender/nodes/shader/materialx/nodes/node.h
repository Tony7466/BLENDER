/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <MaterialXCore/Document.h>

#include "DEG_depsgraph.h"
#include "DNA_material_types.h"
#include "DNA_node_types.h"

namespace blender::nodes::materialx {

class MaterialXNode {
 public:
  const Depsgraph *depsgraph = nullptr;
  const Material *material = nullptr;
  const bNode *node = nullptr;
  MaterialX::NodePtr matx_node;
  MaterialX::DocumentPtr doc;

 public:
  MaterialXNode(MaterialX::DocumentPtr doc,
                const Depsgraph *depsgraph,
                const Material *material,
                const bNode *node);
  virtual ~MaterialXNode() = default;

  virtual MaterialX::NodePtr convert() = 0;
};

}  // namespace blender::nodes::materialx
