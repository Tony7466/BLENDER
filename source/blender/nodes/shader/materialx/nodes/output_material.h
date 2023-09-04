/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "node_parser.h"

namespace blender::nodes::materialx {

class OutputMaterialNodeParser : public NodeParser {
 public:
  OutputMaterialNodeParser(MaterialX::GraphElement *graph,
                           const Depsgraph *depsgraph,
                           const Material *material,
                           const bNode *node);
  NodeItem compute() override;
  NodeItem compute(const std::string &socket_name);
  NodeItem compute_default();
};

}  // namespace blender::nodes::materialx
