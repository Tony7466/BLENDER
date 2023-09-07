/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "node_parser.h"

namespace blender::nodes::materialx {

class OutputMaterialNodeParser : public ShaderNodeParser {
 public:
  OutputMaterialNodeParser(MaterialX::GraphElement *graph,
                           const Depsgraph *depsgraph,
                           const Material *material,
                           const bNode *node);
  NodeItem compute() override;

  using ShaderNodeParser::compute_full;
  NodeItem compute_default();

 protected:
  std::string node_name() override;
};

}  // namespace blender::nodes::materialx
