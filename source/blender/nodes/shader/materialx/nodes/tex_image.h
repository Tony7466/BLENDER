/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "node_parser.h"

namespace blender::nodes::materialx {

class TexImageNodeParser : public NodeParser {
 public:
  using NodeParser::NodeParser;
  NodeItem compute() override;
};

}  // namespace blender::nodes::materialx
