/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "node_parser.h"

/* TODO: pxr::UsdMtlxRead() doesn't perform nodegraphs.
 * Uncomment USE_MATERIALX_NODEGRAPH after fixing it. */
//#define USE_MATERIALX_NODEGRAPH

namespace blender::nodes::materialx {

class GroupInputNodeParser;

class GroupNodeParser : public NodeParser {
  friend NodeParser;
  friend GroupInputNodeParser;

 public:
  using NodeParser::NodeParser;
  NodeItem compute() override;
  NodeItem compute_full() override;
};

class GroupOutputNodeParser : public NodeParser {
 public:
  using NodeParser::NodeParser;
  NodeItem compute() override;
  NodeItem compute_full() override;
};

class GroupInputNodeParser : public NodeParser {
 public:
  using NodeParser::NodeParser;
  NodeItem compute() override;
  NodeItem compute_full() override;
};

}  // namespace blender::nodes::materialx
