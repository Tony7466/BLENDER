/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "node_item.h"

#include "DEG_depsgraph.h"
#include "DNA_material_types.h"
#include "DNA_node_types.h"

namespace blender::nodes::materialx {

class NodeParser {
 public:
  MaterialX::GraphElement *graph;
  const Depsgraph *depsgraph;
  const Material *material;
  const bNode *node;

 public:
  NodeParser(MaterialX::GraphElement *graph,
             const Depsgraph *depsgraph,
             const Material *material,
             const bNode *node);
  virtual ~NodeParser() = default;

  virtual NodeItem compute() = 0;

 protected:
  NodeItem create_node(const std::string &mx_category,
                       const std::string &mx_type,
                       bool accessory = false);
  NodeItem get_input_default(const std::string &name);
  NodeItem get_input_default(int index);
  NodeItem get_input_link(const std::string &name);
  NodeItem get_input_link(int index);
  NodeItem get_input_value(const std::string &name);
  NodeItem get_input_value(int index);
  NodeItem empty() const;
  template<class T> NodeItem value(const T &data) const;

 private:
  NodeItem get_input_default(const bNodeSocket &socket);
  NodeItem get_input_link(const bNodeSocket &socket);
  NodeItem get_input_value(const bNodeSocket &socket);
};

template<class T> NodeItem NodeParser::value(const T &data) const
{
  return empty().val(data);
}

#define DECLARE_PARSER(T) \
  class T : public NodeParser { \
   public: \
    using NodeParser::NodeParser; \
    NodeItem compute() override; \
  };

DECLARE_PARSER(BSDFPrincipledNodeParser)
DECLARE_PARSER(InvertNodeParser)
DECLARE_PARSER(MathNodeParser)
DECLARE_PARSER(MixRGBNodeParser)
DECLARE_PARSER(OutputMaterialNodeParser)
DECLARE_PARSER(TexImageNodeParser)

}  // namespace blender::nodes::materialx
