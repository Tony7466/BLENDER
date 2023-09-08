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
 protected:
  MaterialX::GraphElement *graph_;
  const Depsgraph *depsgraph_;
  const Material *material_;
  const bNode *node_;
  const bNodeSocket *socket_out_;

 public:
  NodeParser(MaterialX::GraphElement *graph,
             const Depsgraph *depsgraph,
             const Material *material,
             const bNode *node,
             const bNodeSocket *socket_out);
  virtual ~NodeParser() = default;

  virtual NodeItem compute() = 0;

 protected:
  virtual NodeItem compute_full();
  virtual std::string node_name();
  NodeItem create_node(const std::string &category, NodeItem::Type type);
  NodeItem get_input_default(const std::string &name, NodeItem::Type to_type);
  NodeItem get_input_default(int index, NodeItem::Type to_type);
  NodeItem get_input_link(const std::string &name, NodeItem::Type to_type);
  NodeItem get_input_link(int index, NodeItem::Type to_type);
  NodeItem get_input_value(const std::string &name, NodeItem::Type to_type);
  NodeItem get_input_value(int index, NodeItem::Type to_type);
  NodeItem empty() const;
  template<class T> NodeItem val(const T &data) const;

 private:
  NodeItem get_input_default(const bNodeSocket &socket, NodeItem::Type to_type);
  NodeItem get_input_link(const bNodeSocket &socket, NodeItem::Type to_type);
  NodeItem get_input_value(const bNodeSocket &socket, NodeItem::Type to_type);
};

class ShaderNodeParser : public NodeParser {
 protected:
  NodeItem::Type shader_type_;

 public:
  ShaderNodeParser(MaterialX::GraphElement *graph,
                   const Depsgraph *depsgraph,
                   const Material *material,
                   const bNode *node,
                   const bNodeSocket *socket_out,
                   NodeItem::Type shader_type);

 protected:
  NodeItem compute_full() override;
  std::string node_name() override;
  NodeItem get_input_shader(const std::string &name, NodeItem::Type shader_type);
  NodeItem get_input_shader(int index, NodeItem::Type shader_type);

 private:
  NodeItem get_input_shader(const bNodeSocket &socket, NodeItem::Type shader_type);
};

template<class T> NodeItem NodeParser::val(const T &data) const
{
  return empty().val(data);
}

#define DECLARE_NODE_PARSER(T) \
  class T : public NodeParser { \
   public: \
    using NodeParser::NodeParser; \
    NodeItem compute() override; \
  };

#define DECLARE_SHADER_NODE_PARSER(T) \
  class T : public ShaderNodeParser { \
   public: \
    using ShaderNodeParser::ShaderNodeParser; \
    NodeItem compute() override; \
  };

DECLARE_NODE_PARSER(BlackbodyNodeParser)
DECLARE_NODE_PARSER(BrightContrastNodeParser)
DECLARE_NODE_PARSER(ClampNodeParser)
DECLARE_NODE_PARSER(CombineColorNodeParser)
DECLARE_NODE_PARSER(CombineXYZNodeParser)
DECLARE_NODE_PARSER(HueSatValNodeParser)
DECLARE_NODE_PARSER(InvertNodeParser)
DECLARE_NODE_PARSER(MapRangeNodeParser)
DECLARE_NODE_PARSER(MathNodeParser)
DECLARE_NODE_PARSER(MixRGBNodeParser)
DECLARE_NODE_PARSER(NormalMapNodeParser)
DECLARE_NODE_PARSER(RGBToBWNodeParser)
DECLARE_NODE_PARSER(SeparateColorNodeParser)
DECLARE_NODE_PARSER(SeparateXYZNodeParser)
DECLARE_NODE_PARSER(TexCheckerNodeParser)
DECLARE_NODE_PARSER(TexEnvironmentNodeParser)
DECLARE_NODE_PARSER(TexImageNodeParser)
DECLARE_NODE_PARSER(TexNoiseNodeParser)
DECLARE_NODE_PARSER(VectorMathNodeParser)

DECLARE_SHADER_NODE_PARSER(AddShaderNodeParser)
DECLARE_SHADER_NODE_PARSER(BSDFDiffuseNodeParser)
DECLARE_SHADER_NODE_PARSER(BSDFGlassNodeParser)
DECLARE_SHADER_NODE_PARSER(BSDFGlossyNodeParser)
DECLARE_SHADER_NODE_PARSER(BSDFPrincipledNodeParser)
DECLARE_SHADER_NODE_PARSER(BSDFRefractionNodeParser)
DECLARE_SHADER_NODE_PARSER(BSDFSheenNodeParser)
DECLARE_SHADER_NODE_PARSER(BSDFToonNodeParser)
DECLARE_SHADER_NODE_PARSER(BSDFTranslucentNodeParser)
DECLARE_SHADER_NODE_PARSER(BSDFTransparentNodeParser)
DECLARE_SHADER_NODE_PARSER(EmissionNodeParser)
DECLARE_SHADER_NODE_PARSER(MixShaderNodeParser)
DECLARE_SHADER_NODE_PARSER(SubsurfaceScatteringNodeParser)

}  // namespace blender::nodes::materialx
