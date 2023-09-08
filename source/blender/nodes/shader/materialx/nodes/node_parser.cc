/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"
#include "../material.h"

#include "BKE_node_runtime.hh"

namespace blender::nodes::materialx {

static const std::string TEXCOORD_NODE_NAME = "node_texcoord";

NodeParser::NodeParser(MaterialX::GraphElement *graph,
                       const Depsgraph *depsgraph,
                       const Material *material,
                       const bNode *node,
                       const bNodeSocket *socket_out)
    : graph_(graph),
      depsgraph_(depsgraph),
      material_(material),
      node_(node),
      socket_out_(socket_out)
{
}

NodeItem NodeParser::compute_full()
{
  CLOG_INFO(LOG_MATERIALX_SHADER, 1, "%s [%d]", node_->name, node_->typeinfo->type);
  NodeItem res = compute();
  if (res.node) {
    res.node->setName(node_name());
  }
  return res;
}

std::string NodeParser::node_name()
{
  return MaterialX::createValidName(node_->output_sockets().size() <= 1 ?
                                        std::string(node_->name) :
                                        std::string(node_->name) + "_" + socket_out_->name);
}

NodeItem NodeParser::create_node(const std::string &category, NodeItem::Type type)
{
  return empty().create_node(category, type);
}

NodeItem NodeParser::get_input_default(const std::string &name, NodeItem::Type to_type)
{
  return get_input_default(node_->input_by_identifier(name), to_type);
}

NodeItem NodeParser::get_input_default(int index, NodeItem::Type to_type)
{
  return get_input_default(node_->input_socket(index), to_type);
}

NodeItem NodeParser::get_input_link(const std::string &name, NodeItem::Type to_type)
{
  return get_input_link(node_->input_by_identifier(name), to_type);
}

NodeItem NodeParser::get_input_link(int index, NodeItem::Type to_type)
{
  return get_input_link(node_->input_socket(index), to_type);
}

NodeItem NodeParser::get_input_value(const std::string &name, NodeItem::Type to_type)
{
  return get_input_value(node_->input_by_identifier(name), to_type);
}

NodeItem NodeParser::get_input_value(int index, NodeItem::Type to_type)
{
  return get_input_value(node_->input_socket(index), to_type);
}

NodeItem NodeParser::empty() const
{
  return NodeItem(graph_);
}

NodeItem NodeParser::texcoord_node()
{
  NodeItem res = empty();
  res.node = graph_->getNode(TEXCOORD_NODE_NAME);
  if (!res.node) {
    res = create_node("texcoord", NodeItem::Type::Vector2);
    res.node->setName(TEXCOORD_NODE_NAME);
  }
  return res;
}

NodeItem NodeParser::get_input_default(const bNodeSocket &socket, NodeItem::Type to_type)
{
  NodeItem res = empty();
  switch (socket.type) {
    case SOCK_FLOAT: {
      float v = socket.default_value_typed<bNodeSocketValueFloat>()->value;
      res.value = MaterialX::Value::createValue<float>(v);
      break;
    }
    case SOCK_VECTOR: {
      const float *v = socket.default_value_typed<bNodeSocketValueVector>()->value;
      res.value = MaterialX::Value::createValue<MaterialX::Vector3>(
          MaterialX::Vector3(v[0], v[1], v[2]));
      break;
    }
    case SOCK_RGBA: {
      const float *v = socket.default_value_typed<bNodeSocketValueRGBA>()->value;
      res.value = MaterialX::Value::createValue<MaterialX::Color4>(
          MaterialX::Color4(v[0], v[1], v[2], v[3]));
      break;
    }
    default: {
      CLOG_WARN(LOG_MATERIALX_SHADER, "Unsupported socket type: %d", socket.type);
    }
  }
  return res.convert(to_type);
}

NodeItem NodeParser::get_input_link(const bNodeSocket &socket, NodeItem::Type to_type)
{
  NodeItem res = empty();

  const bNodeLink *link = socket.link;
  if (!(link && link->is_used())) {
    return res;
  }

  const bNode *from_node = link->fromnode;

  /* Passing NODE_REROUTE nodes */
  while (from_node->type == NODE_REROUTE) {
    link = from_node->input_socket(0).link;
    if (!(link && link->is_used())) {
      return res;
    }
    from_node = link->fromnode;
  }

  /* Creating required NodeParser object */
  std::unique_ptr<NodeParser> parser;

#define CASE_NODE_TYPE(type, T) \
  case type: \
    parser = std::make_unique<T>(graph_, depsgraph_, material_, from_node, link->fromsock); \
    break;

  switch (from_node->typeinfo->type) {
    CASE_NODE_TYPE(SH_NODE_BLACKBODY, BlackbodyNodeParser)
    CASE_NODE_TYPE(SH_NODE_BRIGHTCONTRAST, BrightContrastNodeParser)
    CASE_NODE_TYPE(SH_NODE_CLAMP, ClampNodeParser)
    CASE_NODE_TYPE(SH_NODE_COMBINE_COLOR, CombineColorNodeParser)
    CASE_NODE_TYPE(SH_NODE_COMBXYZ, CombineXYZNodeParser)
    CASE_NODE_TYPE(SH_NODE_HUE_SAT, HueSatValNodeParser)
    CASE_NODE_TYPE(SH_NODE_INVERT, InvertNodeParser)
    CASE_NODE_TYPE(SH_NODE_MAP_RANGE, MapRangeNodeParser)
    CASE_NODE_TYPE(SH_NODE_MATH, MathNodeParser)
    CASE_NODE_TYPE(SH_NODE_MIX_RGB_LEGACY, MixRGBNodeParser)
    CASE_NODE_TYPE(SH_NODE_NORMAL_MAP, NormalMapNodeParser)
    CASE_NODE_TYPE(SH_NODE_RGBTOBW, RGBToBWNodeParser)
    CASE_NODE_TYPE(SH_NODE_SEPARATE_COLOR, SeparateColorNodeParser)
    CASE_NODE_TYPE(SH_NODE_SEPXYZ, SeparateXYZNodeParser)
    CASE_NODE_TYPE(SH_NODE_TEX_CHECKER, TexCheckerNodeParser)
    CASE_NODE_TYPE(SH_NODE_TEX_ENVIRONMENT, TexEnvironmentNodeParser)
    CASE_NODE_TYPE(SH_NODE_TEX_IMAGE, TexImageNodeParser)
    CASE_NODE_TYPE(SH_NODE_TEX_NOISE, TexNoiseNodeParser)
    CASE_NODE_TYPE(SH_NODE_VECTOR_MATH, VectorMathNodeParser)

    default:
      CLOG_WARN(LOG_MATERIALX_SHADER,
                "Unsupported node: %s [%d]",
                from_node->name,
                from_node->typeinfo->type);
  }
  if (!parser) {
    return res;
  }

  /* Checking if node was already computed */
  res.node = graph_->getNode(parser->node_name());
  if (res.node) {
    return res;
  }

  /* Computing */
  res = parser->compute_full();
  return res.convert(to_type);
}

NodeItem NodeParser::get_input_value(const bNodeSocket &socket, NodeItem::Type to_type)
{
  NodeItem res = get_input_link(socket, to_type);
  if (!res) {
    res = get_input_default(socket, to_type);
  }
  return res;
}

ShaderNodeParser::ShaderNodeParser(MaterialX::GraphElement *graph,
                                   const Depsgraph *depsgraph,
                                   const Material *material,
                                   const bNode *node,
                                   const bNodeSocket *socket_out,
                                   NodeItem::Type shader_type)
    : NodeParser(graph, depsgraph, material, node, socket_out), shader_type_(shader_type)
{
}

NodeItem ShaderNodeParser::compute_full()
{
  CLOG_INFO(LOG_MATERIALX_SHADER,
            1,
            "%s [%d] - %s",
            node_->name,
            node_->typeinfo->type,
            NodeItem::type(shader_type_).c_str());
  NodeItem res = compute();
  if (res.node) {
    res.node->setName(node_name());
  }
  return res;
}

std::string ShaderNodeParser::node_name()
{
  std::string name = NodeParser::node_name();
  if (shader_type_ != NodeItem::Type::SurfaceShader) {
    name += "_" + NodeItem::type(shader_type_);
  }
  return name;
}

NodeItem ShaderNodeParser::get_input_shader(const std::string &name, NodeItem::Type shader_type)
{
  return get_input_shader(node_->input_by_identifier(name), shader_type);
}

NodeItem ShaderNodeParser::get_input_shader(int index, NodeItem::Type shader_type)
{
  return get_input_shader(node_->input_socket(index), shader_type);
}

NodeItem ShaderNodeParser::get_input_shader(const bNodeSocket &socket, NodeItem::Type shader_type)
{
  NodeItem res = empty();

  const bNodeLink *link = socket.link;
  if (!(link && link->is_used())) {
    return res;
  }

  const bNode *from_node = link->fromnode;

  /* Passing NODE_REROUTE nodes */
  while (from_node->type == NODE_REROUTE) {
    link = from_node->input_socket(0).link;
    if (!(link && link->is_used())) {
      return res;
    }
    from_node = link->fromnode;
  }

  /* Creating required ShaderNodeParser object */
  std::unique_ptr<ShaderNodeParser> parser;

#define CASE_SHADER_NODE_TYPE(type, T) \
  case type: \
    parser = std::make_unique<T>( \
        graph_, depsgraph_, material_, from_node, link->fromsock, shader_type); \
    break;

  switch (from_node->typeinfo->type) {
    CASE_SHADER_NODE_TYPE(SH_NODE_ADD_SHADER, AddShaderNodeParser)
    CASE_SHADER_NODE_TYPE(SH_NODE_BSDF_DIFFUSE, BSDFDiffuseNodeParser)
    // CASE_SHADER_NODE_TYPE(SH_NODE_BSDF_GLASS, BSDFGlassNodeParser)
    // CASE_SHADER_NODE_TYPE(SH_NODE_BSDF_GLOSSY, BSDFGlossyNodeParser)
    CASE_SHADER_NODE_TYPE(SH_NODE_BSDF_PRINCIPLED, BSDFPrincipledNodeParser)
    // CASE_SHADER_NODE_TYPE(SH_NODE_BSDF_REFRACTION, BSDFRefractionNodeParser)
    // CASE_SHADER_NODE_TYPE(SH_NODE_BSDF_SHEEN, BSDFSheenNodeParser)
    // CASE_SHADER_NODE_TYPE(SH_NODE_BSDF_TOON, BSDFToonNodeParser)
    // CASE_SHADER_NODE_TYPE(SH_NODE_BSDF_TRANSLUCENT, BSDFTranslucentNodeParser)
    // CASE_SHADER_NODE_TYPE(SH_NODE_BSDF_TRANSPARENT, BSDFTransparentNodeParser)
    CASE_SHADER_NODE_TYPE(SH_NODE_EMISSION, EmissionNodeParser)
    CASE_SHADER_NODE_TYPE(SH_NODE_MIX_SHADER, MixShaderNodeParser)
      // CASE_SHADER_NODE_TYPE(SH_NODE_SUBSURFACE_SCATTERING, SubsurfaceScatteringNodeParser)

    default:
      CLOG_WARN(LOG_MATERIALX_SHADER,
                "Unsupported node: %s [%d]",
                from_node->name,
                from_node->typeinfo->type);
  }
  if (!parser) {
    return res;
  }

  /* Checking if node was already computed */
  res.node = graph_->getNode(parser->node_name());
  if (res.node) {
    return res;
  }

  /* Computing */
  res = parser->compute_full();
  return res;
}

}  // namespace blender::nodes::materialx
