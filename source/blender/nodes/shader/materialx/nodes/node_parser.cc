/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"
#include "../material.h"

#include "BKE_node_runtime.hh"

namespace blender::nodes::materialx {

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

std::string NodeParser::node_name(const bNode *node, const bNodeSocket *socket_out)
{
  return MaterialX::createValidName(node->output_sockets().size() <= 1 ?
                                        std::string(node->name) :
                                        std::string(node->name) + "_" + socket_out->name);
}

NodeItem NodeParser::create_node(const std::string &mx_category, const std::string &mx_type)
{
  NodeItem res = empty();
  res.node = graph_->addNode(mx_category, MaterialX::EMPTY_STRING, mx_type);
  return res;
}

NodeItem NodeParser::get_input_default(const std::string &name)
{
  return get_input_default(node_->input_by_identifier(name));
}

NodeItem NodeParser::get_input_default(int index)
{
  return get_input_default(node_->input_socket(index));
}

NodeItem NodeParser::get_input_link(const std::string &name)
{
  return get_input_link(node_->input_by_identifier(name));
}

NodeItem NodeParser::get_input_link(int index)
{
  return get_input_link(node_->input_socket(index));
}

NodeItem NodeParser::get_input_value(const std::string &name, const NodeItem::Type type)
{
  return get_input_value(node_->input_by_identifier(name), type);
}

NodeItem NodeParser::get_input_value(int index, const NodeItem::Type type)
{
  return get_input_value(node_->input_socket(index), type);
}

NodeItem NodeParser::empty() const
{
  return NodeItem(graph_);
}

NodeItem NodeParser::get_input_default(const bNodeSocket &socket)
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
  return res;
}

NodeItem NodeParser::get_input_link(const bNodeSocket &socket)
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

  /* Checking if node was already computed */
  res.node = graph_->getNode(node_name(from_node, link->fromsock));
  if (res.node) {
    return res;
  }

  /* Computing from_node with required NodeParser object */
#define CASE_NODE_TYPE(type, T) \
  case type: \
    res = T(graph_, depsgraph_, material_, from_node, link->fromsock).compute_full(); \
    break;

  switch (from_node->typeinfo->type) {
    CASE_NODE_TYPE(SH_NODE_BRIGHTCONTRAST, BrightContrastNodeParser)
    CASE_NODE_TYPE(SH_NODE_BSDF_PRINCIPLED, BSDFPrincipledNodeParser)
    CASE_NODE_TYPE(SH_NODE_COMBINE_COLOR, CombineColorNodeParser)
    CASE_NODE_TYPE(SH_NODE_COMBXYZ, CombineXYZNodeParser)
    CASE_NODE_TYPE(SH_NODE_HUE_SAT, HueSatValNodeParser)
    CASE_NODE_TYPE(SH_NODE_INVERT, InvertNodeParser)
    CASE_NODE_TYPE(SH_NODE_MATH, MathNodeParser)
    CASE_NODE_TYPE(SH_NODE_MIX_RGB_LEGACY, MixRGBNodeParser)
    CASE_NODE_TYPE(SH_NODE_NORMAL_MAP, NormalMapNodeParser)
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

  return res;
}

NodeItem NodeParser::get_input_value(const bNodeSocket &socket, const NodeItem::Type type)
{
  NodeItem res = get_input_link(socket);
  if (!res) {
    res = get_input_default(socket);
  }
  return type == NodeItem::Type::Empty ? res : res.convert(type);
}

NodeItem NodeParser::compute_full()
{
  CLOG_INFO(LOG_MATERIALX_SHADER, 1, "%s [%d]", node_->name, node_->typeinfo->type);
  NodeItem res = compute();
  if (res.node) {
    res.node->setName(node_name(node_, socket_out_));
  }
  return res;
}

}  // namespace blender::nodes::materialx
