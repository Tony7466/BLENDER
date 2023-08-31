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
                       const bNode *node)
    : graph(graph), depsgraph(depsgraph), material(material), node(node)
{
}

NodeItem NodeParser::create_node(const std::string &mx_category,
                                 const std::string &mx_type,
                                 bool noname)
{
  NodeItem res = empty();
  res.node = graph->addNode(mx_category,
                            noname ? MaterialX::EMPTY_STRING :
                                     MaterialX::createValidName(node->name),
                            mx_type);
  return res;
}

NodeItem NodeParser::get_input_default(const std::string &name)
{
  return get_input_default(node->input_by_identifier(name));
}

NodeItem NodeParser::get_input_default(int index)
{
  return get_input_default(node->input_socket(index));
}

NodeItem NodeParser::get_input_link(const std::string &name)
{
  return get_input_link(node->input_by_identifier(name));
}

NodeItem NodeParser::get_input_link(int index)
{
  return get_input_link(node->input_socket(index));
}

NodeItem NodeParser::get_input_value(const std::string &name)
{
  return get_input_value(node->input_by_identifier(name));
}

NodeItem NodeParser::get_input_value(int index)
{
  return get_input_value(node->input_socket(index));
}

NodeItem NodeParser::empty() const
{
  return NodeItem(graph);
}

NodeItem NodeParser::get_input_default(const bNodeSocket &socket)
{
  NodeItem res = empty();
  switch (socket.type) {
    case SOCK_FLOAT: {
      float v = socket.default_value_typed<bNodeSocketValueFloat>()->value;
      res.value = MaterialX::Value::createValue<float>(v);
    } break;
    case SOCK_VECTOR: {
      const float *v = socket.default_value_typed<bNodeSocketValueVector>()->value;
      res.value = MaterialX::Value::createValue<MaterialX::Vector3>(
          MaterialX::Vector3(v[0], v[1], v[2]));
    } break;
    case SOCK_RGBA: {
      const float *v = socket.default_value_typed<bNodeSocketValueRGBA>()->value;
      res.value = MaterialX::Value::createValue<MaterialX::Color4>(
          MaterialX::Color4(v[0], v[1], v[2], v[3]));
    } break;
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

  const bNode *in_node = link->fromnode;

  /* Passing NODE_REROUTE nodes */
  while (in_node->type == NODE_REROUTE) {
    link = in_node->input_socket(0).link;
    if (!(link && link->is_used())) {
      return res;
    }
    in_node = link->fromnode;
  }

  /* Getting required NodeParser object */
  std::unique_ptr<NodeParser> parser;
  switch (in_node->typeinfo->type) {
    case SH_NODE_BSDF_PRINCIPLED:
      parser = std::make_unique<BSDFPrincipledNodeParser>(graph, depsgraph, material, in_node);
      break;
    case SH_NODE_INVERT:
      parser = std::make_unique<InvertNodeParser>(graph, depsgraph, material, in_node);
      break;
    case SH_NODE_MATH:
      parser = std::make_unique<MathNodeParser>(graph, depsgraph, material, in_node);
      break;
    case SH_NODE_MIX_RGB_LEGACY:
      parser = std::make_unique<MixRGBNodeParser>(graph, depsgraph, material, in_node);
      break;
    case SH_NODE_TEX_IMAGE:
      parser = std::make_unique<TexImageNodeParser>(graph, depsgraph, material, in_node);
      break;
    case SH_NODE_TEX_ENVIRONMENT:
      parser = std::make_unique<TexEnvironmentNodeParser>(graph, depsgraph, material, in_node);
      break;
    case SH_NODE_TEX_NOISE:
      parser = std::make_unique<TexNoiseNodeParser>(graph, depsgraph, material, in_node);
      break;
    case SH_NODE_TEX_CHECKER:
      parser = std::make_unique<TexCheckerNodeParser>(graph, depsgraph, material, in_node);
      break;
    default:
      CLOG_WARN(LOG_MATERIALX_SHADER, "Unsupported node: %s (%d)", in_node->name, in_node->type);
      return res;
  }

  res = parser->compute();
  return res;
}

NodeItem NodeParser::get_input_value(const bNodeSocket &socket)
{
  NodeItem res = get_input_link(socket);
  if (!res) {
    res = get_input_default(socket);
  }
  return res;
}

}  // namespace blender::nodes::materialx
