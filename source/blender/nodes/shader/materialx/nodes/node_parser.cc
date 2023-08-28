/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

#include "bsdf_principled.h"
#include "tex_image.h"

#include <BKE_node_runtime.hh>

namespace blender::nodes::materialx {

NodeItem::NodeItem(MaterialX::GraphElement *graph) : graph_(graph) {}

void NodeItem::set_input(const std::string &name, const NodeItem &item)
{
  if (item.value) {
    set_input(name, item.value);
  }
  else if (item.node) {
    set_input(name, item.node);
  }
}

void NodeItem::set_input(const std::string &name, const MaterialX::ValuePtr value)
{
  if (value->isA<float>()) {
    set_input(name, value->asA<float>(), "float");
  }
  else if (value->isA<MaterialX::Vector3>()) {
    set_input(name, value->asA<MaterialX::Vector3>(), "vector3");
  }
  else if (value->isA<MaterialX::Vector4>()) {
    set_input(name, value->asA<MaterialX::Vector4>(), "vector4");
  }
  else if (value->isA<MaterialX::Color3>()) {
    set_input(name, value->asA<MaterialX::Color3>(), "color3");
  }
  else if (value->isA<MaterialX::Color4>()) {
    set_input(name, value->asA<MaterialX::Color4>(), "color4");
  }
  else {
    BLI_assert_unreachable();
  }
}

void NodeItem::set_input(const std::string &name, const MaterialX::NodePtr node)
{
  this->node->setConnectedNode(name, node);
}

NodeItem::operator bool() const
{
  return value || node;
}

NodeItem NodeItem::to_color3()
{
  NodeItem res(graph_);
  if (value) {
    if (value->isA<float>()) {
      float v = value->asA<float>();
      res.value = MaterialX::Value::createValue<MaterialX::Color3>(MaterialX::Color3(v, v, v));
    }
    else if (value->isA<MaterialX::Color3>()) {
      res.value = value;
    }
    else if (value->isA<MaterialX::Color4>()) {
      auto c = value->asA<MaterialX::Color4>();
      res.value = MaterialX::Value::createValue<MaterialX::Color3>(
          MaterialX::Color3(c[0], c[1], c[2]));
    }
  }
  else if (node) {
    res.node = node;
  }
  return res;
}

NodeParser::NodeParser(MaterialX::GraphElement *graph,
                       const Depsgraph *depsgraph,
                       const Material *material,
                       const bNode *node)
    : graph(graph), depsgraph(depsgraph), material(material), node(node)
{
}

NodeItem NodeParser::create_node(const std::string &mx_category,
                                 const std::string &mx_type,
                                 bool accessory)
{
  NodeItem res = empty_value();
  res.node = graph->addNode(mx_category,
                            accessory ? MaterialX::EMPTY_STRING :
                                        MaterialX::createValidName(node->name),
                            mx_type);
  return res;
}

NodeItem NodeParser::get_input_default(const std::string &name)
{
  NodeItem res = empty_value();

  const bNodeSocket &socket = node->input_by_identifier(name);
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
      // TODO log warn
    }
  }
  return res;
}

NodeItem NodeParser::get_input_link(const std::string &name)
{
  NodeItem res = empty_value();

  const bNodeLink *link = node->input_by_identifier(name).link;
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
  switch (in_node->type) {
    case SH_NODE_BSDF_PRINCIPLED:
      parser = std::make_unique<BSDFPrincipledNodeParser>(graph, depsgraph, material, in_node);
      break;
    case SH_NODE_TEX_IMAGE:
      parser = std::make_unique<TexImageNodeParser>(graph, depsgraph, material, in_node);
      break;
    default:
      // TODO: warning log
      return res;
  }

  res = parser->compute();
  return res;
}

NodeItem NodeParser::get_input_value(const std::string &name)
{
  NodeItem res = get_input_link(name);
  if (!res) {
    res = get_input_default(name);
  }
  return res;
}

NodeItem NodeParser::empty_value()
{
  return NodeItem(graph);
}

}  // namespace blender::nodes::materialx
