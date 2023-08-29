/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_parser.h"

#include <BKE_node_runtime.hh>

namespace blender::nodes::materialx {

NodeItem::NodeItem(MaterialX::GraphElement *graph) : graph_(graph) {}

NodeItem NodeItem::empty() const
{
  return NodeItem(graph_);
}

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
  std::string mx_type = value->getTypeString();
  if (value->isA<float>()) {
    set_input(name, value->asA<float>(), mx_type);
  }
  else if (value->isA<MaterialX::Vector2>()) {
    set_input(name, value->asA<MaterialX::Vector2>(), mx_type);
  }
  else if (value->isA<MaterialX::Vector3>()) {
    set_input(name, value->asA<MaterialX::Vector3>(), mx_type);
  }
  else if (value->isA<MaterialX::Vector4>()) {
    set_input(name, value->asA<MaterialX::Vector4>(), mx_type);
  }
  else if (value->isA<MaterialX::Color3>()) {
    set_input(name, value->asA<MaterialX::Color3>(), mx_type);
  }
  else if (value->isA<MaterialX::Color4>()) {
    set_input(name, value->asA<MaterialX::Color4>(), mx_type);
  }
  else if (value->isA<std::string>()) {
    set_input(name, value->asA<std::string>(), mx_type);
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

NodeItem NodeItem::operator+(const NodeItem &other) const
{
  return arithmetic(other, "add", [](float a, float b) { return a + b; });
}

NodeItem NodeItem::operator-(const NodeItem &other) const
{
  return arithmetic(other, "subtract", [](float a, float b) { return a - b; });
}

NodeItem NodeItem::operator*(const NodeItem &other) const
{
  return arithmetic(other, "multiply", [](float a, float b) { return a * b; });
}

NodeItem NodeItem::operator/(const NodeItem &other) const
{
  return arithmetic(other, "divide", [](float a, float b) { return b == 0.0f ? 0.0f : a / b; });
}

bool NodeItem::operator==(const NodeItem &other) const
{
  if (node && node == other.node) {
    return true;
  }
  /* TODO: implement */
  return false;
}

NodeItem NodeItem::min(const NodeItem &other) const
{
  return arithmetic(other, "min", [](float a, float b) { return std::min(a, b); });
}

NodeItem NodeItem::max(const NodeItem &other) const
{
  return arithmetic(other, "max", [](float a, float b) { return std::max(a, b); });
}

NodeItem NodeItem::blend(const NodeItem &a, const NodeItem &b) const
{
  return (val(1.0f) - *this) * a + *this * b;
}

NodeItem NodeItem::to_color3() const
{
  std::string t = type();
  NodeItem res = empty();
  if (value) {
    MaterialX::Color3 c;
    if (t == "float") {
      float v = value->asA<float>();
      c = {v, v, v};
    }
    else if (t == "color3") {
      auto v = value->asA<MaterialX::Color3>();
      c = {v[0], v[1], v[2]};
    }
    else if (t == "color4") {
      auto v = value->asA<MaterialX::Color4>();
      c = {v[0], v[1], v[2]};
    }
    else if (t == "vector3") {
      auto v = value->asA<MaterialX::Vector3>();
      c = {v[0], v[1], v[2]};
    }
    else if (t == "vector4") {
      auto v = value->asA<MaterialX::Vector4>();
      c = {v[0], v[1], v[2]};
    }
    else {
      return res;
    }
    res.value = MaterialX::Value::createValue<MaterialX::Color3>(c);
  }
  else if (node) {
    if (t != "color3") {
      return res;
    }
    res.node = node;
  }
  return res;
}

bool NodeItem::is_numeric() const
{
  std::string t = type();
  return ELEM(t, "float", "color3", "color4", "vector2", "vector3", "vector4");
}

std::string NodeItem::type() const
{
  return value ? value->getTypeString() : node->getType();
}

NodeItem NodeItem::arithmetic(const std::string &mx_category,
                              std::function<float(float)> func) const
{
  if (!is_numeric()) {
    return empty();
  }

  std::string t = value ? value->getTypeString() : node->getType();
  NodeItem res(graph_);
  if (value) {
    if (t == "float") {
      float v = value->asA<float>();
      res.value = MaterialX::Value::createValue<float>(func(v));
    }
    else if (t == "color3") {
      auto v = value->asA<MaterialX::Color3>();
      res.value = MaterialX::Value::createValue<MaterialX::Color3>(
          {func(v[0]), func(v[1]), func(v[2])});
    }
    else if (t == "color4") {
      auto v = value->asA<MaterialX::Color4>();
      res.value = MaterialX::Value::createValue<MaterialX::Color4>(
          {func(v[0]), func(v[1]), func(v[2]), func(v[3])});
    }
    else if (t == "vector2") {
      auto v = value->asA<MaterialX::Vector2>();
      res.value = MaterialX::Value::createValue<MaterialX::Vector2>({func(v[0]), func(v[1])});
    }
    else if (t == "vector3") {
      auto v = value->asA<MaterialX::Vector3>();
      res.value = MaterialX::Value::createValue<MaterialX::Vector3>(
          {func(v[0]), func(v[1]), func(v[2])});
    }
    else if (t == "vector4") {
      auto v = value->asA<MaterialX::Vector4>();
      res.value = MaterialX::Value::createValue<MaterialX::Vector4>(
          {func(v[0]), func(v[1]), func(v[2]), func(v[3])});
    }
    else {
      BLI_assert_unreachable();
    }
  }
  else {
    res.node = graph_->addNode(mx_category, MaterialX::EMPTY_STRING, t);
    res.set_input("in", *this);
  }
  return res;
}

NodeItem NodeItem::arithmetic(const NodeItem &other,
                              const std::string &mx_category,
                              std::function<float(float, float)> func) const
{
  NodeItem res = empty();
  if (!is_numeric() || !other.is_numeric()) {
    return res;
  }

  std::string t1 = type();
  std::string t2 = other.type();

  if (value && other.value) {
    std::string t = t1;
    auto val1 = value;
    auto val2 = other.value;
    if (t1 != t2) {
      if (t1 == "float") {
        val1 = float_to_type(val1->asA<float>(), t2);
        t = t2;
      }
      else if (t2 == "float") {
        val2 = float_to_type(val2->asA<float>(), t1);
      }
      else {
        return res;
      }
    }

    if (t == "float") {
      float v1 = val1->asA<float>();
      float v2 = val2->asA<float>();
      res.value = MaterialX::Value::createValue<float>(func(v1, v2));
    }
    else if (t == "color3") {
      auto v1 = val1->asA<MaterialX::Color3>();
      auto v2 = val2->asA<MaterialX::Color3>();
      res.value = MaterialX::Value::createValue<MaterialX::Color3>(
          {func(v1[0], v2[0]), func(v1[1], v2[1]), func(v1[2], v2[2])});
    }
    else if (t == "color4") {
      auto v1 = val1->asA<MaterialX::Color4>();
      auto v2 = val2->asA<MaterialX::Color4>();
      res.value = MaterialX::Value::createValue<MaterialX::Color4>(
          {func(v1[0], v2[0]), func(v1[1], v2[1]), func(v1[2], v2[2]), func(v1[3], v2[3])});
    }
    else if (t == "vector2") {
      auto v1 = val1->asA<MaterialX::Vector2>();
      auto v2 = val2->asA<MaterialX::Vector2>();
      res.value = MaterialX::Value::createValue<MaterialX::Vector2>(
          {func(v1[0], v2[0]), func(v1[1], v2[1])});
    }
    else if (t == "vector3") {
      auto v1 = val1->asA<MaterialX::Vector3>();
      auto v2 = val2->asA<MaterialX::Vector3>();
      res.value = MaterialX::Value::createValue<MaterialX::Vector3>(
          {func(v1[0], v2[0]), func(v1[1], v2[1]), func(v1[2], v2[2])});
    }
    else if (t == "vector4") {
      auto v1 = val1->asA<MaterialX::Vector4>();
      auto v2 = val2->asA<MaterialX::Vector4>();
      res.value = MaterialX::Value::createValue<MaterialX::Vector4>(
          {func(v1[0], v2[0]), func(v1[1], v2[1]), func(v1[2], v2[2]), func(v1[3], v2[3])});
    }
    else {
      BLI_assert_unreachable();
    }
  }
  else {
    std::string t = t1;
    auto val1 = *this;
    auto val2 = other;
    if (t1 != t2) {
      if (val1.value && t1 == "float") {
        val1.value = float_to_type(val1.value->asA<float>(), t2);
        t = t2;
      }
      else if (val2.value && t2 == "float") {
        val2.value = float_to_type(val2.value->asA<float>(), t1);
      }
      else {
        return res;
      }
    }

    res.node = graph_->addNode(mx_category, MaterialX::EMPTY_STRING, t);
    res.set_input("in1", val1);
    res.set_input("in2", val2);
  }
  return res;
}

MaterialX::ValuePtr NodeItem::float_to_type(float v, std::string t) const
{
  if (t == "float") {
    return MaterialX::Value::createValue<float>(v);
  }
  if (t == "color3") {
    return MaterialX::Value::createValue<MaterialX::Color3>({v, v, v});
  }
  if (t == "color4") {
    return MaterialX::Value::createValue<MaterialX::Color4>({v, v, v, 1.0f});
  }
  if (t == "vector2") {
    return MaterialX::Value::createValue<MaterialX::Vector2>({v, v});
  }
  if (t == "vector3") {
    return MaterialX::Value::createValue<MaterialX::Vector3>({v, v, v});
  }
  if (t == "vector4") {
    return MaterialX::Value::createValue<MaterialX::Vector4>({v, v, v, 1.0f});
  }

  BLI_assert_unreachable();
  return nullptr;
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
