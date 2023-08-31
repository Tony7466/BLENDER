/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_item.h"
#include "../material.h"

#include "BLI_assert.h"
#include "BLI_utildefines.h"

namespace blender::nodes::materialx {

NodeItem::NodeItem(MaterialX::GraphElement *graph) : graph_(graph) {}

NodeItem NodeItem::empty() const
{
  return NodeItem(graph_);
}

void NodeItem::set_input(const std::string &name,
                         const NodeItem &item,
                         const std::string &output_name)
{
  if (item.value) {
    set_input(name, item.value);
  }
  else if (item.node) {
    set_input(name, item.node, output_name);
  }
  else {
    CLOG_WARN(LOG_MATERIALX_SHADER, "Empty item to input: %s", name.c_str());
  }
}

void NodeItem::set_input(const std::string &name, const MaterialX::ValuePtr value)
{
  std::string mx_type = value->getTypeString();
  if (mx_type == "float") {
    set_input(name, value->asA<float>(), mx_type);
  }
  else if (mx_type == "integer") {
    set_input(name, value->asA<int>(), mx_type);
  }
  else if (mx_type == "vector2") {
    set_input(name, value->asA<MaterialX::Vector2>(), mx_type);
  }
  else if (mx_type == "vector3") {
    set_input(name, value->asA<MaterialX::Vector3>(), mx_type);
  }
  else if (mx_type == "vector4") {
    set_input(name, value->asA<MaterialX::Vector4>(), mx_type);
  }
  else if (mx_type == "color3") {
    set_input(name, value->asA<MaterialX::Color3>(), mx_type);
  }
  else if (mx_type == "color4") {
    set_input(name, value->asA<MaterialX::Color4>(), mx_type);
  }
  else if (mx_type == "string") {
    set_input(name, value->asA<std::string>(), mx_type);
  }
  else {
    BLI_assert_unreachable();
  }
}

void NodeItem::set_input(const std::string &name,
                         const MaterialX::NodePtr node,
                         const std::string &output_name)
{
  this->node->setConnectedNode(name, node);
  if (output_name != "") {
    this->node->setConnectedOutput("in1", node->getOutput(output_name));
  }
}

void NodeItem::add_output(const std::string &name, const std::string &mx_type)
{
  node->addOutput(name, mx_type);
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

NodeItem NodeItem::operator-() const
{
  return val(0.0f) - *this;
}

NodeItem NodeItem::operator*(const NodeItem &other) const
{
  return arithmetic(other, "multiply", [](float a, float b) { return a * b; });
}

NodeItem NodeItem::operator/(const NodeItem &other) const
{
  return arithmetic(other, "divide", [](float a, float b) { return b == 0.0f ? 0.0f : a / b; });
}

NodeItem NodeItem::operator%(const NodeItem &other) const
{
  return arithmetic(
      other, "modulo", [](float a, float b) { return b == 0.0f ? 0.0f : std::fmodf(a, b); });
}

NodeItem NodeItem::operator^(const NodeItem &other) const
{
  return arithmetic(other, "power", [](float a, float b) { return std::powf(a, b); });
}

bool NodeItem::operator==(const NodeItem &other) const
{
  if (!*this) {
    return !other;
  }
  if (!other) {
    return !*this;
  }
  if (node && node == other.node) {
    return true;
  }
  if ((node && other.value) || (value && other.node)) {
    return false;
  }

  std::string mx_type;
  auto val1 = value;
  auto val2 = other.value;
  if (!adjust_types(val1, val2, mx_type)) {
    return false;
  }
  if (mx_type == "float") {
    return val1->asA<float>() == val2->asA<float>();
  }
  if (mx_type == "color3") {
    return val1->asA<MaterialX::Color3>() == val2->asA<MaterialX::Color3>();
  }
  if (mx_type == "color4") {
    return val1->asA<MaterialX::Color4>() == val2->asA<MaterialX::Color4>();
  }
  if (mx_type == "vector2") {
    return val1->asA<MaterialX::Vector2>() == val2->asA<MaterialX::Vector2>();
  }
  if (mx_type == "vector3") {
    return val1->asA<MaterialX::Vector3>() == val2->asA<MaterialX::Vector3>();
  }
  if (mx_type == "vector4") {
    return val1->asA<MaterialX::Vector4>() == val2->asA<MaterialX::Vector4>();
  }

  return false;
}

bool NodeItem::operator!=(const NodeItem &other) const
{
  return !(*this == other);
}

NodeItem NodeItem::abs() const
{
  return arithmetic("absval", [](float a) { return std::fabsf(a); });
}

NodeItem NodeItem::floor() const
{
  return arithmetic("floor", [](float a) { return std::floorf(a); });
}

NodeItem NodeItem::ceil() const
{
  return arithmetic("ceil", [](float a) { return std::ceilf(a); });
}

NodeItem NodeItem::min(const NodeItem &other) const
{
  return arithmetic(other, "min", [](float a, float b) { return std::min(a, b); });
}

NodeItem NodeItem::max(const NodeItem &other) const
{
  return arithmetic(other, "max", [](float a, float b) { return std::max(a, b); });
}

NodeItem NodeItem::dotproduct(const NodeItem &other) const
{
  NodeItem d = arithmetic(other, "dotproduct", [](float a, float b) { return a * b; });
  if (d.value) {
    std::string mx_type = d.type();
    float f = 0.0f;
    if (mx_type == "float") {
      f = value->asA<float>();
    }
    else if (mx_type == "color3") {
      auto v = value->asA<MaterialX::Color3>();
      f = v[0] + v[1] + v[2];
    }
    else if (mx_type == "color4") {
      auto v = value->asA<MaterialX::Color4>();
      f = v[0] + v[1] + v[2] + v[3];
    }
    else if (mx_type == "vector2") {
      auto v = value->asA<MaterialX::Vector2>();
      f = v[0] + v[1];
    }
    else if (mx_type == "vector3") {
      auto v = value->asA<MaterialX::Vector3>();
      f = v[0] + v[1] + v[2];
    }
    else if (mx_type == "vector4") {
      auto v = value->asA<MaterialX::Vector4>();
      f = v[0] + v[1] + v[2] + v[3];
    }
    else {
      BLI_assert_unreachable();
    }
    d.value = MaterialX::Value::createValue(f);
  }
  return d;
}

NodeItem NodeItem::if_else(const std::string &condition,
                           const NodeItem &other,
                           const NodeItem &if_val,
                           const NodeItem &else_val) const
{
  if (condition == "<") {
    return other.if_else(">", *this, else_val, if_val);
  }
  if (condition == "<=") {
    return other.if_else(">=", *this, else_val, if_val);
  }
  if (condition == "!=") {
    return if_else("==", other, else_val, if_val);
  }

  NodeItem res = empty();
  if (type() != "float" || other.type() != "float") {
    return res;
  }

  auto val1 = if_val;
  auto val2 = else_val;
  std::string mx_type;
  if (!adjust_types(val1, val2, mx_type)) {
    return res;
  }

  std::function<bool(float, float)> func = nullptr;
  std::string mx_category;
  if (condition == ">") {
    mx_category = "ifgreater";
    func = [](float a, float b) { return a > b; };
  }
  else if (condition == ">=") {
    mx_category = "ifgreatereq";
    func = [](float a, float b) { return a >= b; };
  }
  else if (condition == "==") {
    mx_category = "ifequal";
    func = [](float a, float b) { return a == b; };
  }
  else {
    BLI_assert_unreachable();
  }

  if (value && other.value) {
    res = func(value->asA<float>(), other.value->asA<float>()) ? val1 : val2;
  }
  else {
    res.node = graph_->addNode(mx_category, MaterialX::EMPTY_STRING, mx_type);
    res.set_input("value1", *this);
    res.set_input("value2", other);
    res.set_input("in1", val1);
    res.set_input("in2", val2);
  }

  return res;
}

NodeItem NodeItem::blend(const NodeItem &a, const NodeItem &b) const
{
  return (val(1.0f) - *this) * a + *this * b;
}

NodeItem NodeItem::clamp(const NodeItem &min_val, const NodeItem &max_val) const
{
  return min(max_val).max(min_val);
}

NodeItem NodeItem::clamp(float min_val, float max_val) const
{
  return clamp(val(min_val), val(max_val));
}

NodeItem NodeItem::sin() const
{
  return arithmetic("sin", [](float a) { return std::sinf(a); });
}

NodeItem NodeItem::cos() const
{
  return arithmetic("cos", [](float a) { return std::cosf(a); });
}

NodeItem NodeItem::tan() const
{
  return arithmetic("tan", [](float a) { return std::tanf(a); });
}

NodeItem NodeItem::asin() const
{
  return arithmetic("asin", [](float a) { return std::asinf(a); });
}

NodeItem NodeItem::acos() const
{
  return arithmetic("acos", [](float a) { return std::acosf(a); });
}

NodeItem NodeItem::atan() const
{
  return arithmetic("atan", [](float a) { return std::atanf(a); });
}

NodeItem NodeItem::atan2(const NodeItem &other) const
{
  return arithmetic(other, "atan2", [](float a, float b) { return std::atan2f(a, b); });
}

NodeItem NodeItem::sinh() const
{
  return (exp() - (-*this).exp()) / val(2.0f);
}

NodeItem NodeItem::cosh() const
{
  return (exp() - (-*this).exp()) / val(2.0f);
}

NodeItem NodeItem::tanh() const
{
  return sinh() / cosh();
}

NodeItem NodeItem::ln() const
{
  return arithmetic("ln", [](float a) { return std::logf(a); });
}

NodeItem NodeItem::sqrt() const
{
  return arithmetic("sqrt", [](float a) { return std::sqrtf(a); });
}

NodeItem NodeItem::sign() const
{
  return arithmetic("sign", [](float a) { return a < 0.0f ? -1.0f : (a == 0.0f ? 0.0f : 1.0f); });
}

NodeItem NodeItem::exp() const
{
  return arithmetic("exp", [](float a) { return std::expf(a); });
}

NodeItem NodeItem::to_color3() const
{
  std::string mx_type = type();
  NodeItem res = empty();
  if (value) {
    MaterialX::Color3 c;
    if (mx_type == "float") {
      float v = value->asA<float>();
      c = {v, v, v};
    }
    else if (mx_type == "color3") {
      auto v = value->asA<MaterialX::Color3>();
      c = {v[0], v[1], v[2]};
    }
    else if (mx_type == "color4") {
      auto v = value->asA<MaterialX::Color4>();
      c = {v[0], v[1], v[2]};
    }
    else if (mx_type == "vector3") {
      auto v = value->asA<MaterialX::Vector3>();
      c = {v[0], v[1], v[2]};
    }
    else if (mx_type == "vector4") {
      auto v = value->asA<MaterialX::Vector4>();
      c = {v[0], v[1], v[2]};
    }
    else {
      return res;
    }
    res.value = MaterialX::Value::createValue<MaterialX::Color3>(c);
  }
  else if (node) {
    if (mx_type != "color3") {
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

  std::string mx_type;
  if (value && other.value) {
    auto val1 = value;
    auto val2 = other.value;
    if (!adjust_types(val1, val2, mx_type)) {
      return res;
    }

    if (mx_type == "float") {
      float v1 = val1->asA<float>();
      float v2 = val2->asA<float>();
      res.value = MaterialX::Value::createValue<float>(func(v1, v2));
    }
    else if (mx_type == "color3") {
      auto v1 = val1->asA<MaterialX::Color3>();
      auto v2 = val2->asA<MaterialX::Color3>();
      res.value = MaterialX::Value::createValue<MaterialX::Color3>(
          {func(v1[0], v2[0]), func(v1[1], v2[1]), func(v1[2], v2[2])});
    }
    else if (mx_type == "color4") {
      auto v1 = val1->asA<MaterialX::Color4>();
      auto v2 = val2->asA<MaterialX::Color4>();
      res.value = MaterialX::Value::createValue<MaterialX::Color4>(
          {func(v1[0], v2[0]), func(v1[1], v2[1]), func(v1[2], v2[2]), func(v1[3], v2[3])});
    }
    else if (mx_type == "vector2") {
      auto v1 = val1->asA<MaterialX::Vector2>();
      auto v2 = val2->asA<MaterialX::Vector2>();
      res.value = MaterialX::Value::createValue<MaterialX::Vector2>(
          {func(v1[0], v2[0]), func(v1[1], v2[1])});
    }
    else if (mx_type == "vector3") {
      auto v1 = val1->asA<MaterialX::Vector3>();
      auto v2 = val2->asA<MaterialX::Vector3>();
      res.value = MaterialX::Value::createValue<MaterialX::Vector3>(
          {func(v1[0], v2[0]), func(v1[1], v2[1]), func(v1[2], v2[2])});
    }
    else if (mx_type == "vector4") {
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
    auto val1 = *this;
    auto val2 = other;
    if (!adjust_types(val1, val2, mx_type)) {
      return res;
    }

    res.node = graph_->addNode(mx_category, MaterialX::EMPTY_STRING, mx_type);
    res.set_input("in1", val1);
    res.set_input("in2", val2);
  }
  return res;
}

MaterialX::ValuePtr NodeItem::float_to_type(float v, std::string mx_type)
{
  if (mx_type == "float") {
    return MaterialX::Value::createValue<float>(v);
  }
  if (mx_type == "color3") {
    return MaterialX::Value::createValue<MaterialX::Color3>({v, v, v});
  }
  if (mx_type == "color4") {
    return MaterialX::Value::createValue<MaterialX::Color4>({v, v, v, 1.0f});
  }
  if (mx_type == "vector2") {
    return MaterialX::Value::createValue<MaterialX::Vector2>({v, v});
  }
  if (mx_type == "vector3") {
    return MaterialX::Value::createValue<MaterialX::Vector3>({v, v, v});
  }
  if (mx_type == "vector4") {
    return MaterialX::Value::createValue<MaterialX::Vector4>({v, v, v, 1.0f});
  }

  BLI_assert_unreachable();
  return nullptr;
}

bool NodeItem::adjust_types(MaterialX::ValuePtr &val1,
                            MaterialX::ValuePtr &val2,
                            std::string &mx_type)
{
  std::string t1 = val1->getTypeString();
  std::string t2 = val2->getTypeString();
  if (t1 != t2) {
    if (t1 == "float") {
      val1 = float_to_type(val1->asA<float>(), t2);
      mx_type = t2;
    }
    else if (t2 == "float") {
      val2 = float_to_type(val2->asA<float>(), t1);
      mx_type = t1;
    }
    else {
      return false;
    }
  }
  else {
    mx_type = t1;
  }
  return true;
}

bool NodeItem::adjust_types(NodeItem &val1, NodeItem &val2, std::string &mx_type)
{
  std::string t1 = val1.type();
  std::string t2 = val2.type();
  if (t1 != t2) {
    if (val1.value && t1 == "float") {
      val1.value = float_to_type(val1.value->asA<float>(), t2);
      mx_type = t2;
    }
    else if (val2.value && t2 == "float") {
      val2.value = float_to_type(val2.value->asA<float>(), t1);
      mx_type = t1;
    }
    else {
      return false;
    }
  }
  else {
    mx_type = t1;
  }
  return true;
}

}  // namespace blender::nodes::materialx
