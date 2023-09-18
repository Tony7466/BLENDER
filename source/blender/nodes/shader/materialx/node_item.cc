/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_item.h"

#include "BLI_assert.h"
#include "BLI_utildefines.h"

namespace blender::nodes::materialx {

NodeItem::NodeItem(MaterialX::GraphElement *graph) : graph_(graph) {}

NodeItem::Type NodeItem::type(const std::string &type_str)
{
  if (type_str == "multioutput") {
    return Type::Multioutput;
  }
  if (type_str == "string") {
    return Type::String;
  }
  if (type_str == "filename") {
    return Type::Filename;
  }
  if (type_str == "boolean") {
    return Type::Boolean;
  }
  if (type_str == "integer") {
    return Type::Integer;
  }
  if (type_str == "float") {
    return Type::Float;
  }
  if (type_str == "vector2") {
    return Type::Vector2;
  }
  if (type_str == "vector3") {
    return Type::Vector3;
  }
  if (type_str == "vector4") {
    return Type::Vector4;
  }
  if (type_str == "color3") {
    return Type::Color3;
  }
  if (type_str == "color4") {
    return Type::Color4;
  }
  if (type_str == "BSDF") {
    return Type::BSDF;
  }
  if (type_str == "EDF") {
    return Type::EDF;
  }
  if (type_str == "surfaceshader") {
    return Type::SurfaceShader;
  }
  if (type_str == "material") {
    return Type::Material;
  }
  BLI_assert_unreachable();
  return Type::Empty;
}

std::string NodeItem::type(Type type)
{
  switch (type) {
    case Type::Any:
      return "any";
    case Type::Multioutput:
      return "multioutput";
    case Type::String:
      return "string";
    case Type::Filename:
      return "filename";
    case Type::Boolean:
      return "boolean";
    case Type::Integer:
      return "integer";
    case Type::Float:
      return "float";
    case Type::Vector2:
      return "vector2";
    case Type::Vector3:
      return "vector3";
    case Type::Vector4:
      return "vector4";
    case Type::Color3:
      return "color3";
    case Type::Color4:
      return "color4";
    case Type::BSDF:
      return "BSDF";
    case Type::EDF:
      return "EDF";
    case Type::SurfaceShader:
      return "surfaceshader";
    case Type::Material:
      return "material";
    default:
      BLI_assert_unreachable();
  }
  return "";
}

bool NodeItem::is_arithmetic(Type type)
{
  return type >= Type::Float && type <= Type::Color4;
}

NodeItem::operator bool() const
{
  return value || node || input || output;
}

NodeItem NodeItem::operator+(const NodeItem &other) const
{
  Type type = this->type();
  if (ELEM(type, Type::BSDF, Type::EDF)) {
    /* Special case: add BSDF/EDF shaders */
    NodeItem res = empty();
    if (other.type() == type) {
      res = create_node("add", type);
      res.set_input("in1", *this);
      res.set_input("in2", other);
    }
    else {
      BLI_assert_unreachable();
    }
    return res;
  }

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
  Type type = this->type();
  if (ELEM(type, Type::BSDF, Type::EDF)) {
    /* Special case: multiple BSDF/EDF shader by Float or Color3 */
    NodeItem res = empty();
    Type other_type = other.type();
    if (ELEM(other_type, Type::Float, Type::Color3)) {
      res = create_node("multiply", type);
      res.set_input("in1", *this);
      res.set_input("in2", other);
    }
    else {
      BLI_assert_unreachable();
    }
    return res;
  }

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

  NodeItem item1 = *this;
  NodeItem item2 = other;
  Type to_type = cast_types(item1, item2);
  if (to_type == Type::Empty) {
    return false;
  }
  return item1.value->getValueString() == item2.value->getValueString();
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
    float f = 0.0f;
    switch (d.type()) {
      case Type::Float: {
        f = value->asA<float>();
        break;
      }
      case Type::Vector2: {
        auto v = value->asA<MaterialX::Vector2>();
        f = v[0] + v[1];
        break;
      }
      case Type::Vector3: {
        auto v = value->asA<MaterialX::Vector3>();
        f = v[0] + v[1] + v[2];
        break;
      }
      case Type::Vector4: {
        auto v = value->asA<MaterialX::Vector4>();
        f = v[0] + v[1] + v[2] + v[3];
        break;
      }
      case Type::Color3: {
        auto v = value->asA<MaterialX::Color3>();
        f = v[0] + v[1] + v[2];
        break;
      }
      case Type::Color4: {
        auto v = value->asA<MaterialX::Color4>();
        f = v[0] + v[1] + v[2] + v[3];
        break;
      }
      default:
        BLI_assert_unreachable();
    }
    d.value = MaterialX::Value::createValue(f);
  }
  return d;
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

NodeItem NodeItem::convert(Type to_type) const
{
  Type from_type = type();
  if (from_type == Type::Empty || from_type == to_type || to_type == Type::Any) {
    return *this;
  }
  if (!is_arithmetic(from_type) || !is_arithmetic(to_type)) {
    CLOG_WARN(LOG_MATERIALX_SHADER,
              "Cannot convert: %s -> %s",
              type(from_type).c_str(),
              type(to_type).c_str());
    return empty();
  }

  if (to_type == Type::Float) {
    return extract(0);
  }

  /* Converting types which requires > 1 iteration */
  switch (from_type) {
    case Type::Vector2:
      switch (to_type) {
        case Type::Vector4:
          return convert(Type::Vector3).convert(Type::Vector4);
        case Type::Color3:
          return convert(Type::Vector3).convert(Type::Color3);
        case Type::Color4:
          return convert(Type::Vector3).convert(Type::Color3).convert(Type::Color4);
        default:
          break;
      }
      break;
    case Type::Vector3:
      switch (to_type) {
        case Type::Color4:
          return convert(Type::Color3).convert(Type::Color4);
        default:
          break;
      }
      break;
    case Type::Vector4:
      switch (to_type) {
        case Type::Vector2:
          return convert(Type::Vector3).convert(Type::Vector2);
        case Type::Color3:
          return convert(Type::Vector3).convert(Type::Color3);
        default:
          break;
      }
      break;
    case Type::Color3:
      switch (to_type) {
        case Type::Vector2:
          return convert(Type::Vector3).convert(Type::Vector2);
        case Type::Vector4:
          return convert(Type::Vector3).convert(Type::Vector4);
        default:
          break;
      }
      break;
    case Type::Color4:
      switch (to_type) {
        case Type::Vector2:
          return convert(Type::Vector4).convert(Type::Vector3).convert(Type::Vector2);
        case Type::Vector3:
          return convert(Type::Vector4).convert(Type::Vector3);
        default:
          break;
      }
      break;
    default:
      break;
  }

  /* Converting 1 iteration types */
  NodeItem res = empty();
  if (value) {
    switch (from_type) {
      case Type::Float: {
        float v = value->asA<float>();
        switch (to_type) {
          case Type::Vector2:
            res.value = MaterialX::Value::createValue<MaterialX::Vector2>({v, v});
            break;
          case Type::Vector3:
            res.value = MaterialX::Value::createValue<MaterialX::Vector3>({v, v, v});
            break;
          case Type::Vector4:
            res.value = MaterialX::Value::createValue<MaterialX::Vector4>({v, v, v, 1.0f});
            break;
          case Type::Color3:
            res.value = MaterialX::Value::createValue<MaterialX::Color3>({v, v, v});
            break;
          case Type::Color4:
            res.value = MaterialX::Value::createValue<MaterialX::Color4>({v, v, v, 1.0f});
            break;
          default:
            BLI_assert_unreachable();
        }
        break;
      }
      case Type::Vector2: {
        auto v = value->asA<MaterialX::Vector2>();
        switch (to_type) {
          case Type::Vector3:
            res.value = MaterialX::Value::createValue<MaterialX::Vector3>({v[0], v[1], 0.0f});
            break;
          default:
            BLI_assert_unreachable();
        }
        break;
      }
      case Type::Vector3: {
        auto v = value->asA<MaterialX::Vector3>();
        switch (to_type) {
          case Type::Vector2:
            res.value = MaterialX::Value::createValue<MaterialX::Vector2>({v[0], v[1]});
            break;
          case Type::Vector4:
            res.value = MaterialX::Value::createValue<MaterialX::Vector4>(
                {v[0], v[1], v[2], 0.0f});
            break;
          case Type::Color3:
            res.value = MaterialX::Value::createValue<MaterialX::Color3>({v[0], v[1], v[2]});
            break;
          default:
            BLI_assert_unreachable();
        }
        break;
      }
      case Type::Vector4: {
        auto v = value->asA<MaterialX::Vector4>();
        switch (to_type) {
          case Type::Vector3:
            res.value = MaterialX::Value::createValue<MaterialX::Vector3>({v[0], v[1], v[2]});
            break;
          case Type::Color4:
            res.value = MaterialX::Value::createValue<MaterialX::Color4>({v[0], v[1], v[2], v[3]});
            break;
          default:
            BLI_assert_unreachable();
        }
        break;
      }
      case Type::Color3: {
        auto v = value->asA<MaterialX::Color3>();
        switch (to_type) {
          case Type::Vector3:
            res.value = MaterialX::Value::createValue<MaterialX::Vector3>({v[0], v[1], v[2]});
            break;
          case Type::Color4:
            res.value = MaterialX::Value::createValue<MaterialX::Color4>({v[0], v[1], v[2], 1.0f});
            break;
          default:
            BLI_assert_unreachable();
        }
        break;
      }
      case Type::Color4: {
        auto v = value->asA<MaterialX::Color4>();
        switch (to_type) {
          case Type::Vector4:
            res.value = MaterialX::Value::createValue<MaterialX::Vector4>(
                {v[0], v[1], v[2], v[3]});
            break;
          case Type::Color3:
            res.value = MaterialX::Value::createValue<MaterialX::Color3>({v[0], v[1], v[2]});
            break;
          default:
            BLI_assert_unreachable();
        }
        break;
      }
      default:
        BLI_assert_unreachable();
    }
  }
  else {
    res = create_node("convert", to_type);
    res.set_input("in", *this);
  }
  return res;
}

NodeItem NodeItem::if_else(CompareOp op,
                           const NodeItem &other,
                           const NodeItem &if_val,
                           const NodeItem &else_val) const
{
  switch (op) {
    case CompareOp::Less:
      return other.if_else(CompareOp::Greater, *this, else_val, if_val);
    case CompareOp::LessEq:
      return other.if_else(CompareOp::GreaterEq, *this, else_val, if_val);
    case CompareOp::NotEq:
      return if_else(CompareOp::Eq, other, else_val, if_val);
    default:
      break;
  }

  NodeItem res = empty();
  if (type() != Type::Float || other.type() != Type::Float) {
    return res;
  }

  auto item1 = if_val;
  auto item2 = else_val;
  Type to_type = cast_types(item1, item2);
  if (to_type == Type::Empty) {
    return res;
  }

  std::function<bool(float, float)> func = nullptr;
  std::string category;
  switch (op) {
    case CompareOp::Greater:
      category = "ifgreater";
      func = [](float a, float b) { return a > b; };
      break;
    case CompareOp::GreaterEq:
      category = "ifgreatereq";
      func = [](float a, float b) { return a >= b; };
      break;
    case CompareOp::Eq:
      category = "ifequal";
      func = [](float a, float b) { return a == b; };
      break;
    default:
      BLI_assert_unreachable();
  }

  if (value && other.value) {
    res = func(value->asA<float>(), other.value->asA<float>()) ? item1 : item2;
  }
  else {
    res = create_node(category, to_type);
    res.set_input("value1", *this);
    res.set_input("value2", other);
    res.set_input("in1", item1);
    res.set_input("in2", item2);
  }

  return res;
}

NodeItem NodeItem::extract(const int index) const
{
  /* TODO: Add check if (value) { ... } */
  NodeItem res = create_node("extract", Type::Float);
  res.set_input("in", *this);
  res.set_input("index", val(index));
  return res;
}

NodeItem NodeItem::empty() const
{
  return NodeItem(graph_);
}

NodeItem::Type NodeItem::type() const
{
  if (value) {
    return type(value->getTypeString());
  }
  if (node) {
    return type(node->getType());
  }
  if (output) {
    return type(output->getType());
  }
  return Type::Empty;
}

NodeItem NodeItem::create_node(const std::string &category, NodeItem::Type type) const
{
  std::string type_str = this->type(type);
  CLOG_INFO(LOG_MATERIALX_SHADER, 2, "<%s type=%s>", category.c_str(), type_str.c_str());
  NodeItem res = empty();
  res.node = graph_->addNode(category, MaterialX::EMPTY_STRING, type_str);
  return res;
}

void NodeItem::set_input(const std::string &in_name, const NodeItem &item)
{
  if (item.value) {
    Type item_type = item.type();
    switch (item_type) {
      case Type::String:
        set_input(in_name, item.value->asA<std::string>(), item_type);
        break;
      case Type::Boolean:
        set_input(in_name, item.value->asA<bool>(), item_type);
        break;
      case Type::Integer:
        set_input(in_name, item.value->asA<int>(), item_type);
        break;
      case Type::Float:
        set_input(in_name, item.value->asA<float>(), item_type);
        break;
      case Type::Vector2:
        set_input(in_name, item.value->asA<MaterialX::Vector2>(), item_type);
        break;
      case Type::Vector3:
        set_input(in_name, item.value->asA<MaterialX::Vector3>(), item_type);
        break;
      case Type::Vector4:
        set_input(in_name, item.value->asA<MaterialX::Vector4>(), item_type);
        break;
      case Type::Color3:
        set_input(in_name, item.value->asA<MaterialX::Color3>(), item_type);
        break;
      case Type::Color4:
        set_input(in_name, item.value->asA<MaterialX::Color4>(), item_type);
        break;
      default:
        BLI_assert_unreachable();
    }
  }
  else if (item.node) {
    node->setConnectedNode(in_name, item.node);
  }
  else if (item.input) {
    node->setAttribute("interfacename", item.input->getName());
  }
  else if (item.output) {
    node->setConnectedOutput(in_name, item.output);
  }
  else {
    CLOG_WARN(LOG_MATERIALX_SHADER, "Empty item to input: %s", in_name.c_str());
  }
}

NodeItem NodeItem::add_output(const std::string &out_name, Type out_type)
{
  NodeItem res = empty();
  res.output = node->addOutput(out_name, type(out_type));
  return res;
}

NodeItem NodeItem::create_input(const std::string &name, const NodeItem &item) const
{
  NodeItem res = empty();
  res.input = graph_->addInput(name);

  Type item_type = item.type();
  if (item.node) {
    res.input->setConnectedNode(item.node);
  }
  else {
    BLI_assert_unreachable();
  }
  res.input->setType(type(item_type));

  return res;
}

NodeItem NodeItem::create_output(const std::string &name, const NodeItem &item) const
{
  NodeItem res = empty();
  res.output = graph_->addOutput(name);

  Type item_type = item.type();
  if (item.node) {
    res.output->setConnectedNode(item.node);
  }
  else if (item.input) {
    res.output->setInterfaceName(item.input->getName());
  }
  else {
    BLI_assert_unreachable();
  }
  res.output->setType(type(item_type));

  return res;
}

NodeItem::Type NodeItem::cast_types(NodeItem &item1, NodeItem &item2)
{
  Type t1 = item1.type();
  Type t2 = item2.type();
  if (t1 == t2) {
    return t1;
  }
  if (!is_arithmetic(t1) || !is_arithmetic(t2)) {
    CLOG_WARN(
        LOG_MATERIALX_SHADER, "Can't adjust types: %s <-> %s", type(t1).c_str(), type(t2).c_str());
    return Type::Empty;
  }
  if (t1 < t2) {
    item1 = item1.convert(t2);
    return t2;
  }
  else {
    item2 = item2.convert(t1);
    return t1;
  }
}

bool NodeItem::is_arithmetic() const
{
  return is_arithmetic(type());
}

NodeItem NodeItem::arithmetic(const std::string &category, std::function<float(float)> func) const
{
  NodeItem res = empty();
  Type type = this->type();
  if (!is_arithmetic(type)) {
    return res;
  }

  if (value) {
    switch (type) {
      case Type::Float: {
        float v = value->asA<float>();
        res.value = MaterialX::Value::createValue<float>(func(v));
        break;
      }
      case Type::Color3: {
        auto v = value->asA<MaterialX::Color3>();
        res.value = MaterialX::Value::createValue<MaterialX::Color3>(
            {func(v[0]), func(v[1]), func(v[2])});
        break;
      }
      case Type::Color4: {
        auto v = value->asA<MaterialX::Color4>();
        res.value = MaterialX::Value::createValue<MaterialX::Color4>(
            {func(v[0]), func(v[1]), func(v[2]), func(v[3])});
        break;
      }
      case Type::Vector2: {
        auto v = value->asA<MaterialX::Vector2>();
        res.value = MaterialX::Value::createValue<MaterialX::Vector2>({func(v[0]), func(v[1])});
      }
      case Type::Vector3: {
        auto v = value->asA<MaterialX::Vector3>();
        res.value = MaterialX::Value::createValue<MaterialX::Vector3>(
            {func(v[0]), func(v[1]), func(v[2])});
        break;
      }
      case Type::Vector4: {
        auto v = value->asA<MaterialX::Vector4>();
        res.value = MaterialX::Value::createValue<MaterialX::Vector4>(
            {func(v[0]), func(v[1]), func(v[2]), func(v[3])});
        break;
      }
      default:
        BLI_assert_unreachable();
    }
  }
  else {
    NodeItem v = *this;
    if (ELEM(type, Type::Color3, Type::Color4) &&
        ELEM(category, "sin", "cos", "tan", "asin", "acos", "atan2", "sqrt", "ln", "exp"))
    {
      /* These functions haven't implementation in MaterialX, converting to Vector types */
      type = type == Type::Color3 ? Type::Vector3 : Type::Vector4;
      v = v.convert(type);
    }
    res = create_node(category, type);
    res.set_input("in", v);
  }
  return res;
}

NodeItem NodeItem::arithmetic(const NodeItem &other,
                              const std::string &category,
                              std::function<float(float, float)> func) const
{
  NodeItem res = empty();
  NodeItem item1 = *this;
  NodeItem item2 = other;
  Type to_type = cast_types(item1, item2);
  if (to_type == Type::Empty) {
    return res;
  }

  if (value && other.value) {
    switch (to_type) {
      case Type::Float: {
        float v1 = item1.value->asA<float>();
        float v2 = item2.value->asA<float>();
        res.value = MaterialX::Value::createValue<float>(func(v1, v2));
        break;
      }
      case Type::Color3: {
        auto v1 = item1.value->asA<MaterialX::Color3>();
        auto v2 = item2.value->asA<MaterialX::Color3>();
        res.value = MaterialX::Value::createValue<MaterialX::Color3>(
            {func(v1[0], v2[0]), func(v1[1], v2[1]), func(v1[2], v2[2])});
        break;
      }
      case Type::Color4: {
        auto v1 = item1.value->asA<MaterialX::Color4>();
        auto v2 = item2.value->asA<MaterialX::Color4>();
        res.value = MaterialX::Value::createValue<MaterialX::Color4>(
            {func(v1[0], v2[0]), func(v1[1], v2[1]), func(v1[2], v2[2]), func(v1[3], v2[3])});
        break;
      }
      case Type::Vector2: {
        auto v1 = item1.value->asA<MaterialX::Vector2>();
        auto v2 = item2.value->asA<MaterialX::Vector2>();
        res.value = MaterialX::Value::createValue<MaterialX::Vector2>(
            {func(v1[0], v2[0]), func(v1[1], v2[1])});
        break;
      }
      case Type::Vector3: {
        auto v1 = item1.value->asA<MaterialX::Vector3>();
        auto v2 = item2.value->asA<MaterialX::Vector3>();
        res.value = MaterialX::Value::createValue<MaterialX::Vector3>(
            {func(v1[0], v2[0]), func(v1[1], v2[1]), func(v1[2], v2[2])});
        break;
      }
      case Type::Vector4: {
        auto v1 = item1.value->asA<MaterialX::Vector4>();
        auto v2 = item2.value->asA<MaterialX::Vector4>();
        res.value = MaterialX::Value::createValue<MaterialX::Vector4>(
            {func(v1[0], v2[0]), func(v1[1], v2[1]), func(v1[2], v2[2]), func(v1[3], v2[3])});
        break;
      }
      default:
        BLI_assert_unreachable();
    }
  }
  else {
    res = create_node(category, to_type);
    res.set_input("in1", item1);
    res.set_input("in2", item2);
  }
  return res;
}

}  // namespace blender::nodes::materialx
