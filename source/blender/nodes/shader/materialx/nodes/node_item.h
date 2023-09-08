/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <MaterialXCore/Node.h>

namespace blender::nodes::materialx {

class NodeItem {
 public:
  enum class Type {
    Any = 0,
    Empty,

    /* Value types */
    String,
    Filename,
    Boolean,
    Integer,
    /* Block of arithmetic types. Ordered by type cast */
    Float,
    Vector2,
    Vector3,
    Color3,
    Vector4,
    Color4,
    /* End of arithmetic types */

    /* Shader types
     * NOTE: There are only supported types */
    BSDF,
    EDF,
    SurfaceShader,
    Material,
  };
  enum class CompareOp { Less = 0, LessEq, Eq, GreaterEq, Greater, NotEq };

 public:
  MaterialX::ValuePtr value;
  MaterialX::NodePtr node;

 private:
  MaterialX::GraphElement *graph_;

 public:
  NodeItem(MaterialX::GraphElement *graph);
  ~NodeItem() = default;

  static Type type(const std::string &type_str);
  static std::string type(Type type);

  /* Operators */
  operator bool() const;
  NodeItem operator+(const NodeItem &other) const;
  NodeItem operator-(const NodeItem &other) const;
  NodeItem operator-() const;
  NodeItem operator*(const NodeItem &other) const;
  NodeItem operator/(const NodeItem &other) const;
  NodeItem operator%(const NodeItem &other) const;
  NodeItem operator^(const NodeItem &other) const;
  bool operator==(const NodeItem &other) const;
  bool operator!=(const NodeItem &other) const;

  /* Math functions */
  NodeItem abs() const;
  NodeItem floor() const;
  NodeItem ceil() const;
  NodeItem min(const NodeItem &other) const;
  NodeItem max(const NodeItem &other) const;
  NodeItem dotproduct(const NodeItem &other) const;
  NodeItem blend(const NodeItem &a, const NodeItem &b) const;
  NodeItem clamp(const NodeItem &min_val, const NodeItem &max_val) const;
  NodeItem clamp(float min_val = 0.0f, float max_val = 1.0f) const;
  NodeItem sin() const;
  NodeItem cos() const;
  NodeItem tan() const;
  NodeItem asin() const;
  NodeItem acos() const;
  NodeItem atan() const;
  NodeItem atan2(const NodeItem &other) const;
  NodeItem sinh() const;
  NodeItem cosh() const;
  NodeItem tanh() const;
  NodeItem ln() const;
  NodeItem sqrt() const;
  NodeItem sign() const;
  NodeItem exp() const;
  NodeItem convert(Type to_type) const;
  NodeItem if_else(CompareOp op,
                   const NodeItem &other,
                   const NodeItem &if_val,
                   const NodeItem &else_val) const;
  NodeItem extract(const int index) const;

  /* Useful functions */
  NodeItem empty() const;
  template<class T> NodeItem val(const T &data) const;
  Type type() const;
  NodeItem create_node(const std::string &category, NodeItem::Type type) const;

  /* Functions to set input and output */
  template<class T> void set_input(const std::string &in_name, const T &value, Type in_type);
  void set_input(const std::string &in_name, const NodeItem &item);
  void set_input_output(const std::string &in_name,
                        const NodeItem &item,
                        const std::string &out_name);
  void add_output(const std::string &in_name, Type out_type);

 private:
  static bool is_arithmetic(Type type);
  static Type cast_types(NodeItem &item1, NodeItem &item2);

  bool is_arithmetic() const;
  NodeItem arithmetic(const std::string &category, std::function<float(float)> func) const;
  NodeItem arithmetic(const NodeItem &other,
                      const std::string &category,
                      std::function<float(float, float)> func) const;
};

template<class T> NodeItem NodeItem::val(const T &data) const
{
  NodeItem res(graph_);
  res.value = MaterialX::Value::createValue<T>(data);
  return res;
}

template<class T>
void NodeItem::set_input(const std::string &in_name, const T &value, Type in_type)
{
  node->setInputValue(in_name, value, type(in_type));
}

}  // namespace blender::nodes::materialx
