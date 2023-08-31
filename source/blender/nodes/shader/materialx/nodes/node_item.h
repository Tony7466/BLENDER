/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <MaterialXCore/Node.h>

namespace blender::nodes::materialx {

class NodeItem {
 public:
  MaterialX::ValuePtr value;
  MaterialX::NodePtr node;

 private:
  MaterialX::GraphElement *graph_;

 public:
  NodeItem(MaterialX::GraphElement *graph);
  ~NodeItem() = default;

  NodeItem empty() const;
  template<class T> NodeItem val(const T &data) const;

  template<class T>
  void set_input(const std::string &name, const T &value, const std::string &mx_type);
  void set_input(const std::string &name, const NodeItem &item);
  void set_input(const std::string &name, const MaterialX::ValuePtr value);
  void set_input(const std::string &name, const MaterialX::NodePtr node);

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

  NodeItem abs() const;
  NodeItem floor() const;
  NodeItem ceil() const;
  NodeItem min(const NodeItem &other) const;
  NodeItem max(const NodeItem &other) const;
  NodeItem dotproduct(const NodeItem &other) const;
  NodeItem if_else(const std::string &condition,
                   const NodeItem &other,
                   const NodeItem &if_val,
                   const NodeItem &else_val) const;
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

  NodeItem to_color3() const;
  bool is_numeric() const;
  std::string type() const;

 private:
  NodeItem arithmetic(const std::string &mx_category, std::function<float(float)> func) const;
  NodeItem arithmetic(const NodeItem &other,
                      const std::string &mx_category,
                      std::function<float(float, float)> func) const;
  static MaterialX::ValuePtr float_to_type(float v, std::string mx_type);
  /* Functions for adjusting values to make equal types  */
  static bool adjust_types(MaterialX::ValuePtr &val1, MaterialX::ValuePtr &val2, std::string &mx_type);
  static bool adjust_types(NodeItem &val1, NodeItem &val2, std::string &mx_type);
};

template<class T> NodeItem NodeItem::val(const T &data) const
{
  NodeItem res(graph_);
  res.value = MaterialX::Value::createValue<T>(data);
  return res;
}

template<class T>
void NodeItem::set_input(const std::string &name, const T &value, const std::string &mx_type)
{
  node->setInputValue(name, value, mx_type);
}

}  // namespace blender::nodes::materialx
