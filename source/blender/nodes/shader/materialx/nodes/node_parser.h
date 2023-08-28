/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <MaterialXCore/Document.h>

#include "DEG_depsgraph.h"
#include "DNA_material_types.h"
#include "DNA_node_types.h"

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

  template<class T>
  void set_input(const std::string &name, const T &value, const std::string &mx_type);
  void set_input(const std::string &name, const NodeItem &item);
  void set_input(const std::string &name, const MaterialX::ValuePtr value);
  void set_input(const std::string &name, const MaterialX::NodePtr node);

  operator bool() const;

  NodeItem to_color3();
};

template<class T>
void NodeItem::set_input(const std::string &name, const T &value, const std::string &mx_type)
{
  node->setInputValue(name, value, mx_type);
}

class NodeParser {
 public:
  MaterialX::GraphElement *graph;
  const Depsgraph *depsgraph;
  const Material *material;
  const bNode *node;

 public:
  NodeParser(MaterialX::GraphElement *graph,
             const Depsgraph *depsgraph,
             const Material *material,
             const bNode *node);
  virtual ~NodeParser() = default;

  virtual NodeItem compute() = 0;

 protected:
  NodeItem create_node(const std::string &mx_category,
                       const std::string &mx_type,
                       bool accessory = false);
  NodeItem get_input_default(const std::string &name);
  NodeItem get_input_link(const std::string &name);
  NodeItem get_input_value(const std::string &name);
  NodeItem empty_value();
};

}  // namespace blender::nodes::materialx
