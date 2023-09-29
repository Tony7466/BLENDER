/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 */

#include "DNA_node_types.h"

#include "BKE_node_runtime.hh"

#include "NOD_node_declaration.hh"

namespace blender::ed::space_node::connection {

class RealNode;

//#if (0)

class Context {
 public:
  bNodeTree *tree;
  std::unique_ptr<RealNode> target_node;
  int index;
  bool side_is_input;
};

class Node {
 public:
  virtual ~Node() = default;

  IndexRange index_range() const
  {
    return IndexRange(this->size());
  }

  std::optional<int> first_connection(const Context &context) const
  {
    for (const int index : this->index_range()) {
      if (this->is_connected(context, index)) {
        return index;
      }
    }
    return std::nullopt;
  }

  virtual int size(const Context &context) const = 0;

  virtual bool is_connected(const Context &context, int socket_index) const = 0;
  virtual eNodeSocketDatatype type(const Context &context, int socket_index) const = 0;

  virtual StringRef name(const Context &context, int socket_index) const = 0;
  virtual bool single_value(const Context &context, int socket_index) const = 0;

  //TODO: std::pair<bNode *, bNodeSocket *> to_connect(Context &context, int socket_index) const = 0;
};

using NodePtr = std::unique_ptr<Node>;

class RealNode : public Node {
 private:
  const bNode &node_;
 public:
  RealNode(const bNode &node) : node_(node) {}

  int size(const Context &context) const final
  {
    return int(context.side_is_input ? node_.input_sockets().size() : node_.output_sockets().size());
  }

  bool is_connected(const Context &context, const int socket_index) const override
  {
    BLI_assert(&node_ != &context.target_node->node_);
    const Span<const bNodeSocket *> sockets = context.side_is_input ? node_.input_sockets() : node_.output_sockets();
    for (const bNodeSocket *other_socket : sockets[socket_index]->logically_linked_sockets()) {
      if (&other_socket->owner_node() == &context.target_node->node_) {
        return true;
      }
    }
    return false;
  }

  eNodeSocketDatatype type(const Context &context, int socket_index) const override
  {
    const Span<const bNodeSocket *> sockets = context.side_is_input ? node_.input_sockets() : node_.output_sockets();
    return eNodeSocketDatatype(sockets[socket_index]->type);
  }

  StringRef name(const Context &context, int socket_index) const override
  {
    const Span<const bNodeSocket *> sockets = context.side_is_input ? node_.input_sockets() : node_.output_sockets();
    return sockets[socket_index]->name;
  }

  bool single_value(const Context &context, int socket_index) const override
  {
    BLI_assert(context.tree != nullptr);
    const Span<const bNodeSocket *> sockets = context.side_is_input ? node_.input_sockets() : node_.output_sockets();
    if (context.side_is_input) {
      return nodes::InputSocketFieldType::None == sockets[socket_index]->runtime->declaration->input_field_type;
    }
    return nodes::OutputSocketFieldType::None == sockets[socket_index]->runtime->declaration->output_field_dependency.field_type();
  }
};

class VirtualNode : public Node {
 public:
  bool is_connected(const Context &/*context*/, const int /*socket_index*/) const final
  {
    return false;
  }
};

class ViewerNode : public VirtualNode {
 private:
  const eNodeSocketDatatype data_type_;
 public:
  ViewerNode(const eNodeSocketDatatype data_type) : data_type_(data_type) {}

  int size(const Context &context) const final
  {
    return 2;
  }

  eNodeSocketDatatype type(const Context &/*context*/, int socket_index) const override
  {
    if (socket_index == 0) {
      return SOCK_GEOMETRY;
    }
    return data_type_;
  }

  StringRef name(const Context &/*context*/, int socket_index) const override
  {
    if (socket_index == 0) {
      return "Geometry";
    }
    return "Value";
  }

  bool single_value(const Context &/*context*/, int socket_index) const override
  {
    if (socket_index == 0) {
      return true;
    }
    return false;
  }
};

class GroupNode : public VirtualNode {
 private:
  const bNodeTree &group_;
 public:
  GroupNode(const bNodeTree &group) : group_(group)
  {
    group_.ensure_topology_cache();
    group_.ensure_interface_cache();
  }

  int size(const Context &context) const final
  {
    const Span<bNodeTreeInterfaceSocket *> sockets = context.side_is_input ? group_.interface_inputs() : group_.interface_outputs();
    return int(context.side_is_input ? group_.interface_inputs().size() : group_.interface_outputs().size());
  }

  eNodeSocketDatatype type(const Context &context, int socket_index) const override
  {
    const Span<bNodeTreeInterfaceSocket *> sockets = context.side_is_input ? group_.interface_inputs() : group_.interface_outputs();
    return eNodeSocketDatatype(sockets[socket_index]->socket_typeinfo()->type);
  }

  StringRef name(const Context &context, int socket_index) const override
  {
    const Span<bNodeTreeInterfaceSocket *> sockets = context.side_is_input ? group_.interface_inputs() : group_.interface_outputs();
    return sockets[socket_index]->name;
  }

  bool single_value(const Context &context, int socket_index) const override
  {
    const nodes::FieldInferencingInterface &interface = *group_.runtime->field_inferencing_interface;
    if (context.side_is_input) {
      return nodes::InputSocketFieldType::None == interface.inputs[socket_index];
    }
    return nodes::OutputSocketFieldType::None == interface.outputs[socket_index].field_type();
  }

  const bNodeTree &group() const
  {
    return group_;
  }
};

class ViewerGroupNode : public GroupNode {
 public:
  ViewerGroupNode(const bNodeTree &group) : GroupNode(group)
  {
    BLI_assert(this->group().is_viewer());
  }
};

/*
static void (const Context &context, const Span<NodePtr> nodes)
{
  const NodePtr target = context.target_node;
  for (const NodePtr &node : nodes) {
    std::optional<int> index = node->first_connection(context);
    if (!index.has_value()) {
      continue;
    }
    for (const int i : target->index_range()) {
      if (target->type(i) != node->type(*index)) {
        continue;
      }
      
    }
    for (const int i : target->index_range()) {
      if (target->type(i) == node->type(*index)) {
        continue;
      }
      
    }
  }
  for (const NodePtr &node : nodes) {
    if (node->first_connection(context).has_value()) {
      continue;
    }
    
  }
}
*/

//#endif

struct ConnectWeight {
  bool connected;
  std::optional<int> first_index;
  bool same_name;
  int type_conversion;
  int field_interfacing_conversion;

  ConnectWeight(const Context &context, const Node &node, const int index) :
    connected(node->first_connection(context).has_value()),
    same_name(node->name() )
};

}
