/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 */

#include "DNA_node_types.h"

#include "BLI_offset_indices.hh"

#include "BKE_node_runtime.hh"

#include "NOD_node_declaration.hh"

namespace blender::ed::space_node::connection {

//#if (0)

enum class SocketState : int8_t {
  ConnectedToTarger,
  ConnectedToOther,
  Free,
};

class TargetSocketNode;

class Context {
 public:
  bNodeTree &tree;
  TargetSocketNode &target_node;
  int index;
  bool side_is_input;
};

class Node {
 public:
  virtual ~Node() = default;

  IndexRange index_range(const Context &context) const
  {
    return IndexRange(this->size(context));
  }

  std::optional<int> first_connection(const Context &context) const
  {
    for (const int index : this->index_range(context)) {
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

  // TODO: std::pair<bNode *, bNodeSocket *> to_connect(Context &context, int socket_index) const =
  // 0;
};

class RealNode : public Node {
 private:
  const bNode &node_;

 public:
  RealNode(const bNode &node) : node_(node) {}

  int size(const Context &context) const final
  {
    /* TODO: Do not count hide sockets! */
    return int(context.side_is_input ? node_.input_sockets().size() :
                                       node_.output_sockets().size());
  }

  bool is_connected(const Context &context, const int socket_index) const override
  {
    BLI_assert(&node_ != &context.target_node);
    const Span<const bNodeSocket *> sockets = context.side_is_input ? node_.input_sockets() :
                                                                      node_.output_sockets();
    for (const bNodeSocket *other_socket : sockets[socket_index]->logically_linked_sockets()) {
      if (&other_socket->owner_node() == &context.target_node->node_) {
        return true;
      }
    }
    return false;
  }

  eNodeSocketDatatype type(const Context &context, int socket_index) const override
  {
    const Span<const bNodeSocket *> sockets = context.side_is_input ? node_.input_sockets() :
                                                                      node_.output_sockets();
    return eNodeSocketDatatype(sockets[socket_index]->type);
  }

  StringRef name(const Context &context, int socket_index) const override
  {
    const Span<const bNodeSocket *> sockets = context.side_is_input ? node_.input_sockets() :
                                                                      node_.output_sockets();
    return sockets[socket_index]->name;
  }

  bool single_value(const Context &context, int socket_index) const override
  {
    BLI_assert(context.tree != nullptr);
    const Span<const bNodeSocket *> sockets = context.side_is_input ? node_.input_sockets() :
                                                                      node_.output_sockets();
    if (context.side_is_input) {
      return nodes::InputSocketFieldType::None ==
             sockets[socket_index]->runtime->declaration->input_field_type;
    }
    return nodes::OutputSocketFieldType::None ==
           sockets[socket_index]->runtime->declaration->output_field_dependency.field_type();
  }
};

class TargetSocketNode : public Node {
 private:
  const eNodeSocketDatatype type_;
  const StringRef name_;

 public:
  TargetSocketNode(const eNodeSocketDatatype type, const StringRef name) : type_(type), name_(name)
  {
  }

  int size(const Context & /*context*/) const final
  {
    return 1;
  }

  bool is_connected(const Context &context, int socket_index) const final
  {
    return false;
  }

  eNodeSocketDatatype type(const Context &context, int socket_index) const final
  {
    return type_;
  }

  StringRef name(const Context &context, int socket_index) const final
  {
    return name;
  }

  bool single_value(const Context &context, int socket_index) const final
  {
    return false;
  }
};

class VirtualNode : public Node {
 public:
  bool is_connected(const Context & /*context*/, const int /*socket_index*/) const final
  {
    return false;
  }
};

class ViewerNode : public VirtualNode {
 private:
  const eNodeSocketDatatype data_type_;

 public:
  ViewerNode(const eNodeSocketDatatype data_type) : data_type_(data_type) {}

  int size(const Context & /*context*/) const final
  {
    return 2;
  }

  eNodeSocketDatatype type(const Context & /*context*/, int socket_index) const override
  {
    if (socket_index == 0) {
      return SOCK_GEOMETRY;
    }
    return data_type_;
  }

  StringRef name(const Context & /*context*/, int socket_index) const override
  {
    if (socket_index == 0) {
      return "Geometry";
    }
    return "Value";
  }

  bool single_value(const Context & /*context*/, int socket_index) const override
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
    const Span<bNodeTreeInterfaceSocket *> sockets = context.side_is_input ?
                                                         group_.interface_inputs() :
                                                         group_.interface_outputs();
    return int(context.side_is_input ? group_.interface_inputs().size() :
                                       group_.interface_outputs().size());
  }

  eNodeSocketDatatype type(const Context &context, int socket_index) const override
  {
    const Span<bNodeTreeInterfaceSocket *> sockets = context.side_is_input ?
                                                         group_.interface_inputs() :
                                                         group_.interface_outputs();
    return eNodeSocketDatatype(sockets[socket_index]->socket_typeinfo()->type);
  }

  StringRef name(const Context &context, int socket_index) const override
  {
    const Span<bNodeTreeInterfaceSocket *> sockets = context.side_is_input ?
                                                         group_.interface_inputs() :
                                                         group_.interface_outputs();
    return sockets[socket_index]->name;
  }

  bool single_value(const Context &context, int socket_index) const override
  {
    const nodes::FieldInferencingInterface &interface =
        *group_.runtime->field_inferencing_interface;
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

/*

class Socket {
 private:
  const NodePtr own_node_;
  const int index_;
  const SocketState state_;

 public:

  bool operator > (const Socket &other) const
  {
    return false;
  }
};

*/

/*

using NodeFilter = FunctionRef<bool(const MutableSpan<NodePtr> nodes)>;

static bool node_state_filter(const MutableSpan<NodePtr> nodes, const NodeFilter next_filter)
{
  Vector<NodePtr> connected;
  Vector<NodePtr> exist;
  Vector<NodePtr> other;

  for (NodePtr &node : nodes) {
    if (node->first_connection()) {
      connected.append(std::move(node));
      continue;
    }
    if (dynamic_cast<const RealNode *>(&*node)) {
      exist.append(std::move(node));
      continue;
    }
    other.append(std::move(node));
  }

  return next_filter(connected) || next_filter(exist) || next_filter(other);
}

static bool socket_state_filter(const MutableSpan<NodePtr> nodes, const NodeFilter next_filter)
{
  Vector<NodePtr> targer;
  Vector<NodePtr> exist;
  Vector<NodePtr> other;

  for (NodePtr &node : nodes) {
    if (node->first_connection()) {
      connected.append(std::move(node));
      continue;
    }
    if (dynamic_cast<const RealNode *>(&*node)) {
      exist.append(std::move(node));
      continue;
    }
    other.append(std::move(node));
  }

  return next_filter(connected) || next_filter(exist) || next_filter(other);
}

*/

/*
struct ConnectWeight {
  bool connected;
  std::optional<int> first_index;
  bool same_name;
  int type_conversion;
  int field_interfacing_conversion;

  ConnectWeight(const Context &context, const Node &node, const int index) :
    connected(node->first_connection(context).has_value()),
    same_name(node->name() );
};
*/

class Socket {
  // const NodePtr &node;
};

static bool sockets_cmp(const Socket &a, const Socket b)
{
  return false;
}

static std::optional<Socket> lookup_socket_for_context(const Span<NodePtr> nodes,
                                                       const Context context)
{
  Array<int> accumulate_sockets(nodes.size() + 1, 0);
  for (const int index : nodes.index_range()) {
    accumulate_sockets[index] = nodes[index]->size(context);
  }
  OffsetIndices<int> sockets_offset = offset_indices::accumulate_counts_to_offsets(
      accumulate_sockets);
  Array<Socket> sockets(sockets_offset.total_size());
  for (const int node_index : nodes.index_range()) {
    const NodePtr &node = nodes[node_index];
    IndexRange node_sockets = sockets_offset[node_index];
    for (const int index : node_sockets.index_range()) {
      sockets[node_sockets[index]];  // = Socket(context, node);
    }
  }

  const Socket *socket = std::max_element(sockets.begin(), sockets.end(), sockets_cmp);
  if (socket == sockets.end()) {
    return std::nullopt;
  }

  return *socket;
}

}  // namespace blender::ed::space_node::connection
