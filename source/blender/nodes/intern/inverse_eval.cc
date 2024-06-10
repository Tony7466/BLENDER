/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <queue>

#include "NOD_inverse_eval_path.hh"

#include "BKE_compute_contexts.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "BLI_map.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"

namespace blender::nodes::inverse_eval {

std::optional<ElemVariant> get_elem_variant_for_socket_type(const eNodeSocketDatatype type)
{
  switch (type) {
    case SOCK_FLOAT:
      return {{FloatElem()}};
    case SOCK_INT:
      return {{IntElem()}};
    case SOCK_BOOLEAN:
      return {{BoolElem()}};
    case SOCK_VECTOR:
      return {{VectorElem()}};
    case SOCK_ROTATION:
      return {{RotationElem()}};
    case SOCK_MATRIX:
      return {{TransformElem()}};
    default:
      return std::nullopt;
  }
}

static bool is_supported_value_node(const bNode &node)
{
  return ELEM(
      node.type, SH_NODE_VALUE, FN_NODE_INPUT_VECTOR, FN_NODE_INPUT_BOOL, FN_NODE_INPUT_INT);
}

static std::optional<ElemVariant> convert_socket_elem(const bNodeSocket &old_socket,
                                                      const bNodeSocket &new_socket,
                                                      const ElemVariant &old_elem)
{
  if (old_socket.type == new_socket.type) {
    return old_elem;
  }
  return std::nullopt;
}

LocalInverseEvalPath find_local_inverse_eval_path(const bNodeTree &tree,
                                                  const SocketElem &initial_socket_elem)
{
  BLI_assert(!tree.has_available_link_cycle());

  tree.ensure_topology_cache();

  Map<const bNodeSocket *, ElemVariant> elem_by_socket_map;
  Set<const bNodeSocket *> final_sockets;
  Set<int> final_group_inputs;
  Set<const bNode *> final_value_nodes;

  Stack<SocketElem> sockets_to_handle;
  sockets_to_handle.push(initial_socket_elem);

  while (!sockets_to_handle.is_empty()) {
    const SocketElem socket_elem = sockets_to_handle.pop();
    const bNodeSocket &socket = *socket_elem.socket;

    ElemVariant &elem_variant = elem_by_socket_map.lookup_or_add_cb(&socket, [&]() {
      return *get_elem_variant_for_socket_type(eNodeSocketDatatype(socket.type));
    });
    const ElemVariant old_elem_variant = elem_variant;
    elem_variant.merge(socket_elem.elem);
    if (elem_variant == old_elem_variant) {
      /* Nothing changed. */
      continue;
    }

    if (socket.is_input()) {
      const Span<const bNodeLink *> links = socket.directly_linked_links();
      if (links.size() >= 2) {
        /* Can't continue propagation. */
        continue;
      }
      const bNodeSocket *origin_socket = nullptr;
      if (links.size() == 1) {
        const bNodeLink &link = *links[0];
        if (link.is_used()) {
          origin_socket = link.fromsock;
        }
      }
      if (origin_socket) {
        const std::optional<ElemVariant> origin_elem = convert_socket_elem(
            socket, *origin_socket, elem_variant);
        if (origin_elem) {
          sockets_to_handle.push(SocketElem{origin_socket, *origin_elem});
        }
        else {
          /* Can't continue propagation. */
          continue;
        }
      }
      else {
        final_sockets.add(&socket);
      }
    }
    else {
      const bNode &node = socket.owner_node();
      if (is_supported_value_node(node)) {
        final_value_nodes.add(&node);
      }
      else if (node.is_group_input()) {
        final_group_inputs.add(socket.index());
      }
      else if (node.is_reroute()) {
        sockets_to_handle.push({&node.input_socket(0), elem_variant});
      }
      else {
        const bke::bNodeType &ntype = *node.typeinfo;
        if (!ntype.eval_inverse_elem) {
          /* Node does not support inverse propagation. */
          continue;
        }
        Vector<SocketElem> input_elems;
        InverseElemEvalParams params{node, elem_by_socket_map, input_elems};
        ntype.eval_inverse_elem(params);
        for (const SocketElem &input_elem : input_elems) {
          if (input_elem.elem) {
            sockets_to_handle.push(input_elem);
          }
        }
      }
    }
  }

  LocalInverseEvalPath path;

  for (const bNodeSocket *socket : final_sockets) {
    const ElemVariant &elem = elem_by_socket_map.lookup(socket);
    if (!elem) {
      continue;
    }
    path.final_input_sockets.append({socket, elem});
  }

  for (const bNode *value_node : final_value_nodes) {
    const bNodeSocket &socket = value_node->output_socket(0);
    const ElemVariant &elem = elem_by_socket_map.lookup(&socket);
    if (!elem) {
      continue;
    }
    path.final_value_nodes.append({value_node, elem});
  }

  for (const int group_input_index : final_group_inputs) {
    const eNodeSocketDatatype type = eNodeSocketDatatype(
        tree.interface_inputs()[group_input_index]->socket_typeinfo()->type);
    std::optional<ElemVariant> elem = get_elem_variant_for_socket_type(type);
    if (!elem) {
      continue;
    }
    for (const bNode *node : tree.group_input_nodes()) {
      const bNodeSocket &socket = node->output_socket(group_input_index);
      const ElemVariant &socket_elem = elem_by_socket_map.lookup(&socket);
      elem->merge(socket_elem);
    }
    if (!*elem) {
      continue;
    }
    path.final_group_inputs.append({group_input_index, *elem});
  }

  for (auto &&item : elem_by_socket_map.items()) {
    const bNodeSocket &socket = *item.key;
    const ElemVariant &elem = item.value;
    path.intermediate_sockets.append({&socket, elem});
  }

  return path;
}

InverseElemEvalParams::InverseElemEvalParams(
    const bNode &node,
    const Map<const bNodeSocket *, ElemVariant> &elem_by_socket,
    Vector<SocketElem> &input_elems)
    : elem_by_socket_(elem_by_socket), input_elems_(input_elems), node(node)
{
}

static Vector<int> get_global_node_sort_vector(const ComputeContext *initial_context,
                                               const bNode &initial_node)
{
  Vector<int> vec;
  vec.append(initial_node.runtime->toposort_right_to_left_index);
  for (const ComputeContext *context = initial_context; context; context = context->parent()) {
    if (const auto *group_context = dynamic_cast<const bke::GroupNodeComputeContext *>(context)) {
      const bNode *caller_group_node = group_context->caller_group_node();
      BLI_assert(caller_group_node != nullptr);
      vec.append(caller_group_node->runtime->toposort_right_to_left_index);
    }
  }
  std::reverse(vec.begin(), vec.end());
  return vec;
}

struct NodeInContext {
  const ComputeContext *context = nullptr;
  const bNode *node = nullptr;

  uint64_t hash() const
  {
    return get_default_hash(this->context, this->node);
  }

  friend bool operator<(const NodeInContext &a, const NodeInContext &b)
  {
    const Vector<int> a_sort_vec = get_global_node_sort_vector(a.context, *a.node);
    const Vector<int> b_sort_vec = get_global_node_sort_vector(b.context, *b.node);
    return std::lexicographical_compare(
        b_sort_vec.begin(), b_sort_vec.end(), a_sort_vec.begin(), a_sort_vec.end());
  }

  BLI_STRUCT_EQUALITY_OPERATORS_2(NodeInContext, context, node)
};

struct SocketInContext {
  const ComputeContext *context = nullptr;
  const bNodeSocket *socket = nullptr;

  uint64_t hash() const
  {
    return get_default_hash(this->context, this->socket);
  }

  BLI_STRUCT_EQUALITY_OPERATORS_2(SocketInContext, context, socket)
};

GlobalInverseEvalPath find_global_inverse_eval_path(const ComputeContext *initial_context,
                                                    const SocketElem &initial_socket_elem)
{
  BLI_assert(initial_socket_elem.socket->is_input());
  if (!initial_socket_elem.elem) {
    return {};
  }
  Map<SocketInContext, ElemVariant> elem_by_socket;

  Set<NodeInContext> added_nodes;
  std::priority_queue<NodeInContext> nodes_to_handle;

  const auto update_input_elem_and_forward =
      [&](const ComputeContext *context, const bNodeSocket &socket, const ElemVariant &new_elem) {
        ElemVariant &elem = elem_by_socket.lookup_or_add({context, &socket}, new_elem);
        elem.merge(new_elem);
        const ElemVariant elem_to_forward = elem;
        for (const bNodeLink *link : socket.directly_linked_links()) {
          if (!link->is_used()) {
            continue;
          }
          const bNodeSocket &origin_socket = *link->fromsock;
          const bNode &origin_node = *link->fromnode;
          if (origin_node.is_group()) {
            /* Not yet supported. */
          }
          else {
            ElemVariant &origin_elem = elem_by_socket.lookup_or_add({context, &origin_socket},
                                                                    elem_to_forward);
            origin_elem.merge(elem_to_forward);
            const NodeInContext origin_node_in_context{context, &origin_node};
            if (added_nodes.add(origin_node_in_context)) {
              nodes_to_handle.push(origin_node_in_context);
            }
          }
        }
      };

  update_input_elem_and_forward(
      initial_context, *initial_socket_elem.socket, initial_socket_elem.elem);

  GlobalInverseEvalPath path;
  while (!nodes_to_handle.empty()) {
    const NodeInContext node_in_context = nodes_to_handle.top();
    nodes_to_handle.pop();

    const bNode &node = *node_in_context.node;
    const ComputeContext *context = node_in_context.context;
    path.ordered_nodes.append({context, &node});

    if (is_supported_value_node(node)) {
      /* Inverse evaluation ends here. */
    }
    else if (node.is_reroute()) {
      const ElemVariant elem = elem_by_socket.lookup({context, &node.output_socket(0)});
      update_input_elem_and_forward(context, node.input_socket(0), elem);
    }
    else if (node.is_group_input()) {
      if (const auto *group_context = dynamic_cast<const bke::GroupNodeComputeContext *>(context))
      {
        const bNode *caller_group_node = group_context->caller_group_node();
        BLI_assert(caller_group_node);
        const ComputeContext *caller_context = context->parent();

        for (const bNodeSocket *socket : node.output_sockets()) {
          if (const ElemVariant *elem = elem_by_socket.lookup_ptr({context, socket})) {
            if (*elem) {
              const bNodeSocket &caller_input_socket = caller_group_node->input_socket(
                  socket->index());
              update_input_elem_and_forward(caller_context, caller_input_socket, *elem);
            }
          }
        }
      }
    }
    else {
      const bke::bNodeType &ntype = *node.typeinfo;
      if (!ntype.eval_inverse_elem) {
        /* Node does not support inverse evaluation. */
        continue;
      }
      Vector<SocketElem> input_elems;
      Map<const bNodeSocket *, ElemVariant> elem_by_local_socket;
      for (const bNodeSocket *output_socket : node.output_sockets()) {
        if (const ElemVariant *elem = elem_by_socket.lookup_ptr({context, output_socket})) {
          elem_by_local_socket.add(output_socket, *elem);
        }
      }
      InverseElemEvalParams params{node, elem_by_local_socket, input_elems};
      ntype.eval_inverse_elem(params);
      for (const SocketElem &input_elem : input_elems) {
        if (input_elem.elem) {
          update_input_elem_and_forward(context, *input_elem.socket, input_elem.elem);
        }
      }
    }
  }

  return path;
}

}  // namespace blender::nodes::inverse_eval
