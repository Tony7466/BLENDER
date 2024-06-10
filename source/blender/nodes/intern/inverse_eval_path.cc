/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_inverse_eval_path.hh"

#include "BKE_node.hh"

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

static std::optional<ElemVariant> convert_socket_elem(const bNodeSocket &old_socket,
                                                      const bNodeSocket &new_socket,
                                                      const ElemVariant &old_elem)
{
  if (old_socket.type == new_socket.type) {
    return old_elem;
  }
  return std::nullopt;
}

LocalInversePropagationPath find_local_inverse_propagation_path(
    const bNodeTree &tree, const SocketElem &initial_socket_elem)
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
      switch (node.type) {
        case SH_NODE_VALUE:
        case FN_NODE_INPUT_VECTOR:
        case FN_NODE_INPUT_BOOL:
        case FN_NODE_INPUT_INT: {
          final_value_nodes.add(&node);
          break;
        }
        case NODE_GROUP_INPUT: {
          final_group_inputs.add(socket.index());
          break;
        }
        case NODE_REROUTE: {
          sockets_to_handle.push({&node.input_socket(0), elem_variant});
          break;
        }
        default: {
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
          break;
        }
      }
    }
  }

  LocalInversePropagationPath propagation_path;

  for (const bNodeSocket *socket : final_sockets) {
    const ElemVariant &elem = elem_by_socket_map.lookup(socket);
    if (!elem) {
      continue;
    }
    propagation_path.final_input_sockets.append({socket, elem});
  }

  for (const bNode *value_node : final_value_nodes) {
    const bNodeSocket &socket = value_node->output_socket(0);
    const ElemVariant &elem = elem_by_socket_map.lookup(&socket);
    if (!elem) {
      continue;
    }
    propagation_path.final_value_nodes.append({value_node, elem});
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
    propagation_path.final_group_inputs.append({group_input_index, *elem});
  }

  for (auto &&item : elem_by_socket_map.items()) {
    const bNodeSocket &socket = *item.key;
    const ElemVariant &elem = item.value;
    propagation_path.intermediate_sockets.append({&socket, elem});
  }

  return propagation_path;
}

InverseElemEvalParams::InverseElemEvalParams(
    const bNode &node,
    const Map<const bNodeSocket *, ElemVariant> &elem_by_socket,
    Vector<SocketElem> &input_elems)
    : elem_by_socket_(elem_by_socket), input_elems_(input_elems), node(node)
{
}

}  // namespace blender::nodes::inverse_eval
