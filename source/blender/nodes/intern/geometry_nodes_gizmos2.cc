/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_base_safe.h"
#include "BLI_math_rotation.hh"

#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "NOD_geometry_nodes_gizmos2.hh"
#include "NOD_inverse_eval.hh"
#include "NOD_inverse_eval_path.hh"

namespace blender::nodes::gizmos2 {

static void reset_gizmo_states(bNodeTree &tree)
{
  for (bNodeSocket *socket : tree.all_sockets()) {
    socket->runtime->has_gizmo2 = false;
  }
}

static ie::ElemVariant get_gizmo_socket_elem(const bNode & /*node*/, const bNodeSocket &socket)
{
  if (std::optional<ie::ElemVariant> elem = ie::get_elem_variant_for_socket_type(
          eNodeSocketDatatype(socket.type)))
  {
    elem->set_all();
    return *elem;
  }
  BLI_assert_unreachable();
  return {};
}

static TreeGizmoPropagation build_tree_gizmo_propagation(bNodeTree &tree)
{
  BLI_assert(!tree.has_available_link_cycle());

  TreeGizmoPropagation gizmo_propagation;

  struct GizmoInput {
    const bNodeSocket *gizmo_socket;
    const bNodeSocket *propagation_start_socket;
    ie::ElemVariant elem;
  };

  Vector<GizmoInput> all_gizmo_inputs;

  for (const bNode *node : tree.all_nodes()) {
    if (node->is_group()) {
      if (!node->id) {
        continue;
      }
      const bNodeTree &group = *reinterpret_cast<const bNodeTree *>(node->id);
      if (!group.runtime->gizmo_propagation) {
        continue;
      }
      const TreeGizmoPropagation &group_gizmo_propagation = *group.runtime->gizmo_propagation;
      if (group_gizmo_propagation.gizmo_inputs_by_group_inputs.size() > 0) {
        gizmo_propagation.nodes_containing_gizmos.append(node);
      }
      for (const ie::GroupInputElem &group_input_elem :
           group_gizmo_propagation.gizmo_inputs_by_group_inputs.keys())
      {
        const bNodeSocket &input_socket = node->input_socket(group_input_elem.group_input_index);
        all_gizmo_inputs.append({&input_socket, &input_socket, group_input_elem.elem});
      }
    }
    if (ELEM(node->type, GEO_NODE_GIZMO_LINEAR, GEO_NODE_GIZMO_DIAL, GEO_NODE_GIZMO_TRANSFORM)) {
      const bNodeSocket &gizmo_input_socket = node->input_socket(0);
      const ie::ElemVariant elem = get_gizmo_socket_elem(*node, gizmo_input_socket);
      for (const bNodeLink *link : gizmo_input_socket.directly_linked_links()) {
        if (!link->is_used()) {
          continue;
        }
        all_gizmo_inputs.append({&gizmo_input_socket, link->fromsock, elem});
      }
    }
  }

  for (const GizmoInput &gizmo_input : all_gizmo_inputs) {
    const ie::SocketElem gizmo_input_socket_elem{gizmo_input.gizmo_socket, gizmo_input.elem};
    const ie::PropagationPath propagation_path = ie::find_propagation_path(
        tree, {gizmo_input.propagation_start_socket, gizmo_input.elem});
    const bool has_target = !propagation_path.final_input_sockets.is_empty() ||
                            !propagation_path.final_group_inputs.is_empty() ||
                            !propagation_path.final_value_nodes.is_empty();
    if (!has_target) {
      continue;
    }
    for (const ie::PropagationPathNode &path_node : propagation_path.nodes.values()) {
      for (const ie::SocketElem &socket_elem : path_node.inputs) {
        socket_elem.socket->runtime->has_gizmo2 = true;
      }
      for (const ie::SocketElem &socket_elem : path_node.outputs) {
        socket_elem.socket->runtime->has_gizmo2 = true;
      }
    }
    for (const ie::SocketElem &input_socket : propagation_path.final_input_sockets) {
      gizmo_propagation.gizmo_inputs_by_node_inputs.add(input_socket, gizmo_input_socket_elem);
    }
    for (const ie::ValueNodeElem &value_node : propagation_path.final_value_nodes) {
      gizmo_propagation.gizmo_inputs_by_value_nodes.add(value_node, gizmo_input_socket_elem);
    }
    for (const ie::GroupInputElem &group_input : propagation_path.final_group_inputs) {
      gizmo_propagation.gizmo_inputs_by_group_inputs.add(group_input, gizmo_input_socket_elem);
    }
  }

  return gizmo_propagation;
}

bool update_tree_gizmo_propagation(bNodeTree &tree)
{
  tree.ensure_topology_cache();
  reset_gizmo_states(tree);
  if (tree.has_available_link_cycle()) {
    const bool changed = tree.runtime->gizmo_propagation.get() != nullptr;
    tree.runtime->gizmo_propagation.reset();
    return changed;
  }

  TreeGizmoPropagation new_gizmo_propagation = build_tree_gizmo_propagation(tree);
  const bool changed = tree.runtime->gizmo_propagation ?
                           *tree.runtime->gizmo_propagation != new_gizmo_propagation :
                           true;
  tree.runtime->gizmo_propagation = std::make_unique<TreeGizmoPropagation>(
      std::move(new_gizmo_propagation));
  return changed;
}

}  // namespace blender::nodes::gizmos2
