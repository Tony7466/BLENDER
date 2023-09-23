/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_geometry_nodes_gizmos.hh"

#include "BKE_node_runtime.hh"

namespace blender::nodes::gizmos {

static bool is_scalar_socket_type(const int socket_type)
{
  return ELEM(socket_type, SOCK_FLOAT, SOCK_INT);
}

static std::optional<GizmoSource> find_scalar_gizmo_source_recursive(
    const bNodeSocket &current_socket, const std::optional<int> &current_elem_index)
{
  if (current_socket.is_input()) {
    const bNodeSocket &input_socket = current_socket;
    const Span<const bNodeLink *> links = input_socket.directly_linked_links();
    if ((input_socket.flag & SOCK_HIDE_VALUE) == 0 && links.is_empty()) {
      return InputSocketGizmoSource{&current_socket, current_elem_index};
    }
    if (links.size() != 1) {
      return std::nullopt;
    }
    const bNodeLink &link = *links[0];
    if (link.is_muted()) {
      return std::nullopt;
    }
    const bNodeSocket &origin_socket = *link.fromsock;
    if (!origin_socket.is_available()) {
      return std::nullopt;
    }
    if (origin_socket.type == input_socket.type ||
        (is_scalar_socket_type(input_socket.type) && is_scalar_socket_type(origin_socket.type)))
    {
      return find_scalar_gizmo_source_recursive(origin_socket, current_elem_index);
    }
    return std::nullopt;
  }
  else {
    const bNodeSocket &output_socket = current_socket;
    const bNode &current_node = output_socket.owner_node();
    switch (current_node.type) {
      case NODE_REROUTE: {
        const bNodeSocket &input_socket = current_node.input_socket(0);
        return find_scalar_gizmo_source_recursive(input_socket, current_elem_index);
      }
      case SH_NODE_VALUE:
      case FN_NODE_INPUT_VECTOR:
      case FN_NODE_INPUT_INT: {
        return ValueNodeGizmoSource{&current_node, current_elem_index};
      }
      case SH_NODE_SEPXYZ: {
        const int axis = output_socket.index();
        const bNodeSocket &input_socket = current_node.input_socket(0);
        return find_scalar_gizmo_source_recursive(input_socket, axis);
      }
      case SH_NODE_COMBXYZ: {
        BLI_assert(current_elem_index.has_value());
        const int axis = *current_elem_index;
        BLI_assert(axis >= 0 && axis < 3);
        const bNodeSocket &input_socket = current_node.input_socket(axis);
        return find_scalar_gizmo_source_recursive(input_socket, std::nullopt);
      }
      case NODE_GROUP_INPUT: {
        const int interface_input_index = output_socket.index();
        return GroupInputGizmoSource{interface_input_index, current_elem_index};
      }
      default: {
        return std::nullopt;
      }
    }
  }
}

std::optional<GizmoSource> find_scalar_gizmo_source(const bNodeSocket &socket)
{
  if (!is_scalar_socket_type(socket.type)) {
    return std::nullopt;
  }
  return find_scalar_gizmo_source_recursive(socket, std::nullopt);
}

static GizmoInferencingResult compute_gizmo_inferencing_result(const bNodeTree &tree)
{
  GizmoInferencingResult result;

  for (const bNode *group_node : tree.group_nodes()) {
    const bNodeTree *group = reinterpret_cast<const bNodeTree *>(group_node->id);
    if (group == nullptr) {
      continue;
    }
    if (!group->runtime->gizmo_inferencing) {
      continue;
    }
    for (const int input_index :
         group->runtime->gizmo_inferencing->gizmo_inputs_by_interface_input.keys())
    {
      result.gizmo_inputs_in_group_node.add(group_node, &group_node->input_socket(input_index));
    }
  }

  for (const StringRefNull idname : {"GeometryNodeGizmoArrow", "GeometryNodeGizmoDial"}) {
    for (const bNode *gizmo_node : tree.nodes_by_type(idname)) {
    }
  }

  return result;
}

bool update_gizmo_inferencing(bNodeTree &tree)
{
  tree.ensure_topology_cache();
  if (tree.has_available_link_cycle()) {
    const bool changed = tree.runtime->gizmo_inferencing.get() != nullptr;
    tree.runtime->gizmo_inferencing.reset();
    return changed;
  }

  GizmoInferencingResult result = compute_gizmo_inferencing_result(tree);

  Vector<int> old_inputs_with_gizmo;
  Vector<int> new_inputs_with_gizmo;

  if (tree.runtime->gizmo_inferencing) {
    auto keys = tree.runtime->gizmo_inferencing->gizmo_inputs_by_interface_input.keys();
    old_inputs_with_gizmo.extend(keys.begin(), keys.end());
  }
  {
    auto keys = result.gizmo_inputs_by_interface_input.keys();
    new_inputs_with_gizmo.extend(keys.begin(), keys.end());
  }
  std::sort(old_inputs_with_gizmo.begin(), old_inputs_with_gizmo.end());
  std::sort(new_inputs_with_gizmo.begin(), new_inputs_with_gizmo.end());

  const bool group_interface_changed = !tree.runtime->gizmo_inferencing ||
                                       old_inputs_with_gizmo != new_inputs_with_gizmo;

  tree.runtime->gizmo_inferencing = std::make_unique<GizmoInferencingResult>(std::move(result));

  return group_interface_changed;
}

}  // namespace blender::nodes::gizmos
