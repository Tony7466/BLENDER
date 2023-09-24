/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>

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

std::optional<GizmoSource> find_gizmo_source(const bNodeSocket &socket,
                                             std::optional<int> elem_index)
{
  /* Only scalar values are supported currently. */
  if (!is_scalar_socket_type(socket.type) && !elem_index) {
    return std::nullopt;
  }
  return find_scalar_gizmo_source_recursive(socket, elem_index);
}

Vector<GizmoNodeSource> find_gizmo_node_sources(const bNodeSocket &gizmo_node_input)
{
  BLI_assert(gizmo_node_input.is_input());
  Vector<GizmoNodeSource> gizmo_node_sources;
  for (const bNodeLink *link : gizmo_node_input.directly_linked_links()) {
    if (link->is_muted()) {
      continue;
    }
    const bNodeSocket &origin_socket = *link->fromsock;
    if (!origin_socket.is_available()) {
      continue;
    }
    const bNode &origin_node = origin_socket.owner_node();
    std::optional<GizmoSource> gizmo_source;
    const bNode *variable_node = nullptr;
    if (origin_node.type == GEO_NODE_GIZMO_VARIABLE) {
      const bNodeSocket &gizmo_variable_input = origin_node.input_socket(0);
      gizmo_source = find_gizmo_source(gizmo_variable_input, std::nullopt);
      variable_node = &origin_node;
    }
    else {
      if (is_scalar_socket_type(origin_socket.type)) {
        gizmo_source = find_gizmo_source(origin_socket, std::nullopt);
      }
    }
    if (!gizmo_source) {
      continue;
    }
    gizmo_node_sources.append({*gizmo_source, variable_node});
  }
  return gizmo_node_sources;
}

static void add_gizmo_input_source_pair(GizmoInferencingResult &inferencing_result,
                                        const GizmoInput &gizmo_input,
                                        const GizmoSource &gizmo_source_variant)
{
  if (const auto *gizmo_source = std::get_if<ValueNodeGizmoSource>(&gizmo_source_variant)) {
    inferencing_result.gizmo_inputs_for_value_node.add(gizmo_source->value_node, gizmo_input);
  }
  else if (const auto *gizmo_source = std::get_if<InputSocketGizmoSource>(&gizmo_source_variant)) {
    inferencing_result.gizmo_inputs_for_node_inputs.add(gizmo_source->input_socket, gizmo_input);
  }
  else if (const auto *gizmo_source = std::get_if<GroupInputGizmoSource>(&gizmo_source_variant)) {
    inferencing_result.gizmo_inputs_for_interface_input.add(
        {gizmo_source->interface_input_index, gizmo_source->elem_index}, gizmo_input);
  }
}

static GizmoInferencingResult compute_gizmo_inferencing_result(const bNodeTree &tree)
{
  GizmoInferencingResult inferencing_result;

  for (const bNode *group_node : tree.group_nodes()) {
    const bNodeTree *group = reinterpret_cast<const bNodeTree *>(group_node->id);
    if (group == nullptr) {
      continue;
    }
    if (!group->runtime->gizmo_inferencing) {
      continue;
    }
    for (const InterfaceGizmoInput interface_gizmo_input :
         group->runtime->gizmo_inferencing->gizmo_inputs_for_interface_input.keys())
    {
      const bNodeSocket &input_socket = group_node->input_socket(
          interface_gizmo_input.input_index);
      const GizmoInput gizmo_input{&input_socket, interface_gizmo_input.elem_index};

      if (const std::optional<GizmoSource> gizmo_source_opt = find_gizmo_source(
              input_socket, interface_gizmo_input.elem_index))
      {
        add_gizmo_input_source_pair(inferencing_result, gizmo_input, *gizmo_source_opt);
      }
    }
  }

  for (const StringRefNull idname : {"GeometryNodeGizmoArrow", "GeometryNodeGizmoDial"}) {
    for (const bNode *gizmo_node : tree.nodes_by_type(idname)) {
      const bNodeSocket &gizmo_node_input = gizmo_node->input_socket(0);
      const GizmoInput gizmo_input{&gizmo_node_input, std::nullopt};
      const Vector<GizmoNodeSource> gizmo_node_sources = find_gizmo_node_sources(gizmo_node_input);
      for (const GizmoNodeSource &gizmo_node_source : gizmo_node_sources) {
        add_gizmo_input_source_pair(inferencing_result, gizmo_input, gizmo_node_source.source);
      }
    }
  }

  return inferencing_result;
}

std::ostream &operator<<(std::ostream &stream, const GizmoInferencingResult &data)
{
  stream << "Interface inputs with gizmo:\n";
  for (const auto item : data.gizmo_inputs_for_interface_input.items()) {
    stream << "  Input Index: " << item.key.input_index;
    if (item.key.elem_index.has_value()) {
      stream << ", Elem Index: " << *item.key.elem_index;
    }
    stream << "\n";
    for (const GizmoInput &gizmo_input : item.value) {
      stream << "    " << gizmo_input.input_socket->owner_node().name << " -> "
             << gizmo_input.input_socket->name;
      if (gizmo_input.elem_index.has_value()) {
        stream << ", Elem Index: " << *gizmo_input.elem_index;
      }
      stream << "\n";
    }
  }
  return stream;
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
  std::cout << result << "\n";
  tree.runtime->gizmo_inferencing = std::make_unique<GizmoInferencingResult>(std::move(result));

  /* TODO: Check if interface changed. */
  return true;
}

}  // namespace blender::nodes::gizmos
