/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>

#include "NOD_geometry_nodes_gizmos.hh"

#include "BLI_set.hh"

#include "BKE_compute_contexts.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_zones.hh"
#include "BKE_workspace.h"

#include "DNA_modifier_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "ED_node.hh"

namespace blender::nodes::gizmos {

static bool is_scalar_socket_type(const int socket_type)
{
  return ELEM(socket_type, SOCK_FLOAT, SOCK_INT);
}

static bool is_valid_gizmo_value_link(const bNodeSocket &from_sock, const bNodeSocket &to_sock)
{
  if (is_scalar_socket_type(from_sock.type) && is_scalar_socket_type(to_sock.type)) {
    return true;
  }
  return from_sock.type == to_sock.type;
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
    if (is_valid_gizmo_value_link(origin_socket, input_socket)) {
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
  return find_scalar_gizmo_source_recursive(socket, elem_index);
}

Vector<GizmoNodeSource> find_gizmo_node_sources(const bNodeSocket &gizmo_node_input,
                                                const std::optional<int> elem_index)
{
  BLI_assert(gizmo_node_input.is_input());
  Vector<GizmoNodeSource> gizmo_node_sources;
  if (!gizmo_node_input.is_directly_linked()) {
    gizmo_node_sources.append(
        {GizmoSource(InputSocketGizmoSource{&gizmo_node_input, elem_index})});
  }
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
      gizmo_source = find_gizmo_source(gizmo_variable_input, elem_index);
      variable_node = &origin_node;
    }
    else {
      if (is_valid_gizmo_value_link(origin_socket, gizmo_node_input)) {
        gizmo_source = find_gizmo_source(origin_socket, elem_index);
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
    if (group->runtime->gizmo_inferencing->gizmo_inputs_for_interface_input.size() > 0) {
      inferencing_result.nodes_with_gizmos_inside.append(group_node);
    }
  }

  for (const StringRefNull idname : {"GeometryNodeGizmoArrow", "GeometryNodeGizmoDial"}) {
    for (const bNode *gizmo_node : tree.nodes_by_type(idname)) {
      const bNodeSocket &gizmo_node_input = gizmo_node->input_socket(0);
      const GizmoInput gizmo_input{&gizmo_node_input, std::nullopt};
      const Vector<GizmoNodeSource> gizmo_node_sources = find_gizmo_node_sources(gizmo_node_input,
                                                                                 std::nullopt);
      for (const GizmoNodeSource &gizmo_node_source : gizmo_node_sources) {
        add_gizmo_input_source_pair(inferencing_result, gizmo_input, gizmo_node_source.source);
      }
      inferencing_result.nodes_with_gizmos_inside.append(gizmo_node);
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

static void foreach_gizmo_for_source(
    const GizmoSource &gizmo_source_variant,
    ComputeContextBuilder &compute_context_builder,
    const bNodeTree &tree,
    FunctionRef<void(const ComputeContext &compute_context, const bNode &gizmo_node)> fn);

static void foreach_gizmo_for_input(
    const GizmoInput &gizmo_input,
    ComputeContextBuilder &compute_context_builder,
    const bNodeTree &tree,
    FunctionRef<void(const ComputeContext &compute_context, const bNode &gizmo_node)> fn)
{
  const bke::bNodeTreeZones *zones = tree.zones();
  if (zones == nullptr) {
    return;
  }
  const bNode &node = gizmo_input.input_socket->owner_node();
  if (zones->get_zone_by_node(node.identifier) != nullptr) {
    /* Gizmos in zones are not supported yet. */
    return;
  }
  if (ELEM(node.type, GEO_NODE_GIZMO_ARROW, GEO_NODE_GIZMO_DIAL)) {
    fn(*compute_context_builder.current(), node);
  }
  else if (node.is_group()) {
    const GroupInputGizmoSource group_input_source{gizmo_input.input_socket->index(),
                                                   gizmo_input.elem_index};
    const bNodeTree &group = *reinterpret_cast<const bNodeTree *>(node.id);
    compute_context_builder.push<bke::NodeGroupComputeContext>(node, tree);
    foreach_gizmo_for_source(group_input_source, compute_context_builder, group, fn);
    compute_context_builder.pop();
  }
}

static void foreach_gizmo_for_source(
    const GizmoSource &gizmo_source_variant,
    ComputeContextBuilder &compute_context_builder,
    const bNodeTree &tree,
    FunctionRef<void(const ComputeContext &compute_context, const bNode &gizmo_node)> fn)
{
  const GizmoInferencingResult &gizmo_inferencing = *tree.runtime->gizmo_inferencing;

  if (const auto *gizmo_source = std::get_if<ValueNodeGizmoSource>(&gizmo_source_variant)) {
    for (const GizmoInput &gizmo_input :
         gizmo_inferencing.gizmo_inputs_for_value_node.lookup(gizmo_source->value_node))
    {
      foreach_gizmo_for_input(gizmo_input, compute_context_builder, tree, fn);
    }
  }
  else if (const auto *gizmo_source = std::get_if<InputSocketGizmoSource>(&gizmo_source_variant)) {
    for (const GizmoInput &gizmo_input :
         gizmo_inferencing.gizmo_inputs_for_node_inputs.lookup(gizmo_source->input_socket))
    {
      foreach_gizmo_for_input(gizmo_input, compute_context_builder, tree, fn);
    }
  }
  else if (const auto *gizmo_source = std::get_if<GroupInputGizmoSource>(&gizmo_source_variant)) {
    for (const GizmoInput &gizmo_input : gizmo_inferencing.gizmo_inputs_for_interface_input.lookup(
             {gizmo_source->interface_input_index, gizmo_source->elem_index}))
    {
      foreach_gizmo_for_input(gizmo_input, compute_context_builder, tree, fn);
    }
  }
}

void foreach_active_gizmo(
    const Object &object,
    const NodesModifierData &nmd,
    const wmWindowManager &wm,
    const FunctionRef<void(const ComputeContext &compute_context, const bNode &gizmo_node)> fn)
{
  if (nmd.node_group == nullptr) {
    return;
  }
  LISTBASE_FOREACH (const wmWindow *, window, &wm.windows) {
    const bScreen *active_screen = BKE_workspace_active_screen_get(window->workspace_hook);
    Vector<const bScreen *> screens = {active_screen};
    if (ELEM(active_screen->state, SCREENMAXIMIZED, SCREENFULL)) {
      const ScrArea *area = static_cast<const ScrArea *>(active_screen->areabase.first);
      screens.append(area->full);
    }
    for (const bScreen *screen : screens) {
      LISTBASE_FOREACH (const ScrArea *, area, &screen->areabase) {
        const SpaceLink *sl = static_cast<SpaceLink *>(area->spacedata.first);
        if (sl == nullptr) {
          continue;
        }
        if (sl->spacetype == SPACE_NODE) {
          const SpaceNode &snode = *reinterpret_cast<const SpaceNode *>(sl);
          if (snode.nodetree == nullptr) {
            continue;
          }
          if (!snode.edittree->runtime->gizmo_inferencing) {
            continue;
          }
          const std::optional<ed::space_node::ObjectAndModifier> object_and_modifier =
              ed::space_node::get_modifier_for_node_editor(snode);
          if (!object_and_modifier.has_value()) {
            continue;
          }
          if (object_and_modifier->object != &object) {
            continue;
          }
          if (object_and_modifier->nmd != &nmd) {
            continue;
          }

          ComputeContextBuilder compute_context_builder;
          compute_context_builder.push<bke::ModifierComputeContext>(nmd.modifier.name);
          if (!ed::space_node::push_compute_context_for_tree_path(snode, compute_context_builder))
          {
            continue;
          }
          const GizmoInferencingResult &gizmo_inferencing =
              *snode.edittree->runtime->gizmo_inferencing;
          Set<GizmoInput> used_gizmo_inputs;
          for (auto item : gizmo_inferencing.gizmo_inputs_for_value_node.items()) {
            const bNode &node = *item.key;
            if (!(node.flag & NODE_SELECT)) {
              continue;
            }
            used_gizmo_inputs.add_multiple(item.value);
          }
          for (auto item : gizmo_inferencing.gizmo_inputs_for_node_inputs.items()) {
            const bNode &node = item.key->owner_node();
            if (!(node.flag & NODE_SELECT)) {
              continue;
            }
            used_gizmo_inputs.add_multiple(item.value);
          }
          for (const bNode *node : gizmo_inferencing.nodes_with_gizmos_inside) {
            if (!(node->flag & NODE_SELECT)) {
              continue;
            }
            if (ELEM(node->type, GEO_NODE_GIZMO_ARROW, GEO_NODE_GIZMO_DIAL)) {
              fn(*compute_context_builder.current(), *node);
            }
            else if (node->is_group()) {
              const bNodeTree *group = reinterpret_cast<const bNodeTree *>(node->id);
              for (const auto item :
                   group->runtime->gizmo_inferencing->gizmo_inputs_for_interface_input.items()) {
                const GizmoInput gizmo_input{&node->input_socket(item.key.input_index),
                                             item.key.elem_index};
                used_gizmo_inputs.add(gizmo_input);
              }
            }
          }
          for (const GizmoInput &gizmo_input : used_gizmo_inputs) {
            foreach_gizmo_for_input(gizmo_input, compute_context_builder, *snode.edittree, fn);
          }
        }
      }
    }
  }

  {
    const bNodeTree &tree = *nmd.node_group;
    if (!tree.runtime->gizmo_inferencing) {
      return;
    }
    const GizmoInferencingResult &gizmo_inferencing = *tree.runtime->gizmo_inferencing;

    ComputeContextBuilder compute_context_builder;
    compute_context_builder.push<bke::ModifierComputeContext>(nmd.modifier.name);

    for (const Span<GizmoInput> gizmo_inputs :
         gizmo_inferencing.gizmo_inputs_for_interface_input.values())
    {
      for (const GizmoInput &gizmo_input : gizmo_inputs) {
        foreach_gizmo_for_input(gizmo_input, compute_context_builder, tree, fn);
      }
    }
  }
}

}  // namespace blender::nodes::gizmos
