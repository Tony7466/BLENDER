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

bool is_valid_gizmo_link(const bNodeLink &link)
{
  const bNodeSocket &from_socket = *link.fromsock;
  const bNodeSocket &to_socket = *link.tosock;
  if (link.is_muted()) {
    return false;
  }
  if (!link.is_available()) {
    return false;
  }
  if (is_scalar_socket_type(from_socket.type) && is_scalar_socket_type(to_socket.type)) {
    return true;
  }
  return from_socket.type == to_socket.type;
}

static std::optional<GizmoSource> find_local_gizmo_source_recursive(
    const bNodeSocket &current_socket,
    const SocketElem &current_elem,
    Vector<GizmoPathElem> &right_to_left_path)
{
  if (current_socket.is_input()) {
    const bNodeSocket &input_socket = current_socket;
    const Span<const bNodeLink *> links = input_socket.directly_linked_links();
    if ((input_socket.flag & SOCK_HIDE_VALUE) == 0 && links.is_empty()) {
      return InputSocketGizmoSource{&current_socket, current_elem};
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
    if (is_valid_gizmo_link(link)) {
      return find_local_gizmo_source_recursive(origin_socket, current_elem, right_to_left_path);
    }
    return std::nullopt;
  }
  else {
    const bNodeSocket &output_socket = current_socket;
    const bNode &current_node = output_socket.owner_node();
    switch (current_node.type) {
      case NODE_REROUTE: {
        const bNodeSocket &input_socket = current_node.input_socket(0);
        return find_local_gizmo_source_recursive(input_socket, current_elem, right_to_left_path);
      }
      case SH_NODE_VALUE:
      case FN_NODE_INPUT_VECTOR:
      case FN_NODE_INPUT_INT: {
        return ValueNodeGizmoSource{&current_node, current_elem};
      }
      case SH_NODE_SEPXYZ: {
        const int axis = output_socket.index();
        const bNodeSocket &input_socket = current_node.input_socket(0);
        return find_local_gizmo_source_recursive(
            input_socket, SocketElem{axis}, right_to_left_path);
      }
      case SH_NODE_COMBXYZ: {
        BLI_assert(current_elem.index.has_value());
        const int axis = *current_elem.index;
        BLI_assert(axis >= 0 && axis < 3);
        const bNodeSocket &input_socket = current_node.input_socket(axis);
        return find_local_gizmo_source_recursive(input_socket, SocketElem{}, right_to_left_path);
      }
      case NODE_GROUP_INPUT: {
        const int interface_input_index = output_socket.index();
        return GroupInputGizmoSource{interface_input_index, current_elem};
      }
      case SH_NODE_MATH: {
        const int mode = current_node.custom1;
        const bNodeSocket &input_socket = current_node.input_socket(0);
        switch (mode) {
          case NODE_MATH_ADD:
          case NODE_MATH_SUBTRACT:
            /* Those modes don't need special handling, because they don't affect the
             * derivative. */
            break;
          case NODE_MATH_MULTIPLY:
          case NODE_MATH_DIVIDE:
            /* We can compute the derivate of those nodes. */
            right_to_left_path.append({&current_node});
            break;
          default:
            return std::nullopt;
        }
        return find_local_gizmo_source_recursive(input_socket, SocketElem{}, right_to_left_path);
      }
      default: {
        return std::nullopt;
      }
    }
  }
}

std::optional<GizmoSource> find_local_gizmo_source(const bNodeSocket &socket,
                                                   const SocketElem &elem,
                                                   Vector<GizmoPathElem> &right_to_left_path)
{
  return find_local_gizmo_source_recursive(socket, elem, right_to_left_path);
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
        {gizmo_source->interface_input_index, gizmo_source->elem}, gizmo_input);
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
      const GizmoInput gizmo_input{&input_socket, interface_gizmo_input.elem};

      Vector<GizmoPathElem> gizmo_path;
      if (const std::optional<GizmoSource> gizmo_source_opt = find_local_gizmo_source(
              input_socket, interface_gizmo_input.elem, gizmo_path))
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
      const bNodeSocket &gizmo_value_input = gizmo_node->input_socket(0);
      for (const bNodeLink *link : gizmo_value_input.directly_linked_links()) {
        if (!is_valid_gizmo_link(*link)) {
          continue;
        }
        Vector<GizmoPathElem> gizmo_path;
        if (const std::optional<GizmoSource> gizmo_source = find_local_gizmo_source(
                *link->fromsock, SocketElem{}, gizmo_path))
        {
          add_gizmo_input_source_pair(
              inferencing_result, GizmoInput{&gizmo_value_input, SocketElem{}}, *gizmo_source);
        }
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
    if (item.key.elem.index.has_value()) {
      stream << ", Elem Index: " << *item.key.elem.index;
    }
    stream << "\n";
    for (const GizmoInput &gizmo_input : item.value) {
      stream << "    " << gizmo_input.input_socket->owner_node().name << " -> "
             << gizmo_input.input_socket->name;
      if (gizmo_input.elem.index.has_value()) {
        stream << ", Elem Index: " << *gizmo_input.elem.index;
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
                                                   gizmo_input.elem};
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
             {gizmo_source->interface_input_index, gizmo_source->elem}))
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
                                             item.key.elem};
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
