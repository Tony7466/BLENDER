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

/**
 * True if a gizmo can be propagated through that link.
 */
static bool is_valid_gizmo_link(const bNodeLink &link)
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

/**
 * Contains the nodes that propagate the gizmo from left-to-right and influence the mapping from
 * gizmo to value changes.
 */
struct LocalPropagationPath {
  struct PathElem {
    const bNode *node = nullptr;
    ValueElem elem;
  };

  Vector<PathElem> path;
};

/**
 * Propagates a gizmo controlling the given socket to its "real" linked target within the current
 * node group. Also keeps track of the nodes the modify the gizmo-to-value-mapping.
 */
static std::optional<GizmoTarget> find_local_gizmo_target(const bNodeSocket &current_socket,
                                                          const ValueElem &current_elem,
                                                          LocalPropagationPath &r_propagation_path)
{
  current_socket.runtime->has_gizmo = true;
  if (current_socket.is_input()) {
    const bNodeSocket &input_socket = current_socket;
    const Span<const bNodeLink *> links = input_socket.directly_linked_links();
    if ((input_socket.flag & SOCK_HIDE_VALUE) == 0 && links.is_empty()) {
      return InputSocketRef{&current_socket, current_elem};
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
      return find_local_gizmo_target(origin_socket, current_elem, r_propagation_path);
    }
    return std::nullopt;
  }
  else {
    const bNodeSocket &output_socket = current_socket;
    const bNode &current_node = output_socket.owner_node();
    switch (current_node.type) {
      case SH_NODE_VALUE:
      case FN_NODE_INPUT_VECTOR:
      case FN_NODE_INPUT_INT: {
        /* The value in those nodes is the gizmo target. */
        return ValueNodeRef{&current_node, current_elem};
      }
      case NODE_REROUTE: {
        const bNodeSocket &input_socket = current_node.input_socket(0);
        return find_local_gizmo_target(input_socket, current_elem, r_propagation_path);
      }
      case SH_NODE_SEPXYZ: {
        const int axis = output_socket.index();
        const bNodeSocket &input_socket = current_node.input_socket(0);
        return find_local_gizmo_target(input_socket, ValueElem{axis}, r_propagation_path);
      }
      case SH_NODE_COMBXYZ: {
        BLI_assert(current_elem.index.has_value());
        const int axis = *current_elem.index;
        BLI_assert(axis >= 0 && axis < 3);
        const bNodeSocket &input_socket = current_node.input_socket(axis);
        return find_local_gizmo_target(input_socket, ValueElem{}, r_propagation_path);
      }
      case NODE_GROUP_INPUT: {
        const int input_index = output_socket.index();
        return GroupInputRef{input_index, current_elem};
      }
      case SH_NODE_MATH: {
        const int mode = current_node.custom1;
        const bNodeSocket &input_socket = current_node.input_socket(0);
        switch (mode) {
          case NODE_MATH_ADD:
          case NODE_MATH_SUBTRACT:
          case NODE_MATH_SNAP:
          case NODE_MATH_MINIMUM:
          case NODE_MATH_MAXIMUM:
          case NODE_MATH_ROUND:
          case NODE_MATH_FLOOR:
          case NODE_MATH_CEIL:
          case NODE_MATH_TRUNC:
            /* Those modes don't need special handling, because they don't affect the
             * derivative. */
            break;
          case NODE_MATH_MULTIPLY:
          case NODE_MATH_DIVIDE:
          case NODE_MATH_RADIANS:
          case NODE_MATH_DEGREES:
            /* We can compute the derivate of those nodes. */
            r_propagation_path.path.append({&current_node, current_elem});
            break;
          default:
            return std::nullopt;
        }
        return find_local_gizmo_target(input_socket, current_elem, r_propagation_path);
      }
      case SH_NODE_VECTOR_MATH: {
        const int mode = current_node.custom1;
        const bNodeSocket &input_socket = current_node.input_socket(0);
        switch (mode) {
          case NODE_VECTOR_MATH_ADD:
          case NODE_VECTOR_MATH_SUBTRACT:
          case NODE_VECTOR_MATH_SNAP:
          case NODE_VECTOR_MATH_MINIMUM:
          case NODE_VECTOR_MATH_MAXIMUM:
          case NODE_VECTOR_MATH_FLOOR:
          case NODE_VECTOR_MATH_CEIL:
            break;
          case NODE_VECTOR_MATH_MULTIPLY:
          case NODE_VECTOR_MATH_DIVIDE:
          case NODE_VECTOR_MATH_SCALE:
            r_propagation_path.path.append({&current_node, current_elem});
            break;
          default:
            return std::nullopt;
        }
        return find_local_gizmo_target(input_socket, current_elem, r_propagation_path);
      }
      case SH_NODE_MAP_RANGE: {
        const eCustomDataType data_type = eCustomDataType(
            static_cast<NodeMapRange *>(current_node.storage)->data_type);
        const bNodeSocket &input_socket = current_node.input_by_identifier(
            data_type == CD_PROP_FLOAT ? "Value" : "Vector");
        r_propagation_path.path.append({&current_node, current_elem});
        return find_local_gizmo_target(input_socket, current_elem, r_propagation_path);
      }
      default: {
        return std::nullopt;
      }
    }
  }
}

static void add_gizmo_input_target_pair(GizmoPropagationResult &inferencing_result,
                                        const InputSocketRef &gizmo_input,
                                        const GizmoTarget &gizmo_target)
{
  if (const auto *ref = std::get_if<ValueNodeRef>(&gizmo_target)) {
    inferencing_result.gizmo_inputs_for_value_nodes.add(ref->value_node, gizmo_input);
  }
  else if (const auto *ref = std::get_if<InputSocketRef>(&gizmo_target)) {
    inferencing_result.gizmo_inputs_for_node_inputs.add(ref->input_socket, gizmo_input);
  }
  else if (const auto *ref = std::get_if<GroupInputRef>(&gizmo_target)) {
    inferencing_result.gizmo_inputs_for_interface_inputs.add({ref->input_index, ref->elem},
                                                             gizmo_input);
  }
}

static GizmoPropagationResult propagate_gizmos(const bNodeTree &tree)
{
  GizmoPropagationResult inferencing_result;

  for (const bNodeSocket *socket : tree.all_sockets()) {
    socket->runtime->has_gizmo = false;
  }

  for (const bNode *group_node : tree.group_nodes()) {
    const bNodeTree *group = reinterpret_cast<const bNodeTree *>(group_node->id);
    if (group == nullptr) {
      continue;
    }
    if (!group->runtime->gizmo_inferencing) {
      continue;
    }
    for (const GroupInputRef group_input_ref :
         group->runtime->gizmo_inferencing->gizmo_inputs_for_interface_inputs.keys())
    {
      const bNodeSocket &input_socket = group_node->input_socket(group_input_ref.input_index);
      const InputSocketRef gizmo_input{&input_socket, group_input_ref.elem};

      LocalPropagationPath propagation_path;
      if (const std::optional<GizmoTarget> gizmo_target_opt = find_local_gizmo_target(
              input_socket, group_input_ref.elem, propagation_path))
      {
        add_gizmo_input_target_pair(inferencing_result, gizmo_input, *gizmo_target_opt);
      }
    }
    if (group->runtime->gizmo_inferencing->gizmo_inputs_for_interface_inputs.size() > 0) {
      inferencing_result.nodes_with_gizmos_inside.append(group_node);
    }
  }

  for (const StringRefNull idname : {"GeometryNodeGizmoArrow", "GeometryNodeGizmoDial"}) {
    for (const bNode *gizmo_node : tree.nodes_by_type(idname)) {
      const bNodeSocket &gizmo_value_input = gizmo_node->input_socket(0);
      gizmo_value_input.runtime->has_gizmo = true;
      for (const bNodeLink *link : gizmo_value_input.directly_linked_links()) {
        if (!is_valid_gizmo_link(*link)) {
          continue;
        }
        LocalPropagationPath propagation_path;
        if (const std::optional<GizmoTarget> gizmo_target = find_local_gizmo_target(
                *link->fromsock, ValueElem{}, propagation_path))
        {
          add_gizmo_input_target_pair(
              inferencing_result, InputSocketRef{&gizmo_value_input, ValueElem{}}, *gizmo_target);
        }
      }
      inferencing_result.nodes_with_gizmos_inside.append(gizmo_node);
    }
  }

  return inferencing_result;
}

std::ostream &operator<<(std::ostream &stream, const GizmoPropagationResult &data)
{
  stream << "Interface inputs with gizmo:\n";
  for (const auto item : data.gizmo_inputs_for_interface_inputs.items()) {
    stream << "  Input Index: " << item.key.input_index;
    if (item.key.elem.index.has_value()) {
      stream << ", Elem Index: " << *item.key.elem.index;
    }
    stream << "\n";
    for (const InputSocketRef &input_ref : item.value) {
      stream << "    " << input_ref.input_socket->owner_node().name << " -> "
             << input_ref.input_socket->name;
      if (input_ref.elem.index.has_value()) {
        stream << ", Elem Index: " << *input_ref.elem.index;
      }
      stream << "\n";
    }
  }
  return stream;
}

bool operator==(const GizmoPropagationResult &a, const GizmoPropagationResult &b)
{
  return a.nodes_with_gizmos_inside == b.nodes_with_gizmos_inside &&
         a.gizmo_inputs_for_value_nodes == b.gizmo_inputs_for_value_nodes &&
         a.gizmo_inputs_for_node_inputs == b.gizmo_inputs_for_node_inputs &&
         a.gizmo_inputs_for_interface_inputs == b.gizmo_inputs_for_interface_inputs;
}

bool operator!=(const GizmoPropagationResult &a, const GizmoPropagationResult &b)
{
  return !(a == b);
}

bool update_gizmo_propagation(bNodeTree &tree)
{
  tree.ensure_topology_cache();
  if (tree.has_available_link_cycle()) {
    const bool changed = tree.runtime->gizmo_inferencing.get() != nullptr;
    tree.runtime->gizmo_inferencing.reset();
    return changed;
  }

  GizmoPropagationResult result = propagate_gizmos(tree);
  const bool changed = tree.runtime->gizmo_inferencing ?
                           *tree.runtime->gizmo_inferencing != result :
                           true;
  tree.runtime->gizmo_inferencing = std::make_unique<GizmoPropagationResult>(std::move(result));
  return changed;
}

static void foreach_gizmo_for_target(
    const GizmoTarget &gizmo_target,
    ComputeContextBuilder &compute_context_builder,
    const bNodeTree &tree,
    FunctionRef<void(const ComputeContext &compute_context, const bNode &gizmo_node)> fn);

static void foreach_gizmo_for_input(
    const InputSocketRef &input_ref,
    ComputeContextBuilder &compute_context_builder,
    const bNodeTree &tree,
    FunctionRef<void(const ComputeContext &compute_context, const bNode &gizmo_node)> fn)
{
  const bke::bNodeTreeZones *zones = tree.zones();
  if (zones == nullptr) {
    return;
  }
  const bNode &node = input_ref.input_socket->owner_node();
  if (zones->get_zone_by_node(node.identifier) != nullptr) {
    /* Gizmos in zones are not supported yet. */
    return;
  }
  if (ELEM(node.type, GEO_NODE_GIZMO_ARROW, GEO_NODE_GIZMO_DIAL)) {
    fn(*compute_context_builder.current(), node);
  }
  else if (node.is_group()) {
    const GroupInputRef group_input_ref{input_ref.input_socket->index(), input_ref.elem};
    const bNodeTree &group = *reinterpret_cast<const bNodeTree *>(node.id);
    group.ensure_topology_cache();
    compute_context_builder.push<bke::GroupNodeComputeContext>(node, tree);
    foreach_gizmo_for_target(group_input_ref, compute_context_builder, group, fn);
    compute_context_builder.pop();
  }
}

static void foreach_gizmo_for_target(
    const GizmoTarget &gizmo_target,
    ComputeContextBuilder &compute_context_builder,
    const bNodeTree &tree,
    FunctionRef<void(const ComputeContext &compute_context, const bNode &gizmo_node)> fn)
{
  const GizmoPropagationResult &gizmo_inferencing = *tree.runtime->gizmo_inferencing;

  if (const auto *ref = std::get_if<ValueNodeRef>(&gizmo_target)) {
    for (const InputSocketRef &gizmo_input :
         gizmo_inferencing.gizmo_inputs_for_value_nodes.lookup(ref->value_node))
    {
      foreach_gizmo_for_input(gizmo_input, compute_context_builder, tree, fn);
    }
  }
  else if (const auto *ref = std::get_if<InputSocketRef>(&gizmo_target)) {
    for (const InputSocketRef &gizmo_input :
         gizmo_inferencing.gizmo_inputs_for_node_inputs.lookup(ref->input_socket))
    {
      foreach_gizmo_for_input(gizmo_input, compute_context_builder, tree, fn);
    }
  }
  else if (const auto *ref = std::get_if<GroupInputRef>(&gizmo_target)) {
    for (const InputSocketRef &gizmo_input :
         gizmo_inferencing.gizmo_inputs_for_interface_inputs.lookup({ref->input_index, ref->elem}))
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

          object_and_modifier->nmd->node_group->ensure_topology_cache();

          ComputeContextBuilder compute_context_builder;
          compute_context_builder.push<bke::ModifierComputeContext>(nmd.modifier.name);
          if (!ed::space_node::push_compute_context_for_tree_path(snode, compute_context_builder))
          {
            continue;
          }
          snode.edittree->ensure_topology_cache();
          const GizmoPropagationResult &gizmo_inferencing =
              *snode.edittree->runtime->gizmo_inferencing;
          Set<InputSocketRef> used_gizmo_inputs;
          for (auto item : gizmo_inferencing.gizmo_inputs_for_value_nodes.items()) {
            const bNode &node = *item.key;
            if ((node.flag & NODE_SELECT) ||
                (node.type == SH_NODE_VALUE && node.output_socket(0).flag & SOCK_GIZMO_PIN))
            {
              used_gizmo_inputs.add_multiple(item.value);
            }
          }
          for (auto item : gizmo_inferencing.gizmo_inputs_for_node_inputs.items()) {
            const bNode &node = item.key->owner_node();
            if ((node.flag & NODE_SELECT) || (item.key->flag & SOCK_GIZMO_PIN)) {
              used_gizmo_inputs.add_multiple(item.value);
            }
          }
          for (const bNode *node : gizmo_inferencing.nodes_with_gizmos_inside) {
            if (ELEM(node->type, GEO_NODE_GIZMO_ARROW, GEO_NODE_GIZMO_DIAL)) {
              fn(*compute_context_builder.current(), *node);
            }
            else if (node->is_group()) {
              if (!(node->flag & NODE_SELECT)) {
                continue;
              }
              const bNodeTree *group = reinterpret_cast<const bNodeTree *>(node->id);
              for (const auto item :
                   group->runtime->gizmo_inferencing->gizmo_inputs_for_interface_inputs.items())
              {
                const InputSocketRef gizmo_input{&node->input_socket(item.key.input_index),
                                                 item.key.elem};
                used_gizmo_inputs.add(gizmo_input);
              }
            }
          }
          for (const InputSocketRef &gizmo_input : used_gizmo_inputs) {
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
    const GizmoPropagationResult &gizmo_inferencing = *tree.runtime->gizmo_inferencing;

    ComputeContextBuilder compute_context_builder;
    compute_context_builder.push<bke::ModifierComputeContext>(nmd.modifier.name);

    for (const Span<InputSocketRef> gizmo_inputs :
         gizmo_inferencing.gizmo_inputs_for_interface_inputs.values())
    {
      for (const InputSocketRef &gizmo_input : gizmo_inputs) {
        foreach_gizmo_for_input(gizmo_input, compute_context_builder, tree, fn);
      }
    }
  }
}

static std::optional<GizmoTarget> find_propagated_gizmo_target_recursive(
    const bNodeSocket &gizmo_socket,
    const ValueElem &elem,
    const ComputeContext &compute_context,
    PropagationPath &r_path)
{
  LocalPropagationPath local_propagation_path;
  std::optional<GizmoTarget> gizmo_target_opt = find_local_gizmo_target(
      gizmo_socket, elem, local_propagation_path);
  if (!gizmo_target_opt) {
    return std::nullopt;
  }

  for (const LocalPropagationPath::PathElem &local_path_elem : local_propagation_path.path) {
    r_path.path.append({local_path_elem.node, local_path_elem.elem, &compute_context});
  }

  if (const auto *ref = std::get_if<InputSocketRef>(&*gizmo_target_opt)) {
    return *ref;
  }
  if (const auto *ref = std::get_if<ValueNodeRef>(&*gizmo_target_opt)) {
    return *ref;
  }
  if (const auto *ref = std::get_if<GroupInputRef>(&*gizmo_target_opt)) {
    if (dynamic_cast<const bke::ModifierComputeContext *>(&compute_context)) {
      return *ref;
    }
    if (const auto *group_node_compute_context =
            dynamic_cast<const bke::GroupNodeComputeContext *>(&compute_context))
    {
      const bNode *caller_node = group_node_compute_context->caller_group_node();
      const bNodeSocket &socket = caller_node->input_socket(ref->input_index);
      return find_propagated_gizmo_target_recursive(
          socket, ref->elem, *compute_context.parent(), r_path);
    }
  }

  return std::nullopt;
}

Vector<PropagatedGizmoTarget> find_propagated_gizmo_targets(const ComputeContext &compute_context,
                                                            const bNode &gizmo_node)
{
  Vector<PropagatedGizmoTarget> propagated_gizmo_targets;
  const bNodeSocket &gizmo_value_input = gizmo_node.input_socket(0);
  for (const bNodeLink *link : gizmo_value_input.directly_linked_links()) {
    if (!is_valid_gizmo_link(*link)) {
      continue;
    }
    PropagationPath path;
    if (std::optional<GizmoTarget> target = find_propagated_gizmo_target_recursive(
            *link->fromsock, ValueElem{}, compute_context, path))
    {
      propagated_gizmo_targets.append({std::move(*target), std::move(path)});
    }
  }
  return propagated_gizmo_targets;
}

}  // namespace blender::nodes::gizmos
