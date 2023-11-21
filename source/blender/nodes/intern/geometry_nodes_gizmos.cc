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

struct LocalGizmoPathElem {
  const bNode *node = nullptr;
  SocketElem elem;
};

static std::optional<GizmoSource> find_local_gizmo_source_recursive(
    const bNodeSocket &current_socket,
    const SocketElem &current_elem,
    Vector<LocalGizmoPathElem> &right_to_left_path)
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
        return ValueNodeRef{&current_node, current_elem};
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
            right_to_left_path.append({&current_node, current_elem});
            break;
          default:
            return std::nullopt;
        }
        return find_local_gizmo_source_recursive(input_socket, current_elem, right_to_left_path);
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
            right_to_left_path.append({&current_node, current_elem});
            break;
          default:
            return std::nullopt;
        }
        return find_local_gizmo_source_recursive(input_socket, current_elem, right_to_left_path);
      }
      case SH_NODE_MAP_RANGE: {
        const eCustomDataType data_type = eCustomDataType(
            static_cast<NodeMapRange *>(current_node.storage)->data_type);
        const bNodeSocket &input_socket = current_node.input_by_identifier(
            data_type == CD_PROP_FLOAT ? "Value" : "Vector");
        right_to_left_path.append({&current_node, current_elem});
        return find_local_gizmo_source_recursive(input_socket, current_elem, right_to_left_path);
      }
      default: {
        return std::nullopt;
      }
    }
  }
}

static std::optional<GizmoSource> find_local_gizmo_source(
    const bNodeSocket &socket,
    const SocketElem &elem,
    Vector<LocalGizmoPathElem> &right_to_left_path)
{
  return find_local_gizmo_source_recursive(socket, elem, right_to_left_path);
}

static void add_gizmo_input_source_pair(GizmoInferencingResult &inferencing_result,
                                        const InputSocketRef &gizmo_input,
                                        const GizmoSource &gizmo_source)
{
  if (const auto *ref = std::get_if<ValueNodeRef>(&gizmo_source)) {
    inferencing_result.gizmo_inputs_for_value_nodes.add(ref->value_node, gizmo_input);
  }
  else if (const auto *ref = std::get_if<InputSocketRef>(&gizmo_source)) {
    inferencing_result.gizmo_inputs_for_node_inputs.add(ref->input_socket, gizmo_input);
  }
  else if (const auto *ref = std::get_if<GroupInputRef>(&gizmo_source)) {
    inferencing_result.gizmo_inputs_for_interface_inputs.add({ref->input_index, ref->elem},
                                                             gizmo_input);
  }
}

static GizmoInferencingResult compute_gizmo_inferencing_result(const bNodeTree &tree)
{
  GizmoInferencingResult inferencing_result;

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

      Vector<LocalGizmoPathElem> gizmo_path;
      if (const std::optional<GizmoSource> gizmo_source_opt = find_local_gizmo_source(
              input_socket, group_input_ref.elem, gizmo_path))
      {
        add_gizmo_input_source_pair(inferencing_result, gizmo_input, *gizmo_source_opt);
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
        Vector<LocalGizmoPathElem> gizmo_path;
        if (const std::optional<GizmoSource> gizmo_source = find_local_gizmo_source(
                *link->fromsock, SocketElem{}, gizmo_path))
        {
          add_gizmo_input_source_pair(
              inferencing_result, InputSocketRef{&gizmo_value_input, SocketElem{}}, *gizmo_source);
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

bool operator==(const GizmoInferencingResult &a, const GizmoInferencingResult &b)
{
  return a.nodes_with_gizmos_inside == b.nodes_with_gizmos_inside &&
         a.gizmo_inputs_for_value_nodes == b.gizmo_inputs_for_value_nodes &&
         a.gizmo_inputs_for_node_inputs == b.gizmo_inputs_for_node_inputs &&
         a.gizmo_inputs_for_interface_inputs == b.gizmo_inputs_for_interface_inputs;
}

bool operator!=(const GizmoInferencingResult &a, const GizmoInferencingResult &b)
{
  return !(a == b);
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
  const bool changed = tree.runtime->gizmo_inferencing ?
                           *tree.runtime->gizmo_inferencing != result :
                           true;
  tree.runtime->gizmo_inferencing = std::make_unique<GizmoInferencingResult>(std::move(result));
  return changed;
}

static void foreach_gizmo_for_source(
    const GizmoSource &gizmo_source,
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
    foreach_gizmo_for_source(group_input_ref, compute_context_builder, group, fn);
    compute_context_builder.pop();
  }
}

static void foreach_gizmo_for_source(
    const GizmoSource &gizmo_source,
    ComputeContextBuilder &compute_context_builder,
    const bNodeTree &tree,
    FunctionRef<void(const ComputeContext &compute_context, const bNode &gizmo_node)> fn)
{
  const GizmoInferencingResult &gizmo_inferencing = *tree.runtime->gizmo_inferencing;

  if (const auto *ref = std::get_if<ValueNodeRef>(&gizmo_source)) {
    for (const InputSocketRef &gizmo_input :
         gizmo_inferencing.gizmo_inputs_for_value_nodes.lookup(ref->value_node))
    {
      foreach_gizmo_for_input(gizmo_input, compute_context_builder, tree, fn);
    }
  }
  else if (const auto *ref = std::get_if<InputSocketRef>(&gizmo_source)) {
    for (const InputSocketRef &gizmo_input :
         gizmo_inferencing.gizmo_inputs_for_node_inputs.lookup(ref->input_socket))
    {
      foreach_gizmo_for_input(gizmo_input, compute_context_builder, tree, fn);
    }
  }
  else if (const auto *ref = std::get_if<GroupInputRef>(&gizmo_source)) {
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
          const GizmoInferencingResult &gizmo_inferencing =
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
    const GizmoInferencingResult &gizmo_inferencing = *tree.runtime->gizmo_inferencing;

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

static std::optional<GizmoSource> find_global_gizmo_source_recursive(
    const bNodeSocket &gizmo_socket,
    const SocketElem &elem,
    const ComputeContext &compute_context,
    Vector<GlobalGizmoPathElem> &r_path)
{
  Vector<LocalGizmoPathElem> local_path;
  std::optional<GizmoSource> gizmo_source_opt = find_local_gizmo_source(
      gizmo_socket, elem, local_path);
  if (!gizmo_source_opt) {
    return std::nullopt;
  }

  for (const LocalGizmoPathElem &local_path_elem : local_path) {
    r_path.append({local_path_elem.node, local_path_elem.elem, &compute_context});
  }

  if (const auto *ref = std::get_if<InputSocketRef>(&*gizmo_source_opt)) {
    return *ref;
  }
  if (const auto *ref = std::get_if<ValueNodeRef>(&*gizmo_source_opt)) {
    return *ref;
  }
  if (const auto *ref = std::get_if<GroupInputRef>(&*gizmo_source_opt)) {
    if (dynamic_cast<const bke::ModifierComputeContext *>(&compute_context)) {
      return *ref;
    }
    if (const auto *group_node_compute_context =
            dynamic_cast<const bke::GroupNodeComputeContext *>(&compute_context))
    {
      const bNode *caller_node = group_node_compute_context->caller_group_node();
      const bNodeSocket &socket = caller_node->input_socket(ref->input_index);
      return find_global_gizmo_source_recursive(
          socket, ref->elem, *compute_context.parent(), r_path);
    }
  }

  return std::nullopt;
}

Vector<GlobalGizmoSource> find_global_gizmo_sources(const ComputeContext &compute_context,
                                                    const bNode &gizmo_node)
{
  Vector<GlobalGizmoSource> global_gizmo_sources;
  const bNodeSocket &gizmo_value_input = gizmo_node.input_socket(0);
  for (const bNodeLink *link : gizmo_value_input.directly_linked_links()) {
    if (!is_valid_gizmo_link(*link)) {
      continue;
    }
    Vector<GlobalGizmoPathElem> path;
    if (std::optional<GizmoSource> source = find_global_gizmo_source_recursive(
            *link->fromsock, SocketElem{}, compute_context, path))
    {
      global_gizmo_sources.append({std::move(*source), std::move(path)});
    }
  }
  return global_gizmo_sources;
}

}  // namespace blender::nodes::gizmos
