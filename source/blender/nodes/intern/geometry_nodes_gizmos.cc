/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_base_safe.h"
#include "BLI_math_rotation.hh"

#include "BKE_compute_contexts.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_zones.hh"
#include "BKE_workspace.hh"

#include "NOD_geometry_nodes_gizmos.hh"
#include "NOD_inverse_eval.hh"
#include "NOD_inverse_eval_path.hh"

#include "DNA_modifier_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

namespace blender::nodes::gizmos {

bool is_builtin_gizmo_node(const bNode &node)
{
  return ELEM(node.type, GEO_NODE_GIZMO_LINEAR, GEO_NODE_GIZMO_DIAL, GEO_NODE_GIZMO_TRANSFORM);
}

static void reset_gizmo_states(bNodeTree &tree)
{
  for (bNodeSocket *socket : tree.all_sockets()) {
    socket->runtime->has_gizmo = false;
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
    if (is_builtin_gizmo_node(*node)) {
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
    gizmo_input.gizmo_socket->runtime->has_gizmo = true;
    const ie::SocketElem gizmo_input_socket_elem{gizmo_input.gizmo_socket, gizmo_input.elem};
    const std::optional<ie::ElemVariant> converted_elem = ie::convert_socket_elem(
        *gizmo_input.gizmo_socket, *gizmo_input.propagation_start_socket, gizmo_input.elem);
    if (!converted_elem) {
      continue;
    }
    const ie::LocalInverseEvalPath path = ie::find_local_inverse_eval_path(
        tree, {gizmo_input.propagation_start_socket, *converted_elem});
    const bool has_target = !path.final_input_sockets.is_empty() ||
                            !path.final_group_inputs.is_empty() ||
                            !path.final_value_nodes.is_empty();
    if (!has_target) {
      continue;
    }
    for (const ie::SocketElem &socket_elem : path.intermediate_sockets) {
      socket_elem.socket->runtime->has_gizmo = true;
    }
    for (const ie::SocketElem &input_socket : path.final_input_sockets) {
      gizmo_propagation.gizmo_inputs_by_node_inputs.add(input_socket, gizmo_input_socket_elem);
    }
    for (const ie::ValueNodeElem &value_node : path.final_value_nodes) {
      gizmo_propagation.gizmo_inputs_by_value_nodes.add(value_node, gizmo_input_socket_elem);
    }
    for (const ie::GroupInputElem &group_input : path.final_group_inputs) {
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

static void foreach_gizmo_for_input(const ie::SocketElem &input_socket,
                                    ComputeContextBuilder &compute_context_builder,
                                    const bNodeTree &tree,
                                    const ForeachGizmoFn fn);

static void foreach_gizmo_for_group_input(const bNodeTree &tree,
                                          const ie::GroupInputElem &group_input,
                                          ComputeContextBuilder &compute_context_builder,
                                          const ForeachGizmoFn fn)
{
  const TreeGizmoPropagation &gizmo_propagation = *tree.runtime->gizmo_propagation;
  for (const ie::SocketElem &gizmo_input :
       gizmo_propagation.gizmo_inputs_by_group_inputs.lookup(group_input))
  {
    foreach_gizmo_for_input(gizmo_input, compute_context_builder, tree, fn);
  }
}

static void foreach_gizmo_for_input(const ie::SocketElem &input_socket,
                                    ComputeContextBuilder &compute_context_builder,
                                    const bNodeTree &tree,
                                    const ForeachGizmoFn fn)
{
  const bke::bNodeTreeZones *zones = tree.zones();
  if (!zones) {
    return;
  }
  const bNode &node = input_socket.socket->owner_node();
  if (zones->get_zone_by_node(node.identifier) != nullptr) {
    /* Gizmos in zones are not supportet yet. */
    return;
  }
  if (is_builtin_gizmo_node(node)) {
    /* Found an actual built-in gizmo node. */
    fn(*compute_context_builder.current(), node);
    return;
  }
  if (node.is_group()) {
    const bNodeTree &group = *reinterpret_cast<const bNodeTree *>(node.id);
    group.ensure_topology_cache();
    compute_context_builder.push<bke::GroupNodeComputeContext>(node, tree);
    foreach_gizmo_for_group_input(
        group,
        ie::GroupInputElem{input_socket.socket->index(), input_socket.elem},
        compute_context_builder,
        fn);
    compute_context_builder.pop();
  }
}

void foreach_active_gizmo(const Object & /*object*/,
                          const NodesModifierData &nmd,
                          const wmWindowManager & /*wm*/,
                          ComputeContextBuilder &compute_context_builder,
                          const ForeachGizmoFn fn)
{
  /* TODO: Find gizmos from open node editors. */

  if (!nmd.node_group) {
    return;
  }
  const bNodeTree &tree = *nmd.node_group;
  if (!tree.runtime->gizmo_propagation) {
    return;
  }
  compute_context_builder.push<bke::ModifierComputeContext>(nmd.modifier.name);
  for (auto &&item : tree.runtime->gizmo_propagation->gizmo_inputs_by_value_nodes.items()) {
    for (const ie::SocketElem &socket_elem : item.value) {
      foreach_gizmo_for_input(socket_elem, compute_context_builder, tree, fn);
    }
  }
  for (auto &&item : tree.runtime->gizmo_propagation->gizmo_inputs_by_node_inputs.items()) {
    for (const ie::SocketElem &socket_elem : item.value) {
      foreach_gizmo_for_input(socket_elem, compute_context_builder, tree, fn);
    }
  }
  for (auto &&item : tree.runtime->gizmo_propagation->gizmo_inputs_by_group_inputs.items()) {
    for (const ie::SocketElem &socket_elem : item.value) {
      foreach_gizmo_for_input(socket_elem, compute_context_builder, tree, fn);
    }
  }
}

ie::GlobalInverseEvalPath find_inverse_eval_path_for_gizmo(const ComputeContext *gizmo_context,
                                                           const bNode &gizmo_node)
{
  const bNodeSocket &gizmo_socket = gizmo_node.input_socket(0);
  return ie::find_global_inverse_eval_path(
      gizmo_context, {&gizmo_socket, get_gizmo_socket_elem(gizmo_node, gizmo_socket)});
}

void apply_gizmo_change(
    Object &object,
    NodesModifierData &nmd,
    geo_eval_log::GeoModifierLog &eval_log,
    const ComputeContext &gizmo_context,
    const bNodeSocket &gizmo_socket,
    const FunctionRef<void(bke::SocketValueVariant &value)> apply_on_gizmo_value_fn)
{
  const bNodeTree &gizmo_node_tree = gizmo_socket.owner_tree();
  geo_eval_log::GeoTreeLog &gizmo_tree_log = eval_log.get_tree_log(gizmo_context.hash());
  for (const bNodeLink *link : gizmo_socket.directly_linked_links()) {
    gizmo_node_tree.ensure_topology_cache();
    if (!link->is_used()) {
      continue;
    }
    if (link->fromnode->is_dangling_reroute()) {
      continue;
    }
    const std::optional<bke::SocketValueVariant> old_value = ie::get_logged_socket_value(
        gizmo_tree_log, *link->fromsock);
    if (!old_value) {
      continue;
    }
    const std::optional<bke::SocketValueVariant> old_value_converted = ie::convert_socket_value(
        *link->fromsock, *link->tosock, *old_value);
    if (!old_value_converted) {
      continue;
    }
    bke::SocketValueVariant new_value = *old_value_converted;
    apply_on_gizmo_value_fn(new_value);

    /* TODO: Call this for all modififed values at once. Otherwise, they might overwrite each
     * other. */
    ie::try_change_link_target_and_update_source(
        object, nmd, eval_log, &gizmo_context, *link, new_value);
  }
}

}  // namespace blender::nodes::gizmos
