/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_compute_context.hh"
#include "BLI_function_ref.hh"
#include "BLI_multi_value_map.hh"

#include "NOD_inverse_eval_path.hh"

struct Object;
struct NodesModifierData;
struct wmWindowManager;

namespace blender::nodes::gizmos2 {

namespace ie = inverse_eval;

struct TreeGizmoPropagation {
  Vector<const bNode *> nodes_containing_gizmos;
  MultiValueMap<ie::ValueNodeElem, ie::SocketElem> gizmo_inputs_by_value_nodes;
  MultiValueMap<ie::SocketElem, ie::SocketElem> gizmo_inputs_by_node_inputs;
  MultiValueMap<ie::GroupInputElem, ie::SocketElem> gizmo_inputs_by_group_inputs;

  BLI_STRUCT_EQUALITY_OPERATORS_4(TreeGizmoPropagation,
                                  nodes_containing_gizmos,
                                  gizmo_inputs_by_value_nodes,
                                  gizmo_inputs_by_node_inputs,
                                  gizmo_inputs_by_group_inputs)
};

bool is_builtin_gizmo_node(const bNode &node);

bool update_tree_gizmo_propagation(bNodeTree &tree);

using ForeachGizmoFn =
    FunctionRef<void(const ComputeContext &compute_context, const bNode &gizmo_node)>;

void foreach_active_gizmo(const Object &object,
                          const NodesModifierData &nmd,
                          const wmWindowManager &wm,
                          ForeachGizmoFn fn);

}  // namespace blender::nodes::gizmos2
