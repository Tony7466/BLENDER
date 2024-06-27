/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_compute_context.hh"
#include "BLI_function_ref.hh"
#include "BLI_multi_value_map.hh"

#include "NOD_inverse_eval_path.hh"
#include "NOD_inverse_eval_run.hh"

struct Object;
struct NodesModifierData;
struct wmWindowManager;

namespace blender::nodes::gizmos {

namespace ie = inverse_eval;

struct TreeGizmoPropagation {
  Vector<const bNode *> nodes_containing_gizmos;
  Vector<const bNode *> gizmo_nodes;
  Set<const bNodeSocket *> gizmo_endpoint_sockets;
  MultiValueMap<ie::ValueNodeElem, ie::SocketElem> gizmo_inputs_by_value_nodes;
  MultiValueMap<ie::SocketElem, ie::SocketElem> gizmo_inputs_by_node_inputs;
  MultiValueMap<ie::GroupInputElem, ie::SocketElem> gizmo_inputs_by_group_inputs;

  BLI_STRUCT_EQUALITY_OPERATORS_5(TreeGizmoPropagation,
                                  nodes_containing_gizmos,
                                  gizmo_nodes,
                                  gizmo_inputs_by_value_nodes,
                                  gizmo_inputs_by_node_inputs,
                                  gizmo_inputs_by_group_inputs)
};

bool is_builtin_gizmo_node(const bNode &node);

bool update_tree_gizmo_propagation(bNodeTree &tree);

using ForeachGizmoInModifierFn = FunctionRef<void(const ComputeContext &compute_context,
                                                  const bNode &gizmo_node,
                                                  const bNodeSocket &gizmo_socket)>;

void foreach_active_gizmo_in_modifier(const Object &object,
                                      const NodesModifierData &nmd,
                                      const wmWindowManager &wm,
                                      ComputeContextBuilder &compute_context_builder,
                                      ForeachGizmoInModifierFn fn);

using ForeachGizmoFn = FunctionRef<void(const Object &object,
                                        const NodesModifierData &nmd,
                                        const ComputeContext &compute_context,
                                        const bNode &gizmo_node,
                                        const bNodeSocket &gizmo_socket)>;

void foreach_active_gizmo(const bContext &C,
                          ComputeContextBuilder &compute_context_builder,
                          ForeachGizmoFn fn);

void foreach_compute_context_on_gizmo_path(const ComputeContext &gizmo_context,
                                           const bNode &gizmo_node,
                                           const bNodeSocket &gizmo_socket,
                                           FunctionRef<void(const ComputeContext &context)> fn);
void foreach_socket_on_gizmo_path(const ComputeContext &gizmo_context,
                                  const bNode &gizmo_node,
                                  const bNodeSocket &gizmo_socket,
                                  FunctionRef<void(const ComputeContext &context,
                                                   const bNodeSocket &socket,
                                                   const ie::ElemVariant &elem)> fn);
ie::ElemVariant get_editable_gizmo_elem(const ComputeContext &gizmo_context,
                                        const bNode &gizmo_node,
                                        const bNodeSocket &gizmo_socket);

void apply_gizmo_change(bContext &C,
                        Object &object,
                        NodesModifierData &nmd,
                        geo_eval_log::GeoModifierLog &eval_log,
                        const ComputeContext &gizmo_context,
                        const bNodeSocket &gizmo_socket,
                        FunctionRef<void(bke::SocketValueVariant &value)> apply_on_gizmo_value_fn);

}  // namespace blender::nodes::gizmos
