/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "NOD_node_in_compute_context.hh"

#include "BLI_function_ref.hh"
#include "BLI_resource_scope.hh"
#include "BLI_set.hh"

#include "DNA_node_types.h"

namespace blender::nodes::partial_eval {

void eval_downstream(
    const Span<SocketInContext> initial_sockets,
    ResourceScope &scope,
    FunctionRef<void(const NodeInContext &ctx_node,
                     Vector<const bNodeSocket *> &r_outputs_to_propagate)> evaluate_node_fn,
    FunctionRef<bool(const SocketInContext &ctx_from, const SocketInContext &ctx_to)>
        propagate_value_fn);

struct UpstreamEvalTargets {
  Set<SocketInContext> sockets;
  Set<NodeInContext> value_nodes;
  Set<SocketInContext> group_inputs;
};

UpstreamEvalTargets eval_upstream(
    const Span<SocketInContext> initial_sockets,
    ResourceScope &scope,
    FunctionRef<void(const NodeInContext &ctx_node,
                     Vector<const bNodeSocket *> &r_modified_inputs)> evaluate_node_fn,
    FunctionRef<bool(const SocketInContext &ctx_from, const SocketInContext &ctx_to)>
        propagate_value_fn,
    FunctionRef<void(const NodeInContext &ctx_node, Vector<const bNodeSocket *> &r_sockets)>
        get_inputs_to_propagate_fn);

}  // namespace blender::nodes::partial_eval
