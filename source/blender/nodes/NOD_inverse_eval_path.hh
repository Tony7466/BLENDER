/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_compute_context.hh"

#include "NOD_inverse_eval.hh"
#include "NOD_value_elem.hh"

namespace blender::nodes::inverse_eval {

using namespace value_elem;

struct LocalInverseEvalTargets {
  Vector<SocketElem> input_sockets;
  Vector<GroupInputElem> group_inputs;
  Vector<ValueNodeElem> value_nodes;
};

LocalInverseEvalTargets find_local_inverse_eval_targets(const bNodeTree &tree,
                                                        const SocketElem &initial_socket_elem);

void foreach_element_on_inverse_eval_path(
    const ComputeContext &initial_context,
    const SocketElem &initial_socket_elem,
    FunctionRef<void(const ComputeContext &context)> foreach_context_fn,
    FunctionRef<void(const ComputeContext &context,
                     const bNodeSocket &socket,
                     const ElemVariant &elem)> foreach_socket_fn);

}  // namespace blender::nodes::inverse_eval
