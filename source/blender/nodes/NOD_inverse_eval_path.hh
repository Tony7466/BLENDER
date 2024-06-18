/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_compute_context.hh"

#include "NOD_inverse_eval.hh"

namespace blender::nodes::inverse_eval {

struct LocalInverseEvalPath {
  Vector<SocketElem> intermediate_sockets;

  Vector<SocketElem> final_input_sockets;
  Vector<GroupInputElem> final_group_inputs;
  Vector<ValueNodeElem> final_value_nodes;
};

LocalInverseEvalPath find_local_inverse_eval_path(const bNodeTree &tree,
                                                  const SocketElem &initial_socket_elem);

void foreach_node_on_inverse_eval_path(
    const ComputeContext &initial_context,
    const SocketElem &initial_socket_elem,
    FunctionRef<void(const ComputeContext &context, const bNode &node)> fn);

std::optional<ElemVariant> convert_socket_elem(const bNodeSocket &old_socket,
                                               const bNodeSocket &new_socket,
                                               const ElemVariant &old_elem);

}  // namespace blender::nodes::inverse_eval
