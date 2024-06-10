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

struct GlobalInverseEvalPath {
  struct Node {
    const ComputeContext *compute_context;
    const bNode *node;
  };

  Vector<Node> ordered_nodes;
};

GlobalInverseEvalPath find_global_inverse_eval_path(const ComputeContext *initial_context,
                                                    const SocketElem &initial_socket_elem);

}  // namespace blender::nodes::inverse_eval
