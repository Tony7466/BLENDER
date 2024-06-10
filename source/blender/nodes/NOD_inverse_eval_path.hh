/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "NOD_inverse_eval.hh"

namespace blender::nodes::inverse_eval {

struct LocalInversePropagationPath {
  Vector<SocketElem> intermediate_sockets;

  Vector<SocketElem> final_input_sockets;
  Vector<GroupInputElem> final_group_inputs;
  Vector<ValueNodeElem> final_value_nodes;
};

LocalInversePropagationPath find_local_inverse_propagation_path(
    const bNodeTree &tree, const SocketElem &initial_socket_elem);

}  // namespace blender::nodes::inverse_eval
