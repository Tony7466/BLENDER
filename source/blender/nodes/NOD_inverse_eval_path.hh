/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "NOD_inverse_eval.hh"

namespace blender::nodes::inverse_eval {

struct PropagationPathNode {
  Vector<SocketElem> inputs;
  Vector<SocketElem> outputs;
};

struct PropagationPath {
  Map<const bNode *, PropagationPathNode> nodes;
  Vector<SocketElem> final_socket_elems;
};

PropagationPath find_propagation_path(const bNodeTree &tree,
                                      const SocketElem &initial_socket_elem);

}  // namespace blender::nodes::inverse_eval
