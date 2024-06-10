/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_multi_value_map.hh"

#include "NOD_inverse_eval_path.hh"

namespace blender::nodes::gizmos2 {

namespace ie = inverse_eval;

struct TreeGizmos {
  Vector<const bNode *> nodes_containing_gizmos;
  MultiValueMap<ie::ValueNodeElem, ie::SocketElem> gizmo_inputs_by_value_nodes;
  MultiValueMap<ie::SocketElem, ie::SocketElem> gizmo_inputs_by_node_inputs;
  MultiValueMap<ie::GroupInputElem, ie::SocketElem> gizmo_inputs_by_group_inputs;
};

}  // namespace blender::nodes::gizmos2
