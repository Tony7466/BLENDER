/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>
#include <variant>

#include "BLI_multi_value_map.hh"
#include "BLI_vector.hh"

#include "DNA_node_types.h"

namespace blender::nodes::gizmos {

struct InputSocketGizmoSource {
  const bNodeSocket *input_socket;
  std::optional<int> elem_index;
};

struct ValueNodeGizmoSource {
  const bNode *value_node;
  std::optional<int> elem_index;
};

struct GroupInputGizmoSource {
  int interface_input_index;
  std::optional<int> elem_index;
};

using GizmoSource =
    std::variant<InputSocketGizmoSource, ValueNodeGizmoSource, GroupInputGizmoSource>;

struct GizmoInferencingResult {
  MultiValueMap<const bNode *, const bNodeSocket *> gizmo_inputs_in_group_node;

  MultiValueMap<const bNode *, const bNodeSocket *> gizmo_inputs_by_value_node;
  MultiValueMap<int, const bNodeSocket *> gizmo_inputs_by_interface_input;
};

std::optional<GizmoSource> find_scalar_gizmo_source(const bNodeSocket &socket);

bool update_gizmo_inferencing(bNodeTree &tree);

}  // namespace blender::nodes::gizmos
