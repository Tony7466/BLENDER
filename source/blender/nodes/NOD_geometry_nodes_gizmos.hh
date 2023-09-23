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

  friend bool operator==(const ValueNodeGizmoSource &a, const ValueNodeGizmoSource &b)
  {
    return a.value_node == b.value_node && a.elem_index == b.elem_index;
  }

  uint64_t hash() const
  {
    return get_default_hash_2(this->value_node, this->elem_index.value_or(0));
  }
};

struct GroupInputGizmoSource {
  int interface_input_index;
  std::optional<int> elem_index;
};

using GizmoSource =
    std::variant<InputSocketGizmoSource, ValueNodeGizmoSource, GroupInputGizmoSource>;

struct GizmoInput {
  const bNodeSocket *input_socket;
  std::optional<int> elem_index;
};

struct InterfaceGizmoInput {
  int input_index;
  std::optional<int> elem_index;

  friend bool operator==(const InterfaceGizmoInput &a, const InterfaceGizmoInput &b)
  {
    return a.input_index == b.input_index && a.elem_index == b.elem_index;
  }

  uint64_t hash() const
  {
    return get_default_hash_2(this->input_index, this->elem_index.value_or(0));
  }
};

struct GizmoInferencingResult {
  MultiValueMap<const bNode *, GizmoInput> gizmo_inputs_for_value_node;
  MultiValueMap<InterfaceGizmoInput, GizmoInput> gizmo_inputs_for_interface_input;
};

std::optional<GizmoSource> find_gizmo_source(const bNodeSocket &socket,
                                             std::optional<int> elem_index);

bool update_gizmo_inferencing(bNodeTree &tree);

}  // namespace blender::nodes::gizmos
