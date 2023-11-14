/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>
#include <variant>

#include "BLI_compute_context.hh"
#include "BLI_function_ref.hh"
#include "BLI_multi_value_map.hh"
#include "BLI_vector.hh"

#include "DNA_node_types.h"

struct wmWindowManager;
struct NodesModifierData;

namespace blender::nodes::gizmos {

struct SocketElem {
  std::optional<int> index;

  friend bool operator==(const SocketElem &a, const SocketElem &b)
  {
    return a.index == b.index;
  }

  uint64_t hash() const
  {
    return get_default_hash(this->index.value_or(0));
  }
};

struct InputSocketGizmoSource {
  const bNodeSocket *input_socket;
  SocketElem elem;
};

struct ValueNodeGizmoSource {
  const bNode *value_node;
  SocketElem elem;

  friend bool operator==(const ValueNodeGizmoSource &a, const ValueNodeGizmoSource &b)
  {
    return a.value_node == b.value_node && a.elem == b.elem;
  }

  uint64_t hash() const
  {
    return get_default_hash_2(this->value_node, this->elem);
  }
};

struct GroupInputGizmoSource {
  int interface_input_index;
  SocketElem elem;
};

/**
 * A #GizmoSource is a place where a value controlled by a gizmo lives.
 */
using GizmoSource =
    std::variant<InputSocketGizmoSource, ValueNodeGizmoSource, GroupInputGizmoSource>;

/**
 * A #GizmoInput is an input socket that has a gizmo attached. It can also be the value input of a
 * gizmo node itself.
 */
struct GizmoInput {
  const bNodeSocket *input_socket;
  SocketElem elem;

  friend bool operator==(const GizmoInput &a, const GizmoInput &b)
  {
    return a.input_socket == b.input_socket && a.elem == b.elem;
  }

  uint64_t hash() const
  {
    return get_default_hash_2(this->input_socket, this->elem);
  }
};

struct InterfaceGizmoInput {
  int input_index;
  SocketElem elem;

  friend bool operator==(const InterfaceGizmoInput &a, const InterfaceGizmoInput &b)
  {
    return a.input_index == b.input_index && a.elem == b.elem;
  }

  uint64_t hash() const
  {
    return get_default_hash_2(this->input_index, this->elem);
  }
};

struct GizmoNodeSource {
  GizmoSource source;
};

struct GizmoInferencingResult {
  Vector<const bNode *> nodes_with_gizmos_inside;
  MultiValueMap<const bNode *, GizmoInput> gizmo_inputs_for_value_node;
  MultiValueMap<const bNodeSocket *, GizmoInput> gizmo_inputs_for_node_inputs;
  MultiValueMap<InterfaceGizmoInput, GizmoInput> gizmo_inputs_for_interface_input;

  friend std::ostream &operator<<(std::ostream &stream, const GizmoInferencingResult &data);
};

struct LocalGizmoPathElem {
  const bNode *node = nullptr;
};

/**
 * Find which value is controlled by a gizmo attached to the given socket. This function works
 * locally in the current node tree.
 */
std::optional<GizmoSource> find_local_gizmo_source(const bNodeSocket &socket,
                                                   const SocketElem &elem,
                                                   Vector<LocalGizmoPathElem> &right_to_left_path);

struct GlobalGizmoPathElem {
  const bNode *node;
  const ComputeContext *compute_context;
};

struct GlobalGizmoSource {
  GizmoSource source;
  Vector<GlobalGizmoPathElem> right_to_left_path;
};

Vector<GlobalGizmoSource> find_global_gizmo_sources(const ComputeContext &compute_context,
                                                    const bNode &gizmo_node);

bool is_valid_gizmo_link(const bNodeLink &link);

bool update_gizmo_inferencing(bNodeTree &tree);

void foreach_active_gizmo(
    const Object &object,
    const NodesModifierData &nmd,
    const wmWindowManager &wm,
    FunctionRef<void(const ComputeContext &compute_context, const bNode &gizmo_node)> fn);

}  // namespace blender::nodes::gizmos
