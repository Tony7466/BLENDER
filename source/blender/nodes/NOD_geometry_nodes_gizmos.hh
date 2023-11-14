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

/**
 * Often gizmos don't reference the whole socket value, but only a part of it. E.g. an arrow gizmo
 * may only control the Z value of a vector socket. In this case, the `index` is set to 2.
 */
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

/**
 * An input socket that can be modified by a gizmo.
 */
struct InputSocketRef {
  const bNodeSocket *input_socket;
  SocketElem elem;

  friend bool operator==(const InputSocketRef &a, const InputSocketRef &b)
  {
    return a.input_socket == b.input_socket && a.elem == b.elem;
  }

  uint64_t hash() const
  {
    return get_default_hash_2(this->input_socket, this->elem);
  }
};

/**
 * A value node that can be modified by a gizmo.
 */
struct ValueNodeRef {
  const bNode *value_node;
  SocketElem elem;

  friend bool operator==(const ValueNodeRef &a, const ValueNodeRef &b)
  {
    return a.value_node == b.value_node && a.elem == b.elem;
  }

  uint64_t hash() const
  {
    return get_default_hash_2(this->value_node, this->elem);
  }
};

/**
 * A group input that can be modified by a gizmo.
 */
struct GroupInputRef {
  int input_index;
  SocketElem elem;

  friend bool operator==(const GroupInputRef &a, const GroupInputRef &b)
  {
    return a.input_index == b.input_index && a.elem == b.elem;
  }

  uint64_t hash() const
  {
    return get_default_hash_2(this->input_index, this->elem);
  }
};

/**
 * Locates a value that can be modified by a gizmo.
 */
using GizmoSource = std::variant<InputSocketRef, ValueNodeRef, GroupInputRef>;

struct GizmoInferencingResult {
  Vector<const bNode *> nodes_with_gizmos_inside;
  MultiValueMap<const bNode *, InputSocketRef> gizmo_inputs_for_value_nodes;
  MultiValueMap<const bNodeSocket *, InputSocketRef> gizmo_inputs_for_node_inputs;
  MultiValueMap<GroupInputRef, InputSocketRef> gizmo_inputs_for_interface_inputs;

  friend std::ostream &operator<<(std::ostream &stream, const GizmoInferencingResult &data);
};

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
