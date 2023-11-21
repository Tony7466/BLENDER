/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>
#include <variant>

#include "BLI_compute_context.hh"
#include "BLI_function_ref.hh"
#include "BLI_multi_value_map.hh"
#include "BLI_struct_equality_utils.hh"
#include "BLI_vector.hh"

#include "DNA_node_types.h"

struct wmWindowManager;
struct NodesModifierData;

namespace blender::nodes::gizmos {

/**
 * Often gizmos don't reference the whole socket value, but only a part of it. E.g. an arrow gizmo
 * may only control the Z value of a vector socket. In this case, the `index` is set to 2.
 */
struct ValueElem {
  std::optional<int> index;

  BLI_STRUCT_EQUALITY_OPERATORS_1(ValueElem, index)

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
  ValueElem elem;

  BLI_STRUCT_EQUALITY_OPERATORS_2(InputSocketRef, input_socket, elem)

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
  ValueElem elem;

  BLI_STRUCT_EQUALITY_OPERATORS_2(ValueNodeRef, value_node, elem)

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
  ValueElem elem;

  BLI_STRUCT_EQUALITY_OPERATORS_2(GroupInputRef, input_index, elem)

  uint64_t hash() const
  {
    return get_default_hash_2(this->input_index, this->elem);
  }
};

/**
 * Locates a value that can be modified by a gizmo.
 */
using GizmoSource = std::variant<InputSocketRef, ValueNodeRef, GroupInputRef>;

/**
 * Gizmo propagation is preprocessed on a per node-group basis, instead processing the same node
 * group multiple depending on how it's used.
 */
struct GizmoPropagationResult {
  /**
   * These members allow quickly looking up which gizmos are active.
   */
  Vector<const bNode *> nodes_with_gizmos_inside;
  MultiValueMap<const bNode *, InputSocketRef> gizmo_inputs_for_value_nodes;
  MultiValueMap<const bNodeSocket *, InputSocketRef> gizmo_inputs_for_node_inputs;
  MultiValueMap<GroupInputRef, InputSocketRef> gizmo_inputs_for_interface_inputs;

  friend std::ostream &operator<<(std::ostream &stream, const GizmoPropagationResult &data);
  friend bool operator==(const GizmoPropagationResult &a, const GizmoPropagationResult &b);
  friend bool operator!=(const GizmoPropagationResult &a, const GizmoPropagationResult &b);
};

struct PropagationPath {
  struct PathElem {
    const bNode *node;
    ValueElem elem;
    const ComputeContext *compute_context;
  };
  /**
   * Contains nodes that influence how changes in the gizmo modify the controlled value. The nodes
   * are ordered right-to-left based on the node tree.
   */
  Vector<PathElem> path;
};

struct PropagatedGizmoSource {
  GizmoSource source;
  PropagationPath propagation_path;
};

Vector<PropagatedGizmoSource> find_propagated_gizmo_sources(const ComputeContext &compute_context,
                                                            const bNode &gizmo_node);

void foreach_active_gizmo(
    const Object &object,
    const NodesModifierData &nmd,
    const wmWindowManager &wm,
    FunctionRef<void(const ComputeContext &compute_context, const bNode &gizmo_node)> fn);

bool update_gizmo_propagation(bNodeTree &tree);

}  // namespace blender::nodes::gizmos
