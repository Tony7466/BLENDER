/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_global.hh"
#include "BKE_main.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "BLI_bit_span.hh"
#include "DNA_node_tree_interface_types.h"
#include "NOD_geometry.hh"
#include "NOD_node_declaration.hh"
#include "NOD_socket.hh"

#include "BLI_bit_group_vector.hh"
#include "BLI_bit_span_ops.hh"
#include "BLI_multi_value_map.hh"
#include "BLI_path_util.h"
#include "BLI_offset_indices.hh"
#include "BLI_resource_scope.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"
#include "BLI_string.h"
#include "BLI_tempfile.h"

#include <fstream>
#include <iostream>
#include <ostream>

namespace blender::bke::node_field_inferencing {

using nodes::FieldInferencingInterface;
using nodes::InputSocketFieldType;
using nodes::NodeDeclaration;
using nodes::OutputFieldDependency;
using nodes::OutputSocketFieldType;
using nodes::SocketDeclaration;

static bool is_field_socket_type(const bNodeSocket &socket)
{
  return nodes::socket_type_supports_fields((eNodeSocketDatatype)socket.typeinfo->type);
}

static InputSocketFieldType get_interface_input_field_type(const bNode &node,
                                                           const bNodeSocket &socket)
{
  if (!is_field_socket_type(socket)) {
    return InputSocketFieldType::None;
  }
  if (node.type == NODE_REROUTE) {
    return InputSocketFieldType::IsSupported;
  }
  if (node.type == NODE_GROUP_OUTPUT) {
    /* Outputs always support fields when the data type is correct. */
    return InputSocketFieldType::IsSupported;
  }
  if (node.typeinfo == &blender::bke::NodeTypeUndefined) {
    return InputSocketFieldType::None;
  }
  if (node.type == NODE_CUSTOM) {
    return InputSocketFieldType::None;
  }

  /* TODO: Ensure declaration exists. */
  const NodeDeclaration *node_decl = node.declaration();

  /* Node declarations should be implemented for nodes involved here. */
  BLI_assert(node_decl != nullptr);

  /* Get the field type from the declaration. */
  const SocketDeclaration &socket_decl = *node_decl->inputs[socket.index()];
  const InputSocketFieldType field_type = socket_decl.input_field_type;
  return field_type;
}

static OutputFieldDependency get_interface_output_field_dependency(const bNode &node,
                                                                   const bNodeSocket &socket)
{
  if (!is_field_socket_type(socket)) {
    /* Non-field sockets always output data. */
    return OutputFieldDependency::ForDataSource();
  }
  if (node.type == NODE_REROUTE) {
    /* The reroute just forwards what is passed in. */
    return OutputFieldDependency::ForDependentField();
  }
  if (node.type == NODE_GROUP_INPUT) {
    /* Input nodes get special treatment in #determine_group_input_states. */
    return OutputFieldDependency::ForDependentField();
  }
  if (node.typeinfo == &blender::bke::NodeTypeUndefined) {
    return OutputFieldDependency::ForDataSource();
  }
  if (node.type == NODE_CUSTOM) {
    return OutputFieldDependency::ForDataSource();
  }

  const NodeDeclaration *node_decl = node.declaration();

  /* Node declarations should be implemented for nodes involved here. */
  BLI_assert(node_decl != nullptr);

  /* Use the socket declaration. */
  const SocketDeclaration &socket_decl = *node_decl->outputs[socket.index()];
  return socket_decl.output_field_dependency;
}

static const FieldInferencingInterface &get_dummy_field_inferencing_interface(const bNode &node,
                                                                              ResourceScope &scope)
{
  auto &inferencing_interface = scope.construct<FieldInferencingInterface>();
  inferencing_interface.inputs.append_n_times(InputSocketFieldType::None,
                                              node.input_sockets().size());
  inferencing_interface.outputs.append_n_times(OutputFieldDependency::ForDataSource(),
                                               node.output_sockets().size());
  return inferencing_interface;
}

/**
 * Retrieves information about how the node interacts with fields.
 * In the future, this information can be stored in the node declaration. This would allow this
 * function to return a reference, making it more efficient.
 */
static const FieldInferencingInterface &get_node_field_inferencing_interface(const bNode &node,
                                                                             ResourceScope &scope)
{
  /* Node groups already reference all required information, so just return that. */
  if (node.is_group()) {
    bNodeTree *group = (bNodeTree *)node.id;
    if (group == nullptr) {
      static const FieldInferencingInterface empty_interface;
      return empty_interface;
    }
    if (!ntreeIsRegistered(group)) {
      /* This can happen when there is a linked node group that was not found (see #92799). */
      return get_dummy_field_inferencing_interface(node, scope);
    }
    if (!group->runtime->field_inferencing_interface) {
      /* This shouldn't happen because referenced node groups should always be updated first. */
      BLI_assert_unreachable();
    }
    return *group->runtime->field_inferencing_interface;
  }

  auto &inferencing_interface = scope.construct<FieldInferencingInterface>();
  for (const bNodeSocket *input_socket : node.input_sockets()) {
    inferencing_interface.inputs.append(get_interface_input_field_type(node, *input_socket));
  }

  for (const bNodeSocket *output_socket : node.output_sockets()) {
    inferencing_interface.outputs.append(
        get_interface_output_field_dependency(node, *output_socket));
  }
  return inferencing_interface;
}

/**
 * This struct contains information for every socket. The values are propagated through the
 * network.
 */
struct SocketFieldState {
  /* This socket starts a new field. */
  bool is_field_source = false;
  /* This socket can never become a field, because the node itself does not support it. */
  bool is_always_single = false;
  /* This socket is currently a single value. It could become a field though. */
  bool is_single = true;
  /* This socket is required to be a single value. This can be because the node itself only
   * supports this socket to be a single value, or because a node afterwards requires this to be a
   * single value. */
  bool requires_single = false;
};

static Vector<const bNodeSocket *> gather_input_socket_dependencies(
    const OutputFieldDependency &field_dependency, const bNode &node)
{
  const OutputSocketFieldType type = field_dependency.field_type();
  Vector<const bNodeSocket *> input_sockets;
  switch (type) {
    case OutputSocketFieldType::FieldSource:
    case OutputSocketFieldType::None: {
      break;
    }
    case OutputSocketFieldType::DependentField: {
      /* This output depends on all inputs. */
      input_sockets.extend(node.input_sockets());
      break;
    }
    case OutputSocketFieldType::PartiallyDependent: {
      /* This output depends only on a few inputs. */
      for (const int i : field_dependency.linked_input_indices()) {
        input_sockets.append(&node.input_socket(i));
      }
      break;
    }
  }
  return input_sockets;
}

/**
 * Check what the group output socket depends on. Potentially traverses the node tree
 * to figure out if it is always a field or if it depends on any group inputs.
 */
static OutputFieldDependency find_group_output_dependencies(
    const bNodeSocket &group_output_socket,
    const Span<const FieldInferencingInterface *> interface_by_node,
    const Span<SocketFieldState> field_state_by_socket_id)
{
  if (!is_field_socket_type(group_output_socket)) {
    return OutputFieldDependency::ForDataSource();
  }

  /* Use a Set here instead of an array indexed by socket id, because we my only need to look at
   * very few sockets. */
  Set<const bNodeSocket *> handled_sockets;
  Stack<const bNodeSocket *> sockets_to_check;

  handled_sockets.add(&group_output_socket);
  sockets_to_check.push(&group_output_socket);

  /* Keeps track of group input indices that are (indirectly) connected to the output. */
  Vector<int> linked_input_indices;

  while (!sockets_to_check.is_empty()) {
    const bNodeSocket *input_socket = sockets_to_check.pop();

    if (!input_socket->is_directly_linked() &&
        !field_state_by_socket_id[input_socket->index_in_tree()].is_single)
    {
      /* This socket uses a field as input by default. */
      return OutputFieldDependency::ForFieldSource();
    }

    for (const bNodeSocket *origin_socket : input_socket->directly_linked_sockets()) {
      const bNode &origin_node = origin_socket->owner_node();
      const SocketFieldState &origin_state =
          field_state_by_socket_id[origin_socket->index_in_tree()];

      if (origin_state.is_field_source) {
        if (origin_node.type == NODE_GROUP_INPUT) {
          /* Found a group input that the group output depends on. */
          linked_input_indices.append_non_duplicates(origin_socket->index());
        }
        else {
          /* Found a field source that is not the group input. So the output is always a field. */
          return OutputFieldDependency::ForFieldSource();
        }
      }
      else if (!origin_state.is_single) {
        const FieldInferencingInterface &inferencing_interface =
            *interface_by_node[origin_node.index()];
        const OutputFieldDependency &field_dependency =
            inferencing_interface.outputs[origin_socket->index()];

        /* Propagate search further to the left. */
        for (const bNodeSocket *origin_input_socket :
             gather_input_socket_dependencies(field_dependency, origin_node))
        {
          if (!origin_input_socket->is_available()) {
            continue;
          }
          if (!field_state_by_socket_id[origin_input_socket->index_in_tree()].is_single) {
            if (handled_sockets.add(origin_input_socket)) {
              sockets_to_check.push(origin_input_socket);
            }
          }
        }
      }
    }
  }
  return OutputFieldDependency::ForPartiallyDependentField(std::move(linked_input_indices));
}

/** Result of syncing two field states. */
enum class eFieldStateSyncResult : char {
  /* Nothing changed. */
  NONE = 0,
  /* State A has been modified. */
  CHANGED_A = (1 << 0),
  /* State B has been modified. */
  CHANGED_B = (1 << 1),
};
ENUM_OPERATORS(eFieldStateSyncResult, eFieldStateSyncResult::CHANGED_B)

/**
 * Compare both field states and select the most compatible.
 * Afterwards both field states will be the same.
 * \return eFieldStateSyncResult flags indicating which field states have changed.
 */
static eFieldStateSyncResult sync_field_states(SocketFieldState &a, SocketFieldState &b)
{
  const bool requires_single = a.requires_single || b.requires_single;
  const bool is_single = a.is_single && b.is_single;

  eFieldStateSyncResult res = eFieldStateSyncResult::NONE;
  if (a.requires_single != requires_single || a.is_single != is_single) {
    res |= eFieldStateSyncResult::CHANGED_A;
  }
  if (b.requires_single != requires_single || b.is_single != is_single) {
    res |= eFieldStateSyncResult::CHANGED_B;
  }

  a.requires_single = requires_single;
  b.requires_single = requires_single;
  a.is_single = is_single;
  b.is_single = is_single;

  return res;
}

/**
 * Compare field states of simulation nodes sockets and select the most compatible.
 * Afterwards all field states will be the same.
 * \return eFieldStateSyncResult flags indicating which field states have changed.
 */
static eFieldStateSyncResult simulation_nodes_field_state_sync(
    const bNode &input_node,
    const bNode &output_node,
    const MutableSpan<SocketFieldState> field_state_by_socket_id)
{
  eFieldStateSyncResult res = eFieldStateSyncResult::NONE;
  for (const int i : output_node.output_sockets().index_range()) {
    /* First input node output is Delta Time which does not appear in the output node outputs. */
    const bNodeSocket &input_socket = input_node.output_socket(i + 1);
    const bNodeSocket &output_socket = output_node.output_socket(i);
    SocketFieldState &input_state = field_state_by_socket_id[input_socket.index_in_tree()];
    SocketFieldState &output_state = field_state_by_socket_id[output_socket.index_in_tree()];
    res |= sync_field_states(input_state, output_state);
  }
  return res;
}

static eFieldStateSyncResult repeat_field_state_sync(
    const bNode &input_node,
    const bNode &output_node,
    const MutableSpan<SocketFieldState> field_state_by_socket_id)
{
  eFieldStateSyncResult res = eFieldStateSyncResult::NONE;
  for (const int i : output_node.output_sockets().index_range()) {
    const bNodeSocket &input_socket = input_node.output_socket(i);
    const bNodeSocket &output_socket = output_node.output_socket(i);
    SocketFieldState &input_state = field_state_by_socket_id[input_socket.index_in_tree()];
    SocketFieldState &output_state = field_state_by_socket_id[output_socket.index_in_tree()];
    res |= sync_field_states(input_state, output_state);
  }
  return res;
}

static bool propagate_special_data_requirements(
    const bNodeTree &tree,
    const bNode &node,
    const MutableSpan<SocketFieldState> field_state_by_socket_id)
{
  tree.ensure_topology_cache();

  bool need_update = false;

  /* Sync field state between zone nodes and schedule another pass if necessary. */
  switch (node.type) {
    case GEO_NODE_SIMULATION_INPUT: {
      const NodeGeometrySimulationInput &data = *static_cast<const NodeGeometrySimulationInput *>(
          node.storage);
      if (const bNode *output_node = tree.node_by_id(data.output_node_id)) {
        const eFieldStateSyncResult sync_result = simulation_nodes_field_state_sync(
            node, *output_node, field_state_by_socket_id);
        if (bool(sync_result & eFieldStateSyncResult::CHANGED_B)) {
          need_update = true;
        }
      }
      break;
    }
    case GEO_NODE_SIMULATION_OUTPUT: {
      for (const bNode *input_node : tree.nodes_by_type("GeometryNodeSimulationInput")) {
        const NodeGeometrySimulationInput &data =
            *static_cast<const NodeGeometrySimulationInput *>(input_node->storage);
        if (node.identifier == data.output_node_id) {
          const eFieldStateSyncResult sync_result = simulation_nodes_field_state_sync(
              *input_node, node, field_state_by_socket_id);
          if (bool(sync_result & eFieldStateSyncResult::CHANGED_A)) {
            need_update = true;
          }
        }
      }
      break;
    }
    case GEO_NODE_REPEAT_INPUT: {
      const NodeGeometryRepeatInput &data = *static_cast<const NodeGeometryRepeatInput *>(
          node.storage);
      if (const bNode *output_node = tree.node_by_id(data.output_node_id)) {
        const eFieldStateSyncResult sync_result = repeat_field_state_sync(
            node, *output_node, field_state_by_socket_id);
        if (bool(sync_result & eFieldStateSyncResult::CHANGED_B)) {
          need_update = true;
        }
      }
      break;
    }
    case GEO_NODE_REPEAT_OUTPUT: {
      for (const bNode *input_node : tree.nodes_by_type("GeometryNodeRepeatInput")) {
        const NodeGeometryRepeatInput &data = *static_cast<const NodeGeometryRepeatInput *>(
            input_node->storage);
        if (node.identifier == data.output_node_id) {
          const eFieldStateSyncResult sync_result = repeat_field_state_sync(
              *input_node, node, field_state_by_socket_id);
          if (bool(sync_result & eFieldStateSyncResult::CHANGED_A)) {
            need_update = true;
          }
        }
      }
      break;
    }
  }

  return need_update;
}

static void propagate_data_requirements_from_right_to_left(
    const bNodeTree &tree,
    const Span<const FieldInferencingInterface *> interface_by_node,
    const MutableSpan<SocketFieldState> field_state_by_socket_id)
{
  const Span<const bNode *> toposort_result = tree.toposort_right_to_left();

  while (true) {
    /* Node updates may require several passes due to cyclic dependencies caused by simulation or
     * repeat input/output nodes. */
    bool need_update = false;

    for (const bNode *node : toposort_result) {
      const FieldInferencingInterface &inferencing_interface = *interface_by_node[node->index()];

      for (const bNodeSocket *output_socket : node->output_sockets()) {
        SocketFieldState &state = field_state_by_socket_id[output_socket->index_in_tree()];

        const OutputFieldDependency &field_dependency =
            inferencing_interface.outputs[output_socket->index()];

        if (field_dependency.field_type() == OutputSocketFieldType::FieldSource) {
          continue;
        }
        if (field_dependency.field_type() == OutputSocketFieldType::None) {
          state.requires_single = true;
          state.is_always_single = true;
          continue;
        }

        /* The output is required to be a single value when it is connected to any input that does
         * not support fields. */
        for (const bNodeSocket *target_socket : output_socket->directly_linked_sockets()) {
          if (target_socket->is_available()) {
            state.requires_single |=
                field_state_by_socket_id[target_socket->index_in_tree()].requires_single;
          }
        }

        if (state.requires_single) {
          bool any_input_is_field_implicitly = false;
          const Vector<const bNodeSocket *> connected_inputs = gather_input_socket_dependencies(
              field_dependency, *node);
          for (const bNodeSocket *input_socket : connected_inputs) {
            if (!input_socket->is_available()) {
              continue;
            }
            if (inferencing_interface.inputs[input_socket->index()] ==
                InputSocketFieldType::Implicit)
            {
              if (!input_socket->is_logically_linked()) {
                any_input_is_field_implicitly = true;
                break;
              }
            }
          }
          if (any_input_is_field_implicitly) {
            /* This output isn't a single value actually. */
            state.requires_single = false;
          }
          else {
            /* If the output is required to be a single value, the connected inputs in the same
             * node must not be fields as well. */
            for (const bNodeSocket *input_socket : connected_inputs) {
              field_state_by_socket_id[input_socket->index_in_tree()].requires_single = true;
            }
          }
        }
      }

      /* Some inputs do not require fields independent of what the outputs are connected to. */
      for (const bNodeSocket *input_socket : node->input_sockets()) {
        SocketFieldState &state = field_state_by_socket_id[input_socket->index_in_tree()];
        if (inferencing_interface.inputs[input_socket->index()] == InputSocketFieldType::None) {
          state.requires_single = true;
          state.is_always_single = true;
        }
      }

      /* Find reverse dependencies and resolve conflicts, which may require another pass. */
      if (propagate_special_data_requirements(tree, *node, field_state_by_socket_id)) {
        need_update = true;
      }
    }

    if (!need_update) {
      break;
    }
  }
}

static void determine_group_input_states(
    const bNodeTree &tree,
    FieldInferencingInterface &new_inferencing_interface,
    const MutableSpan<SocketFieldState> field_state_by_socket_id)
{
  {
    /* Non-field inputs never support fields. */
    for (const int index : tree.interface_inputs().index_range()) {
      const bNodeTreeInterfaceSocket *group_input = tree.interface_inputs()[index];
      const bNodeSocketType *typeinfo = group_input->socket_typeinfo();
      const eNodeSocketDatatype type = typeinfo ? eNodeSocketDatatype(typeinfo->type) :
                                                  SOCK_CUSTOM;
      if (!nodes::socket_type_supports_fields(type)) {
        new_inferencing_interface.inputs[index] = InputSocketFieldType::None;
      }
      else if (group_input->default_input != NODE_INPUT_DEFAULT_VALUE) {
        new_inferencing_interface.inputs[index] = InputSocketFieldType::Implicit;
      }
      else if (is_layer_selection_field(*group_input)) {
        new_inferencing_interface.inputs[index] = InputSocketFieldType::Implicit;
      }
      else if (group_input->flag & NODE_INTERFACE_SOCKET_SINGLE_VALUE_ONLY) {
        new_inferencing_interface.inputs[index] = InputSocketFieldType::None;
      }
    }
  }
  /* Check if group inputs are required to be single values, because they are (indirectly)
   * connected to some socket that does not support fields. */
  for (const bNode *node : tree.group_input_nodes()) {
    for (const bNodeSocket *output_socket : node->output_sockets().drop_back(1)) {
      SocketFieldState &state = field_state_by_socket_id[output_socket->index_in_tree()];
      const int output_index = output_socket->index();
      if (state.requires_single) {
        if (new_inferencing_interface.inputs[output_index] == InputSocketFieldType::Implicit) {
          /* Don't override hard-coded implicit fields. */
          continue;
        }
        new_inferencing_interface.inputs[output_index] = InputSocketFieldType::None;
      }
    }
  }
  /* If an input does not support fields, this should be reflected in all Group Input nodes. */
  for (const bNode *node : tree.group_input_nodes()) {
    for (const bNodeSocket *output_socket : node->output_sockets().drop_back(1)) {
      SocketFieldState &state = field_state_by_socket_id[output_socket->index_in_tree()];
      const bool supports_field = new_inferencing_interface.inputs[output_socket->index()] !=
                                  InputSocketFieldType::None;
      if (supports_field) {
        state.is_single = false;
        state.is_field_source = true;
      }
      else {
        state.requires_single = true;
      }
    }
    SocketFieldState &dummy_socket_state =
        field_state_by_socket_id[node->output_sockets().last()->index_in_tree()];
    dummy_socket_state.requires_single = true;
  }
}

static void propagate_field_status_from_left_to_right(
    const bNodeTree &tree,
    const Span<const FieldInferencingInterface *> interface_by_node,
    const MutableSpan<SocketFieldState> field_state_by_socket_id)
{
  const Span<const bNode *> toposort_result = tree.toposort_left_to_right();

  while (true) {
    /* Node updates may require several passes due to cyclic dependencies. */
    bool need_update = false;

    for (const bNode *node : toposort_result) {
      if (node->type == NODE_GROUP_INPUT) {
        continue;
      }

      const FieldInferencingInterface &inferencing_interface = *interface_by_node[node->index()];

      /* Update field state of input sockets, also taking into account linked origin sockets. */
      for (const bNodeSocket *input_socket : node->input_sockets()) {
        SocketFieldState &state = field_state_by_socket_id[input_socket->index_in_tree()];
        if (state.is_always_single) {
          state.is_single = true;
          continue;
        }
        state.is_single = true;
        if (!input_socket->is_directly_linked()) {
          if (inferencing_interface.inputs[input_socket->index()] ==
              InputSocketFieldType::Implicit)
          {
            state.is_single = false;
          }
        }
        else {
          for (const bNodeSocket *origin_socket : input_socket->directly_linked_sockets()) {
            if (!field_state_by_socket_id[origin_socket->index_in_tree()].is_single) {
              state.is_single = false;
              break;
            }
          }
        }
      }

      /* Update field state of output sockets, also taking into account input sockets. */
      for (const bNodeSocket *output_socket : node->output_sockets()) {
        SocketFieldState &state = field_state_by_socket_id[output_socket->index_in_tree()];
        const OutputFieldDependency &field_dependency =
            inferencing_interface.outputs[output_socket->index()];

        switch (field_dependency.field_type()) {
          case OutputSocketFieldType::None: {
            state.is_single = true;
            break;
          }
          case OutputSocketFieldType::FieldSource: {
            state.is_single = false;
            state.is_field_source = true;
            break;
          }
          case OutputSocketFieldType::PartiallyDependent:
          case OutputSocketFieldType::DependentField: {
            for (const bNodeSocket *input_socket :
                 gather_input_socket_dependencies(field_dependency, *node))
            {
              if (!input_socket->is_available()) {
                continue;
              }
              if (!field_state_by_socket_id[input_socket->index_in_tree()].is_single) {
                state.is_single = false;
                break;
              }
            }
            break;
          }
        }
      }

      /* Find reverse dependencies and resolve conflicts, which may require another pass. */
      if (propagate_special_data_requirements(tree, *node, field_state_by_socket_id)) {
        need_update = true;
      }
    }

    if (!need_update) {
      break;
    }
  }
}

static void determine_group_output_states(
    const bNodeTree &tree,
    FieldInferencingInterface &new_inferencing_interface,
    const Span<const FieldInferencingInterface *> interface_by_node,
    const Span<SocketFieldState> field_state_by_socket_id)
{
  const bNode *group_output_node = tree.group_output_node();
  if (!group_output_node) {
    return;
  }

  for (const bNodeSocket *group_output_socket : group_output_node->input_sockets().drop_back(1)) {
    OutputFieldDependency field_dependency = find_group_output_dependencies(
        *group_output_socket, interface_by_node, field_state_by_socket_id);
    new_inferencing_interface.outputs[group_output_socket->index()] = std::move(field_dependency);
  }
}

static void update_socket_shapes(const bNodeTree &tree,
                                 const Span<SocketFieldState> field_state_by_socket_id)
{
  const eNodeSocketDisplayShape requires_data_shape = SOCK_DISPLAY_SHAPE_CIRCLE;
  const eNodeSocketDisplayShape data_but_can_be_field_shape = SOCK_DISPLAY_SHAPE_DIAMOND_DOT;
  const eNodeSocketDisplayShape is_field_shape = SOCK_DISPLAY_SHAPE_DIAMOND;

  auto get_shape_for_state = [&](const SocketFieldState &state) {
    if (state.is_always_single) {
      return requires_data_shape;
    }
    if (!state.is_single) {
      return is_field_shape;
    }
    if (state.requires_single) {
      return requires_data_shape;
    }
    return data_but_can_be_field_shape;
  };

  for (const bNodeSocket *socket : tree.all_input_sockets()) {
    const SocketFieldState &state = field_state_by_socket_id[socket->index_in_tree()];
    const_cast<bNodeSocket *>(socket)->display_shape = get_shape_for_state(state);
  }
  for (const bNodeSocket *socket : tree.all_sockets()) {
    const SocketFieldState &state = field_state_by_socket_id[socket->index_in_tree()];
    const_cast<bNodeSocket *>(socket)->display_shape = get_shape_for_state(state);
  }
}

static void prepare_inferencing_interfaces(
    const Span<const bNode *> nodes,
    MutableSpan<const FieldInferencingInterface *> interface_by_node,
    ResourceScope &scope)
{
  for (const int i : nodes.index_range()) {
    interface_by_node[i] = &get_node_field_inferencing_interface(*nodes[i], scope);
  }
}

namespace ac3 {

/** Unary constraint function, returns true if the value is allowed. */
using UnaryConstraintFn = std::function<bool(int value)>;
/** Binary constraint function, returns true if both values are compatible. */
using BinaryConstraintFn = std::function<bool(int value_a, int value_b)>;

class VariableRef {
  int index_;

 public:
  explicit VariableRef(const int index) : index_(index) {}

  int index() const
  {
    return index_;
  }

  operator int() const
  {
    return index_;
  }

  bool operator==(const VariableRef other) const
  {
    return index_ == other.index_;
  }

  uint64_t hash() const
  {
    return get_default_hash(index_);
  }
};

struct MutableVariableRef {
  int index_;

 public:
  explicit MutableVariableRef(const int index) : index_(index) {}

  int index() const
  {
    return index_;
  }

  operator int() const
  {
    return index_;
  }

  operator VariableRef() const
  {
    return VariableRef(index_);
  }

  bool operator==(const MutableVariableRef other) const
  {
    return index_ == other.index_;
  }

  uint64_t hash() const
  {
    return get_default_hash(index_);
  }
};

class ConstraintSet {
 public:
  struct Target {
    MutableVariableRef variable;
    BinaryConstraintFn constraint;
  };
  struct Source {
    VariableRef variable;
    BinaryConstraintFn constraint;
  };

 private:
  MultiValueMap<MutableVariableRef, UnaryConstraintFn> unary_;
  MultiValueMap<VariableRef, Target> binary_by_source_;
  MultiValueMap<MutableVariableRef, Source> binary_by_target_;

 public:
  const MultiValueMap<MutableVariableRef, UnaryConstraintFn> &all_unary_constraints() const
  {
    return unary_;
  }
  const MultiValueMap<VariableRef, Target> &binary_constraints_by_source() const
  {
    return binary_by_source_;
  }
  const MultiValueMap<MutableVariableRef, Source> &binary_constraints_by_target() const
  {
    return binary_by_target_;
  }

  Span<UnaryConstraintFn> get_unary_constraints(const MutableVariableRef source) const
  {
    return unary_.lookup(source);
  }
  Span<Target> get_target_constraints(const VariableRef source) const
  {
    return binary_by_source_.lookup(source);
  }
  Span<Source> get_source_constraints(const MutableVariableRef target) const
  {
    return binary_by_target_.lookup(target);
  }
  BinaryConstraintFn get_binary_constraint(const VariableRef source_key,
                                           const MutableVariableRef target_key) const
  {
    for (const Target &target : binary_by_source_.lookup(source_key)) {
      if (target.variable == target_key) {
        return target.constraint;
      }
    }
    return nullptr;
  }

  void add(const MutableVariableRef variable, UnaryConstraintFn constraint)
  {
    unary_.add(variable, constraint);
  }
  void add(const MutableVariableRef target,
           const VariableRef source,
           BinaryConstraintFn constraint)
  {
    binary_by_source_.add(source, {target, constraint});
    binary_by_target_.add(target, {source, constraint});
  }
};

/* Remove all domain values that are not allowed by the constraint. */
static void reduce_unary(const UnaryConstraintFn &constraint, MutableBitSpan domain)
{
  for (const int i : domain.index_range()) {
    if (!domain[i]) {
      continue;
    }
    if (!constraint(i)) {
      domain[i].reset();
    }
  }
}

/* Remove all domain values from A that can not be paired with any value in B. */
static bool reduce_binary(const BinaryConstraintFn &constraint,
                          BitSpan domain_src,
                          MutableBitSpan domain_dst)
{
  bool changed = false;
  for (const int i : domain_dst.index_range()) {
    if (!domain_dst[i]) {
      continue;
    }
    bool valid = false;
    for (const int j : domain_src.index_range()) {
      if (domain_src[j] && constraint(i, j)) {
        valid = true;
      }
    }
    if (!valid) {
      domain_dst[i].reset();
      changed = true;
    }
  }
  return changed;
}

struct NullLogger {
  void on_start(StringRef /*message*/) {}
  void on_end() {}

  void declare_variables(const int /*num_vars*/, FunctionRef<std::string(VariableRef)> /*names_fn*/) {}
  void declare_constraints(const ConstraintSet &/*constraints*/) {}

  void notify(StringRef /*message*/) {}

  void on_solve_start() {}
  void on_worklist_extended(VariableRef /*src*/, VariableRef /*dst*/) {}
  void on_binary_constraint_applied(VariableRef /*src*/, VariableRef /*dst*/) {}

  void on_domain_init(VariableRef /*var*/, const BitSpan /*domain*/) {}
  void on_domain_reduced(VariableRef /*var*/, const BitSpan /*domain*/) {}
  void on_domain_empty(VariableRef /*var*/) {}

  void on_solve_end() {}
};

struct PrintLogger {
  void on_start(StringRef message)
  {
    std::cout << message << std::endl;
  }
  void on_end() {}

  void declare_variables(const int /*num_vars*/,
                         FunctionRef<std::string(VariableRef)> /*names_fn*/)
  {
  }
  void declare_constraints(const ConstraintSet & /*constraints*/) {}

  void notify(StringRef message)
  {
    std::cout << message << std::endl;
  }

  void on_solve_start() {}

  void on_worklist_extended(VariableRef src, VariableRef dst)
  {
    std::cout << "  Worklist extended: " << src.index() << ", " << dst.index() << std::endl;
  }

  void on_binary_constraint_applied(VariableRef src, VariableRef dst)
  {
    std::cout << "  Applying " << src.index() << ", " << dst.index() << std::endl;
  }

  void on_domain_init(VariableRef /*var*/, const BitSpan /*domain*/)
  {
    std::cout << "    Initialized domain" << std::endl;
  }

  void on_domain_reduced(VariableRef /*var*/, const BitSpan /*domain*/)
  {
    std::cout << "    Reduced domain!" << std::endl;
  }

  void on_domain_empty(VariableRef var)
  {
    std::cout << "      FAILED! No possible values for " << var.index() << std::endl;
  }

  static void on_solve_end() {}
};

struct JSONLogger {
  std::ostream &stream;
  Array<std::string> variable_names;
  bool has_events = false;

  JSONLogger(std::ostream &stream) : stream(stream)
  {
    this->stream << "{" << std::endl;
  }
  ~JSONLogger()
  {
    this->stream << "}" << std::endl;
  }

  template<typename Fn> void on_event(StringRef type, Fn fn)
  {
    if (has_events) {
      this->stream << ", ";
    }
    this->stream << "{\"type\": \"" << type << "\", ";
    fn();
    this->stream << "}" << std::endl;
    has_events = true;
  }

  std::string stringify(StringRef s)
  {
    return "\"" + s + "\"";
  }

  int domain_as_int(const BitSpan domain)
  {
    int d = 0;
    for (const int k : domain.index_range()) {
      if (domain[k]) {
        d |= (1 << k);
      }
    }
    return d;
  }

  void declare_variables(const int num_vars, FunctionRef<std::string(VariableRef)> names_fn)
  {
    variable_names.reinitialize(num_vars);
    this->stream << "\"variables\": [";
    for (const int i : IndexRange(num_vars)) {
      variable_names[i] = names_fn(VariableRef(i));
      if (i > 0) {
        this->stream << ", ";
      }
      this->stream << "{\"name\": " << stringify(variable_names[i]) << "}" << std::endl;
    }
    this->stream << "]" << std::endl;
  }

  void declare_constraints(const ConstraintSet &constraints)
  {
    bool has_constraints = false;
    this->stream << ", \"constraints\": [";
    for (const auto &item : constraints.binary_constraints_by_source().items()) {
      const VariableRef src = item.key;
      for (const ConstraintSet::Target &target : item.value) {
        const VariableRef dst = target.variable;
        if (has_constraints) {
          this->stream << ", ";
        }
        const std::string src_name = variable_names[src.index()];
        const std::string dst_name = variable_names[dst.index()];
        this->stream << "{\"name\": " << stringify(src_name + "--" + dst_name)
                     << ", \"source\": " << stringify(src_name)
                     << ", \"target\": " << stringify(dst_name) << "}" << std::endl;
        has_constraints = true;
      }
    }
    this->stream << "]" << std::endl;
  }

  void notify(StringRef /*message*/) {}

  void on_solve_start()
  {
    this->stream << ", \"events\": [";
  }

  void on_worklist_extended(VariableRef src, VariableRef dst)
  {
    on_event("worklist added", [&]() {
      this->stream << "\"source\": " << stringify(variable_names[src.index()]) << ", \"target\": \""
                   << variable_names[dst.index()] << "\"";
    });
  }

  void on_binary_constraint_applied(VariableRef src, VariableRef dst)
  {
    on_event("constraint applied", [&]() {
      this->stream << "\"source\": " << stringify(variable_names[src.index()])
                   << ", \"target\": " << stringify(variable_names[dst.index()]);
    });
  }

  void on_domain_init(VariableRef var, const BitSpan domain)
  {
    on_event("domain init", [&]() {
      this->stream << "\"variable\": " << stringify(variable_names[var.index()])
                   << ", \"domain\": " << domain_as_int(domain);
    });
  }

  void on_domain_reduced(VariableRef var, const BitSpan domain)
  {
    on_event("domain reduced", [&]() {
      this->stream << "\"variable\": " << stringify(variable_names[var.index()])
                   << ", \"domain\": " << domain_as_int(domain);
    });
  }

  void on_domain_empty(VariableRef var)
  {
    on_event("domain empty",
             [&]() { this->stream << "\"variable\": " << stringify(variable_names[var.index()]); });
  }

  void on_solve_end()
  {
    this->stream << "]" << std::endl;
  }
};

/* Apply all unitary constraints. */
template<typename Logger = NullLogger>
static void solve_unary_constraints(const ConstraintSet &constraints,
                                    BitGroupVector<> &variable_domains,
                                    Logger & /*logger*/)
{
  for (const int i : variable_domains.index_range()) {
    const MutableVariableRef var(i);
    for (const UnaryConstraintFn &constraint : constraints.get_unary_constraints(var)) {
      reduce_unary(constraint, variable_domains[i]);
    }
  }
}

template<typename Logger = NullLogger>
static void solve_binary_constraints(const ConstraintSet &constraints,
                                     BitGroupVector<> &variable_domains,
                                     Logger &logger)
{
  /* TODO sorting the worklist could have significant impact on performance
   * by reducing unnecessary repetition of constraints.
   * Using the topological sorting of sockets should make a decent "preconditioner".
   * This is similar to what the current R-L/L-R solver does. */
  struct BinaryKey {
    VariableRef source;
    MutableVariableRef target;
  };
  Stack<BinaryKey> worklist;
  logger.notify("Binary Constraint Solve");
  for (const int i : variable_domains.index_range()) {
    VariableRef source_var(i);
    for (const ConstraintSet::Target &target : constraints.get_target_constraints(source_var)) {
      worklist.push({source_var, target.variable});
      logger.on_worklist_extended(source_var, target.variable);
    }
  }

  while (!worklist.is_empty()) {
    const BinaryKey key = worklist.pop();
    logger.on_binary_constraint_applied(key.source, key.target);
    const BinaryConstraintFn &constraint = constraints.get_binary_constraint(key.source, key.target);
    const BitSpan domain_src = variable_domains[key.source];
    const MutableBitSpan domain_dst = variable_domains[key.target];
    if (reduce_binary(constraint, domain_src, domain_dst)) {
      logger.on_domain_reduced(key.source, domain_src);
      if (!bits::any_bit_set(domain_src)) {
        /* TODO FAILURE CASE! */
        logger.on_domain_empty(key.target);
        break;
      }

      /* Add arcs from target to all dependant variables (except the source). */
      for (const ConstraintSet::Target &target : constraints.get_target_constraints(key.target)) {
        if (target.variable == key.source) {
          continue;
        }
        logger.on_worklist_extended(key.target, target.variable);
        worklist.push({key.source, target.variable});
      }
    }
  }
}

template<typename Logger = NullLogger>
static BitGroupVector<> solve_constraints(const ConstraintSet &constraints,
                                          const int num_vars,
                                          const int domain_size,
                                          Logger &logger)
{
  BitGroupVector variable_domains(num_vars, domain_size, true);

  logger.on_solve_start();
  for (const int i : variable_domains.index_range()) {
    logger.on_domain_init(VariableRef(i), variable_domains[i]);
  }

  solve_unary_constraints<Logger>(constraints, variable_domains, logger);
  solve_binary_constraints<Logger>(constraints, variable_domains, logger);
  logger.on_solve_end();

  return variable_domains;
}

}  // namespace ac3

enum DomainValue { Single, Field, NumDomainValues };

static void add_node_type_constraints(const bNodeTree &tree,
                                      const bNode &node,
                                      ac3::ConstraintSet &constraints)
{
  tree.ensure_topology_cache();

  /* Constraint is satisfied if both inputs or outputs of a zone node pair are the same type. */
  auto shared_field_type_constraint = [](const int value_a, const int value_b) {
    return value_a == value_b;
  };

  auto add_zone_constraints = [&](const Span<const bNodeSocket *> input_inputs,
                                  const Span<const bNodeSocket *> input_outputs,
                                  const Span<const bNodeSocket *> output_inputs,
                                  const Span<const bNodeSocket *> output_outputs) {
    BLI_assert(input_inputs.size() == output_inputs.size());
    BLI_assert(input_outputs.size() == output_outputs.size());
    for (const int i : input_inputs.index_range()) {
      if (!input_inputs[i]->is_available() || !output_inputs[i]->is_available()) {
        continue;
      }
      const ac3::MutableVariableRef var_a(input_inputs[i]->index_in_tree());
      const ac3::MutableVariableRef var_b(output_inputs[i]->index_in_tree());
      constraints.add(var_a, var_b, shared_field_type_constraint);
      constraints.add(var_b, var_a, shared_field_type_constraint);
    }
    for (const int i : input_outputs.index_range()) {
      if (!input_outputs[i]->is_available() || !output_outputs[i]->is_available()) {
        continue;
      }
      const ac3::MutableVariableRef var_a(input_outputs[i]->index_in_tree());
      const ac3::MutableVariableRef var_b(output_outputs[i]->index_in_tree());
      constraints.add(var_a, var_b, shared_field_type_constraint);
      constraints.add(var_b, var_a, shared_field_type_constraint);
    }
  };

  switch (node.type) {
    case GEO_NODE_SIMULATION_INPUT: {
      const NodeGeometrySimulationInput &data = *static_cast<const NodeGeometrySimulationInput *>(
          node.storage);
      if (const bNode *output_node = tree.node_by_id(data.output_node_id)) {
        /* First input node output is Delta Time which does not appear in the output node. */
        add_zone_constraints(node.input_sockets(),
                             node.output_sockets().drop_front(1),
                             output_node->input_sockets(),
                             output_node->output_sockets());
      }
      break;
    }
    case GEO_NODE_SIMULATION_OUTPUT: {
      /* Already handled in the input node case. */
      // for (const bNode *input_node : tree.nodes_by_type("GeometryNodeSimulationInput")) {
      //   const NodeGeometrySimulationInput &data =
      //       *static_cast<const NodeGeometrySimulationInput *>(input_node->storage);
      //   if (node.identifier == data.output_node_id) {
      //     /* First input node output is Delta Time which does not appear in the output node. */
      //     add_zone_constraints(input_node->input_sockets(),
      //                          input_node->output_sockets().drop_front(1),
      //                          node.input_sockets(),
      //                          node.output_sockets());
      //   }
      // }
      break;
    }
    case GEO_NODE_REPEAT_INPUT: {
      const NodeGeometryRepeatInput &data = *static_cast<const NodeGeometryRepeatInput *>(
          node.storage);
      if (const bNode *output_node = tree.node_by_id(data.output_node_id)) {
        add_zone_constraints(node.input_sockets(),
                             node.output_sockets(),
                             output_node->input_sockets(),
                             output_node->output_sockets());
      }
      break;
    }
    case GEO_NODE_REPEAT_OUTPUT: {
      /* Already handled in the input node case. */
      // for (const bNode *input_node : tree.nodes_by_type("GeometryNodeRepeatInput")) {
      //   const NodeGeometryRepeatInput &data = *static_cast<const NodeGeometryRepeatInput *>(
      //       input_node->storage);
      //   if (node.identifier == data.output_node_id) {
      //     add_zone_constraints(input_node->input_sockets(),
      //                          input_node->output_sockets(),
      //                          node.input_sockets(),
      //                          node.output_sockets());
      //   }
      // }
      break;
    }
  }
}

/**
 * Check what the group output socket depends on. Potentially traverses the node tree
 * to figure out if it is always a field or if it depends on any group inputs.
 */
static OutputFieldDependency find_group_output_dependencies(
    const bNodeSocket &group_output_socket,
    const Span<const FieldInferencingInterface *> interface_by_node,
    const BitGroupVector<> &field_state_by_socket_id)
{
  if (!is_field_socket_type(group_output_socket)) {
    return OutputFieldDependency::ForDataSource();
  }

  /* Use a Set here instead of an array indexed by socket id, because we my only need to look at
   * very few sockets. */
  Set<const bNodeSocket *> handled_sockets;
  Stack<const bNodeSocket *> sockets_to_check;

  handled_sockets.add(&group_output_socket);
  sockets_to_check.push(&group_output_socket);

  /* Keeps track of group input indices that are (indirectly) connected to the output. */
  Vector<int> linked_input_indices;

  while (!sockets_to_check.is_empty()) {
    const bNodeSocket *input_socket = sockets_to_check.pop();
    const BitSpan input_state = field_state_by_socket_id[input_socket->index_in_tree()];
    const bool can_be_single = input_state[DomainValue::Single];
    const bool can_be_field = input_state[DomainValue::Field];

    if (!input_socket->is_directly_linked() && can_be_field && !can_be_single) {
      /* This socket uses a field as input by default. */
      return OutputFieldDependency::ForFieldSource();
    }
    return OutputFieldDependency::ForFieldSource();

    for (const bNodeSocket *origin_socket : input_socket->directly_linked_sockets()) {
      const bNode &origin_node = origin_socket->owner_node();
      const BitSpan origin_state = field_state_by_socket_id[origin_socket->index_in_tree()];
      const bool origin_can_be_single = origin_state[DomainValue::Single];
      const bool origin_can_be_field = origin_state[DomainValue::Field];

      if (origin_can_be_field && !origin_can_be_single) {
        if (origin_node.type == NODE_GROUP_INPUT) {
          /* Found a group input that the group output depends on. */
          linked_input_indices.append_non_duplicates(origin_socket->index());
        }
        else {
          /* Found a field source that is not the group input. So the output is always a field. */
          return OutputFieldDependency::ForFieldSource();
        }
      }
      else if (!origin_can_be_single) {
        const FieldInferencingInterface &inferencing_interface =
            *interface_by_node[origin_node.index()];
        const OutputFieldDependency &field_dependency =
            inferencing_interface.outputs[origin_socket->index()];

        /* Propagate search further to the left. */
        for (const bNodeSocket *origin_input_socket :
             gather_input_socket_dependencies(field_dependency, origin_node))
        {
          const BitSpan origin_input_state =
              field_state_by_socket_id[origin_input_socket->index_in_tree()];
          const bool origin_input_can_be_single = origin_input_state[DomainValue::Single];
          if (!origin_input_socket->is_available()) {
            continue;
          }
          if (!origin_input_can_be_single) {
            if (handled_sockets.add(origin_input_socket)) {
              sockets_to_check.push(origin_input_socket);
            }
          }
        }
      }
    }
  }
  return OutputFieldDependency::ForPartiallyDependentField(std::move(linked_input_indices));
}

template <typename Logger>
static void test_ac3_field_inferencing(
    const bNodeTree &tree,
    const Span<const FieldInferencingInterface *> interface_by_node,
    FieldInferencingInterface &inferencing_interface,
    Logger &logger)
{
  const bNode *group_output_node = tree.group_output_node();
  if (!group_output_node) {
    return;
  }

  /* Index ranges of variables for node sockets and interface sockets.
   * Group input/output nodes use the tree interface variables directly,
   * their socket variables are unused. */
  const IndexRange socket_vars = tree.all_sockets().index_range();
  const IndexRange tree_input_vars = socket_vars.after(tree.interface_inputs().size());
  const IndexRange tree_output_vars = tree_input_vars.after(tree.interface_outputs().size());
  const int num_vars = tree_output_vars.one_after_last();

  tree.ensure_topology_cache();
  auto variable_name = [&](ac3::VariableRef var) -> std::string {
    if (socket_vars.contains(var.index())) {
      const bNodeSocket &socket = *tree.all_sockets()[var.index()];
      const bNode &node = socket.owner_node();
      return socket.is_output() ? std::string(node.name) + ":O:" + socket.identifier :
                                  std::string(node.name) + ":I:" + socket.identifier;
    }
    if (tree_input_vars.contains(var.index())) {
      const bNodeTreeInterfaceSocket &iosocket =
          *tree.interface_inputs()[var.index() - tree_input_vars.start()];
      return std::string("I:") + iosocket.identifier;
    }
    if (tree_output_vars.contains(var.index())) {
      const bNodeTreeInterfaceSocket &iosocket =
          *tree.interface_outputs()[var.index() - tree_output_vars.start()];
      return std::string("O:") + iosocket.identifier;
    }
    return "";
  };
  logger.declare_variables(num_vars, variable_name);

  const Span<const bNode *> nodes = tree.toposort_right_to_left();

  ac3::ConstraintSet constraints;
  for (const bNode *node : nodes) {
    /* Special case: Group inputs and outputs use the interface variables directly. */
    const IndexRange interface_inputs = node->is_group_input() ?
                                            tree.interface_inputs().index_range() :
                                            IndexRange();
    const IndexRange interface_outputs = node->is_group_output() ?
                                             tree.interface_outputs().index_range() :
                                             IndexRange();
    auto get_socket_variable = [&](const bNodeSocket &socket) -> ac3::MutableVariableRef {
      if (socket.is_output()) {
        if (interface_inputs.contains(socket.index())) {
          return ac3::MutableVariableRef(tree_input_vars[socket.index()]);
        }
        else {
          return ac3::MutableVariableRef(socket_vars[socket.index_in_tree()]);
        }
      }
      else {
        if (interface_outputs.contains(socket.index())) {
          return ac3::MutableVariableRef(tree_output_vars[socket.index()]);
        }
        else {
          return ac3::MutableVariableRef(socket_vars[socket.index_in_tree()]);
        }
      }
    };

    const FieldInferencingInterface &inferencing_interface = *interface_by_node[node->index()];
    for (const bNodeSocket *output_socket : node->output_sockets()) {
      if (!output_socket->is_available()) {
        continue;
      }
      const ac3::MutableVariableRef var = get_socket_variable(*output_socket);
      const bNodeSocketType *typeinfo = output_socket->typeinfo;
      const eNodeSocketDatatype type = typeinfo ? eNodeSocketDatatype(typeinfo->type) :
                                                  SOCK_CUSTOM;

      if (!nodes::socket_type_supports_fields(type)) {
        constraints.add(var, [](const int value) { return value == DomainValue::Single; });
      }

      const OutputFieldDependency &field_dependency =
          inferencing_interface.outputs[output_socket->index()];
      if (field_dependency.field_type() == OutputSocketFieldType::FieldSource) {
        constraints.add(var,
                        [](const int value) { return value == DomainValue::Field; });
      }
      if (field_dependency.field_type() == OutputSocketFieldType::None) {
        constraints.add(var, [](const int value) { return value == DomainValue::Single; });
      }

      /* The output is required to be a single value when it is connected to any input that does
       * not support fields. */
      for (const bNodeSocket *target_socket : output_socket->directly_linked_sockets()) {
        if (target_socket->is_available()) {
          constraints.add(var,
                          ac3::VariableRef{target_socket->index_in_tree()},
                          [](int value_a, int value_b) {
            return value_a == DomainValue::Single || value_b == DomainValue::Field;
          });
        }
      }

      /* TODO not sure if this is already covered by other constraints? */
      // if (state.requires_single) {
      //   bool any_input_is_field_implicitly = false;
      //   const Vector<const bNodeSocket *> connected_inputs = gather_input_socket_dependencies(
      //       field_dependency, *node);
      //   for (const bNodeSocket *input_socket : connected_inputs) {
      //     if (!input_socket->is_available()) {
      //       continue;
      //     }
      //     if (inferencing_interface.inputs[input_socket->index()] ==
      //         InputSocketFieldType::Implicit)
      //     {
      //       if (!input_socket->is_logically_linked()) {
      //         any_input_is_field_implicitly = true;
      //         break;
      //       }
      //     }
      //   }
      //   if (any_input_is_field_implicitly) {
      //     /* This output isn't a single value actually. */
      //     state.requires_single = false;
      //   }
      //   else {
      //     /* If the output is required to be a single value, the connected inputs in the same
      //      * node must not be fields as well. */
      //     for (const bNodeSocket *input_socket : connected_inputs) {
      //       field_state_by_socket_id[input_socket->index_in_tree()].requires_single = true;
      //     }
      //   }
      // }
    }

    /* Some inputs do not require fields independent of what the outputs are connected to. */
    for (const bNodeSocket *input_socket : node->input_sockets()) {
      if (!input_socket->is_available()) {
        continue;
      }
      const ac3::MutableVariableRef var = get_socket_variable(*input_socket);
      const bNodeSocketType *typeinfo = input_socket->typeinfo;
      const eNodeSocketDatatype type = typeinfo ? eNodeSocketDatatype(typeinfo->type) :
                                                  SOCK_CUSTOM;

      if (!nodes::socket_type_supports_fields(type)) {
        constraints.add(var, [](const int value) { return value == DomainValue::Single; });
      }

      const InputSocketFieldType field_type = inferencing_interface.inputs[input_socket->index()];
      if (field_type == InputSocketFieldType::None) {
        constraints.add(ac3::MutableVariableRef(input_socket->index_in_tree()),
                        [](int value) { return value == DomainValue::Single; });
      }
    }

    /* Constraints for consistent field type across zones. */
    add_node_type_constraints(tree, *node, constraints);
  }
  logger.declare_constraints(constraints);

  BitGroupVector<> result = ac3::solve_constraints(constraints, num_vars, NumDomainValues, logger);

  /* Setup inferencing interface for the tree. */
  for (const int i : tree.interface_inputs().index_range()) {
    const int var_index = tree_input_vars[i];
    const BitSpan state = result[var_index];
    if (state[DomainValue::Single]) {
      if (state[DomainValue::Field]) {
        inferencing_interface.inputs[i] = InputSocketFieldType::IsSupported;
      }
      else {
        inferencing_interface.inputs[i] = InputSocketFieldType::None;
      }
    }
    else {
      if (state[DomainValue::Field]) {
        inferencing_interface.inputs[i] = InputSocketFieldType::Implicit;
      }
      else {
        /* Error: No supported field type. */
        BLI_assert_unreachable();
      }
    }
  }

  for (const bNodeSocket *group_output_socket : group_output_node->input_sockets().drop_back(1)) {
    OutputFieldDependency field_dependency = find_group_output_dependencies(
        *group_output_socket, interface_by_node, result);
    inferencing_interface.outputs[group_output_socket->index()] = std::move(field_dependency);
  }
}

template <typename Logger>
static void test_ac3_example(Logger &logger)
{
  /* Example taken from
   * https://www.boristhebrave.com/2021/08/30/arc-consistency-explained/
   */
  const int num_vars = 5;
  const int domain_size = 4;

  ac3::ConstraintSet constraints;
  enum Symmetry {
    None,
    Symmetric,
    Antisymmetric,
  };
  auto add_binary_constraint = [&](const int a,
                                   const int b,
                                   const Symmetry symmetry,
                                   const ac3::BinaryConstraintFn &constraint) {
    constraints.add(a, b, constraint);
    switch (symmetry) {
      case None:
        break;
      case Symmetric:
        constraints.add(b, a, constraint);
        break;
      case Antisymmetric: {
        const auto anti_constraint = [constraint](int value_a, int value_b) {
          return constraint(value_b, value_a);
        };
        constraints.add(b, a, anti_constraint);
        break;
      }
    }
  };

  const int var_A = 0;
  const int var_B = 1;
  const int var_C = 2;
  const int var_D = 3;
  const int var_E = 4;

  [[maybe_unused]] const int value_1 = 0;
  [[maybe_unused]] const int value_2 = 1;
  [[maybe_unused]] const int value_3 = 2;
  [[maybe_unused]] const int value_4 = 3;

  /* C = {1, 2, 4} */
  constraints.add(var_B, [](const int value) { return value != value_3; });
  /* C = {1, 3, 4} */
  constraints.add(var_C, [](const int value) { return value != value_2; });

  /* A != B */
  add_binary_constraint(var_A, var_B, Symmetric, [](const int value_a, const int value_b) {
    return value_a != value_b;
  });
  /* A == D */
  add_binary_constraint(var_A, var_D, Symmetric, [](const int value_a, const int value_b) {
    return value_a == value_b;
  });
  /* A > E */
  add_binary_constraint(var_A, var_E, Antisymmetric, [](const int value_a, const int value_b) {
    return value_a > value_b;
  });
  /* B != C */
  add_binary_constraint(var_B, var_C, Symmetric, [](const int value_a, const int value_b) {
    return value_a != value_b;
  });
  /* B != D */
  add_binary_constraint(var_B, var_D, Symmetric, [](const int value_a, const int value_b) {
    return value_a != value_b;
  });
  /* B > E */
  add_binary_constraint(var_B, var_E, Antisymmetric, [](const int value_a, const int value_b) {
    return value_a > value_b;
  });
  /* C < D */
  add_binary_constraint(var_C, var_D, Antisymmetric, [](const int value_a, const int value_b) {
    return value_a < value_b;
  });
  /* C > E */
  add_binary_constraint(var_C, var_E, Antisymmetric, [](const int value_a, const int value_b) {
    return value_a > value_b;
  });
  /* D > E */
  add_binary_constraint(var_D, var_E, Antisymmetric, [](const int value_a, const int value_b) {
    return value_a > value_b;
  });

  ac3::solve_constraints(constraints, num_vars, domain_size, logger);
}

template <typename Logger>
static bool update_field_inferencing_ex(const bNodeTree &tree, Logger &logger)
{
  BLI_assert(tree.type == NTREE_GEOMETRY);
  tree.ensure_topology_cache();
  tree.ensure_interface_cache();

  const Span<const bNode *> nodes = tree.all_nodes();
  ResourceScope scope;
  Array<const FieldInferencingInterface *> interface_by_node(nodes.size());
  prepare_inferencing_interfaces(nodes, interface_by_node, scope);

  /* Create new inferencing interface for this node group. */
  std::unique_ptr<FieldInferencingInterface> new_inferencing_interface =
      std::make_unique<FieldInferencingInterface>();
  new_inferencing_interface->inputs.resize(tree.interface_inputs().size(),
                                           InputSocketFieldType::IsSupported);
  new_inferencing_interface->outputs.resize(tree.interface_outputs().size(),
                                            OutputFieldDependency::ForDataSource());

#if 0
  /* Keep track of the state of all sockets. The index into this array is #SocketRef::id(). */
  Array<SocketFieldState> field_state_by_socket_id(tree.all_sockets().size());

  propagate_data_requirements_from_right_to_left(
      tree, interface_by_node, field_state_by_socket_id);
  determine_group_input_states(tree, *new_inferencing_interface, field_state_by_socket_id);
  propagate_field_status_from_left_to_right(tree, interface_by_node, field_state_by_socket_id);
  determine_group_output_states(
      tree, *new_inferencing_interface, interface_by_node, field_state_by_socket_id);
  update_socket_shapes(tree, field_state_by_socket_id);
#else
  test_ac3_field_inferencing(tree, interface_by_node, *new_inferencing_interface, logger);
#endif

  /* Update the previous group interface. */
  const bool group_interface_changed = !tree.runtime->field_inferencing_interface ||
                                       *tree.runtime->field_inferencing_interface !=
                                           *new_inferencing_interface;
  tree.runtime->field_inferencing_interface = std::move(new_inferencing_interface);

  return group_interface_changed;
}

bool update_field_inferencing(const bNodeTree &tree) {
  ac3::NullLogger logger;
  return update_field_inferencing_ex(tree, logger);
}

bool dump_field_inferencing_debug_data(const bNodeTree &tree, StringRef filepath) {
  std::ofstream fs;
  fs.open(filepath, std::fstream::out);
  ac3::JSONLogger logger(fs);
  return update_field_inferencing_ex(tree, logger);
}

}  // namespace blender::bke::node_field_inferencing
