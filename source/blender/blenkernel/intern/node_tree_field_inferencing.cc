/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "DNA_node_tree_interface_types.h"
#include "NOD_geometry.hh"
#include "NOD_node_declaration.hh"
#include "NOD_socket.hh"

#include "BLI_constraint_satisfaction.hh"
#include "BLI_resource_scope.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"

#include <fstream>

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

static void prepare_inferencing_interfaces(
    const Span<const bNode *> nodes,
    MutableSpan<const FieldInferencingInterface *> interface_by_node,
    ResourceScope &scope)
{
  for (const int i : nodes.index_range()) {
    interface_by_node[i] = &get_node_field_inferencing_interface(*nodes[i], scope);
  }
}

enum DomainValue {
  /* Socket is a single value. */
  Single,
  /* Socket is a field. */
  Field,

  /* Keep last. */
  NumDomainValues
};

namespace csp = constraint_satisfaction;

struct NodeTreeVariables {
  /* Index ranges of variables for node sockets and interface sockets.
   * Group input/output nodes use the tree interface variables directly,
   * their socket variables are unused. */
  IndexRange socket_vars;
  IndexRange tree_input_vars;
  IndexRange tree_output_vars;

  NodeTreeVariables(const bNodeTree &tree)
  {
    this->socket_vars = tree.all_sockets().index_range();
    this->tree_input_vars = socket_vars.after(tree.interface_inputs().size());
    this->tree_output_vars = tree_input_vars.after(tree.interface_outputs().size());
  }

  int num_vars() const
  {
    return tree_output_vars.one_after_last();
  }

  csp::VariableIndex get_socket_variable(const bNodeSocket &socket) const
  {
    if (socket.is_output()) {
      /* Use tree variables directly for group input nodes (except the extension socket). */
      if (socket.owner_node().is_group_input() &&
          tree_input_vars.index_range().contains(socket.index()))
      {
        return tree_input_vars[socket.index()];
      }
      return socket_vars[socket.index_in_tree()];
    }
    else {
      /* Use tree variables directly for group output nodes (except the extension socket). */
      if (socket.owner_node().is_group_output() &&
          tree_output_vars.index_range().contains(socket.index()))
      {
        return tree_output_vars[socket.index()];
      }
      return socket_vars[socket.index_in_tree()];
    }
  }
};

static std::string input_field_type_name(const InputSocketFieldType type)
{
  switch (type) {
    case InputSocketFieldType::None:
      return "None";
    case InputSocketFieldType::IsSupported:
      return "IsSupported";
    case InputSocketFieldType::Implicit:
      return "Implicit";
  }
  return "";
}

static std::string output_field_type_name(const OutputSocketFieldType type)
{
  switch (type) {
    case OutputSocketFieldType::None:
      return "None";
    case OutputSocketFieldType::FieldSource:
      return "FieldSource";
    case OutputSocketFieldType::DependentField:
      return "DependentField";
    case OutputSocketFieldType::PartiallyDependent:
      return "PartiallyDependent";
  }
  return "";
}

static bool verify_field_inferencing_csp_result(
    const bNodeTree &tree,
    const Span<const FieldInferencingInterface *> interface_by_node,
    const BitGroupVector<> csp_result,
    const FieldInferencingInterface &inferencing_interface)
{
  /* Use the old propagation method to provide "ground truth" to compare against. */
  const bool use_propagation_result = true;

  NodeTreeVariables variables(tree);

  Array<SocketFieldStateLegacy> field_state_by_socket_id;
  std::unique_ptr<FieldInferencingInterface> tmp_inferencing_interface;
  if (use_propagation_result) {
    /* Temp local inferencing interface to avoid overwriting the actual interface.
     * The propagation method directly writes to the interface. */
    tmp_inferencing_interface = std::make_unique<FieldInferencingInterface>();
    tmp_inferencing_interface->inputs.resize(tree.interface_inputs().size(),
                                             InputSocketFieldType::IsSupported);
    tmp_inferencing_interface->outputs.resize(tree.interface_outputs().size(),
                                              OutputFieldDependency::ForDataSource());

    field_state_by_socket_id = solve_field_types_legacy(
        tree, interface_by_node, *tmp_inferencing_interface);
  }

  std::cout << "Verify field type inferencing for tree " << tree.id.name << std::endl;
  bool error = false;
  for (const bNodeSocket *socket : tree.all_sockets()) {
    if (!socket->is_available()) {
      continue;
    }
    auto log_error = [&](StringRef message) {
      const std::string socket_address = std::string(socket->owner_node().name) +
                                         (socket->is_output() ? "|>" : "<|") + socket->identifier;
      std::cout << "  [Error] " << socket_address << ": " << message << std::endl;
      error = true;
    };
    const int var = variables.get_socket_variable(*socket);
    const BitSpan state = csp_result[var];
    const int num_values = int(state[DomainValue::Single]) + int(state[DomainValue::Field]);
    if (num_values == 0) {
      log_error("No valid result");
      continue;
    }

    if (use_propagation_result) {
      const SocketFieldStateLegacy &legacy_state =
          field_state_by_socket_id[socket->index_in_tree()];
      if (legacy_state.is_always_single) {
        if (!state[DomainValue::Single] || state[DomainValue::Field]) {
          log_error("Should only be single value");
        }
        continue;
      }
      if (!legacy_state.is_single) {
        if (state[DomainValue::Single] || !state[DomainValue::Field]) {
          log_error("Should only be field");
        }
        continue;
      }
      if (legacy_state.requires_single) {
        if (!state[DomainValue::Single] || state[DomainValue::Field]) {
          log_error("Should only be single value");
        }
        continue;
      }
      if (!state[DomainValue::Single] || !state[DomainValue::Field]) {
        log_error("Should be single value or field");
      }
    }
  }

  if (use_propagation_result) {
    for (const int i : tree.interface_inputs().index_range()) {
      auto log_error = [&](StringRef message) {
        std::cout << "  [Error] " << tree.interface_inputs()[i]->identifier << ": " << message
                  << std::endl;
        error = true;
      };
      const InputSocketFieldType old_field_type = tmp_inferencing_interface->inputs[i];
      const InputSocketFieldType new_field_type = inferencing_interface.inputs[i];
      if (old_field_type != new_field_type) {
        log_error("Input field type is " + input_field_type_name(new_field_type) + ", expected " +
                  input_field_type_name(old_field_type));
      }
    }
    for (const int i : tree.interface_outputs().index_range()) {
      auto log_error = [&](StringRef message) {
        std::cout << "  [Error] " << tree.interface_outputs()[i]->identifier << ": " << message
                  << std::endl;
        error = true;
      };
      const OutputFieldDependency &old_field_dep = tmp_inferencing_interface->outputs[i];
      const OutputFieldDependency &new_field_dep = inferencing_interface.outputs[i];
      if (old_field_dep.field_type() != new_field_dep.field_type()) {
        log_error("Output field type is " + output_field_type_name(new_field_dep.field_type()) +
                  ", expected " + output_field_type_name(old_field_dep.field_type()));
      }
      if (old_field_dep.linked_input_indices() != new_field_dep.linked_input_indices()) {
        log_error("Output field dependencies don't match");
      }
    }
  }

  if (!error) {
    std::cout << "  OK!" << std::endl;
  }
  return error;
}

static InputSocketFieldType group_input_field_type(const bNodeTreeInterfaceSocket &group_input)
{
  const bNodeSocketType *typeinfo = group_input.socket_typeinfo();
  const eNodeSocketDatatype type = typeinfo ? eNodeSocketDatatype(typeinfo->type) : SOCK_CUSTOM;
  if (!nodes::socket_type_supports_fields(type)) {
    return InputSocketFieldType::None;
  }
  if (group_input.default_input != NODE_INPUT_DEFAULT_VALUE) {
    return InputSocketFieldType::Implicit;
  }
  if (is_layer_selection_field(group_input)) {
    return InputSocketFieldType::Implicit;
  }
  if (group_input.flag & NODE_INTERFACE_SOCKET_SINGLE_VALUE_ONLY) {
    return InputSocketFieldType::None;
  }
  return InputSocketFieldType::IsSupported;
}

static void add_group_constraints(const bNodeTree &tree, csp::ConstraintSet &constraints)
{
  NodeTreeVariables variables(tree);

  /* If an input does not support fields, this should be reflected in all Group Input nodes. */
  for (const int index : tree.interface_inputs().index_range()) {
    const bNodeTreeInterfaceSocket &group_input = *tree.interface_inputs()[index];
    const bool supports_field = group_input_field_type(group_input) != InputSocketFieldType::None;

    const int var = variables.tree_input_vars[index];
    /* If the group input supports fields assume the input is a field, otherwise only support
     * single values. */
    if (supports_field) {
      constraints.add_unary(var, [](const int value) { return value == DomainValue::Field; });
    }
    else {
      constraints.add_unary(var, [](const int value) { return value == DomainValue::Single; });
    }
  }
}

static void add_node_constraints(const bNodeTree &tree,
                                 const bNode &node,
                                 const FieldInferencingInterface &inferencing_interface,
                                 csp::ConstraintSet &constraints)
{
  NodeTreeVariables variables(tree);

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
      const int var_a = input_inputs[i]->index_in_tree();
      const int var_b = output_inputs[i]->index_in_tree();
      constraints.add_binary_symmetric(var_a, var_b, shared_field_type_constraint);
    }
    for (const int i : input_outputs.index_range()) {
      if (!input_outputs[i]->is_available() || !output_outputs[i]->is_available()) {
        continue;
      }
      const int var_a = input_outputs[i]->index_in_tree();
      const int var_b = output_outputs[i]->index_in_tree();
      constraints.add_binary_symmetric(var_a, var_b, shared_field_type_constraint);
    }
  };

  for (const bNodeSocket *output_socket : node.output_sockets()) {
    if (!output_socket->is_available()) {
      continue;
    }
    const int var = variables.get_socket_variable(*output_socket);
    const bNodeSocketType *typeinfo = output_socket->typeinfo;
    const eNodeSocketDatatype type = typeinfo ? eNodeSocketDatatype(typeinfo->type) : SOCK_CUSTOM;

    if (!nodes::socket_type_supports_fields(type)) {
      constraints.add_unary(var, [](const int value) { return value == DomainValue::Single; });
    }

    const OutputFieldDependency &field_dependency =
        inferencing_interface.outputs[output_socket->index()];
    switch (field_dependency.field_type()) {
      /* Fixed single value output. */
      case OutputSocketFieldType::None:
        constraints.add_unary(var, [](const int value) { return value == DomainValue::Single; });
        break;
      /* Fixed field source output. */
      case OutputSocketFieldType::FieldSource:
        constraints.add_unary(var, [](const int value) { return value == DomainValue::Field; });
        break;
      /* Internal dependency on one or more inputs. */
      case OutputSocketFieldType::DependentField:
      case OutputSocketFieldType::PartiallyDependent:
        for (const bNodeSocket *input_socket :
             gather_input_socket_dependencies(field_dependency, node))
        {
          if (!input_socket->is_available()) {
            continue;
          }
          const int input_var = variables.get_socket_variable(*input_socket);
          /* The output must be a field if the input it depends on is a field. */
          constraints.add_binary(var, input_var, [](const int value_dst, const int value_src) {
            return value_dst == DomainValue::Field || value_src == DomainValue::Single;
          });
        }
        break;
    }

    /* The output must be a single value when it is connected to any input that does
     * not support fields. */
    for (const bNodeSocket *src_socket : output_socket->directly_linked_sockets()) {
      if (src_socket->is_available()) {
        const int src_var = variables.get_socket_variable(*src_socket);
        constraints.add_binary(var, src_var, [](int value_dst, int value_src) {
          return value_dst == DomainValue::Single || value_src == DomainValue::Field;
        });
      }
    }

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
  for (const bNodeSocket *input_socket : node.input_sockets()) {
    if (!input_socket->is_available()) {
      continue;
    }
    const int var = variables.get_socket_variable(*input_socket);
    const bNodeSocketType *typeinfo = input_socket->typeinfo;
    const eNodeSocketDatatype type = typeinfo ? eNodeSocketDatatype(typeinfo->type) : SOCK_CUSTOM;

    if (!nodes::socket_type_supports_fields(type)) {
      constraints.add_unary(var, [](const int value) { return value == DomainValue::Single; });
    }

    const InputSocketFieldType field_type = inferencing_interface.inputs[input_socket->index()];
    if (field_type == InputSocketFieldType::None) {
      constraints.add_unary(input_socket->index_in_tree(),
                            [](int value) { return value == DomainValue::Single; });
    }

    /* The input must be a field value when it is connected to any output that can't be a single
     * value. */
    for (const bNodeSocket *src_socket : input_socket->directly_linked_sockets()) {
      if (src_socket->is_available()) {
        const int src_var = variables.get_socket_variable(*src_socket);
        constraints.add_binary(var, src_var, [](int value_dst, int value_src) {
          return value_dst == DomainValue::Field || value_src == DomainValue::Single;
        });
      }
    }
  }

  /* Special constraints for certain node types. */
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
      break;
    }
  }
}

static void update_socket_shapes(const bNodeTree &tree,
                                 const NodeTreeVariables &variables,
                                 const BitGroupVector<> csp_result)
{
  auto get_shape_for_state = [](const BitSpan state) {
    return state[DomainValue::Field] ?
               (state[DomainValue::Single] ? SOCK_DISPLAY_SHAPE_DIAMOND_DOT :
                                             SOCK_DISPLAY_SHAPE_DIAMOND) :
               SOCK_DISPLAY_SHAPE_CIRCLE;
  };

  for (const bNodeSocket *socket : tree.all_input_sockets()) {
    const int var = variables.get_socket_variable(*socket);
    const BitSpan state = csp_result[var];
    const_cast<bNodeSocket *>(socket)->display_shape = get_shape_for_state(state);
  }
  for (const bNodeSocket *socket : tree.all_sockets()) {
    const int var = variables.get_socket_variable(*socket);
    const BitSpan state = csp_result[var];
    const_cast<bNodeSocket *>(socket)->display_shape = get_shape_for_state(state);
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

/* Setup inferencing interface for the tree after internal field types have been resolved. */
static void determine_group_interface(
    const bNodeTree &tree,
    const Span<const FieldInferencingInterface *> interface_by_node,
    const BitGroupVector<> &field_state_by_socket_id,
    FieldInferencingInterface &inferencing_interface)
{
  NodeTreeVariables variables(tree);

  const bNode *group_output_node = tree.group_output_node();
  if (!group_output_node) {
    return;
  }

  for (const int index : tree.interface_inputs().index_range()) {
    const bNodeTreeInterfaceSocket &group_input = *tree.interface_inputs()[index];
    InputSocketFieldType &field_type = inferencing_interface.inputs[index];

    field_type = group_input_field_type(group_input);
    if (field_type == InputSocketFieldType::IsSupported) {
      const int var = variables.tree_input_vars[index];
      const BitSpan state = field_state_by_socket_id[var];
      if (state[DomainValue::Single]) {
        /* Check if group inputs are required to be single values, because they are (indirectly)
         * connected to some socket that does not support fields. */
        if (!state[DomainValue::Field]) {
          field_type = InputSocketFieldType::None;
        }
      }
      else {
        if (state[DomainValue::Field]) {
          field_type = InputSocketFieldType::Implicit;
        }
        else {
          /* Error: No supported field type. */
          BLI_assert_unreachable();
        }
      }
    }
  }

  for (const bNodeSocket *group_output_socket : group_output_node->input_sockets().drop_back(1)) {
    OutputFieldDependency field_dependency = find_group_output_dependencies(
        *group_output_socket, interface_by_node, field_state_by_socket_id);
    inferencing_interface.outputs[group_output_socket->index()] = std::move(field_dependency);
  }
}

template<typename Logger>
static void solve_field_inferencing_constraints(
    const bNodeTree &tree,
    const Span<const FieldInferencingInterface *> interface_by_node,
    FieldInferencingInterface &inferencing_interface,
    Logger &logger)
{
  tree.ensure_topology_cache();
  NodeTreeVariables variables(tree);

  logger.declare_variables(variables.num_vars(), [&](const csp::VariableIndex var) -> std::string {
    if (variables.socket_vars.contains(var)) {
      const bNodeSocket &socket = *tree.all_sockets()[var - variables.socket_vars.start()];
      const bNode &node = socket.owner_node();
      return socket.is_output() ? std::string(node.name) + ":O:" + socket.identifier :
                                  std::string(node.name) + ":I:" + socket.identifier;
    }
    if (variables.tree_input_vars.contains(var)) {
      const bNodeTreeInterfaceSocket &iosocket =
          *tree.interface_inputs()[var - variables.tree_input_vars.start()];
      return std::string("I:") + iosocket.identifier;
    }
    if (variables.tree_output_vars.contains(var)) {
      const bNodeTreeInterfaceSocket &iosocket =
          *tree.interface_outputs()[var - variables.tree_output_vars.start()];
      return std::string("O:") + iosocket.identifier;
    }
    return "";
  });

  const Span<const bNode *> nodes = tree.toposort_right_to_left();

  csp::ConstraintSet constraints;
  add_group_constraints(tree, constraints);
  for (const bNode *node : nodes) {
    add_node_constraints(tree, *node, *interface_by_node[node->index()], constraints);
  }
  logger.declare_constraints(constraints);

  BitGroupVector<> result = csp::solve_constraints_with_logger(
      constraints, variables.num_vars(), NumDomainValues, logger);

  determine_group_interface(tree, interface_by_node, result, inferencing_interface);

  /* Verify the result. */
  verify_field_inferencing_csp_result(tree, interface_by_node, result, inferencing_interface);

  update_socket_shapes(tree, variables, result);
}

template<typename Logger>
static bool update_field_inferencing_ex(const bNodeTree &tree,
                                        const bool use_constraint_solver,
                                        Logger &logger)
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

  if (use_constraint_solver) {
    solve_field_inferencing_constraints(
        tree, interface_by_node, *new_inferencing_interface, logger);
  }
  else {
    update_field_inferencing_legacy(tree, interface_by_node, *new_inferencing_interface);
  }

  /* Update the previous group interface. */
  const bool group_interface_changed = !tree.runtime->field_inferencing_interface ||
                                       *tree.runtime->field_inferencing_interface !=
                                           *new_inferencing_interface;
  tree.runtime->field_inferencing_interface = std::move(new_inferencing_interface);

  return group_interface_changed;
}

bool update_field_inferencing(const bNodeTree &tree)
{
  csp::NullLogger logger;
  return update_field_inferencing_ex(
      tree, U.experimental.use_node_field_inferencing_constraint_solver, logger);
}

bool dump_field_inferencing_debug_data(const bNodeTree &tree, StringRef filepath)
{
  std::ofstream fs;
  fs.open(filepath, std::fstream::out);
  csp::JSONLogger logger(fs);
  return update_field_inferencing_ex(tree, true, logger);
}

}  // namespace blender::bke::node_field_inferencing
