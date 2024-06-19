/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <fmt/format.h>
#include <queue>

#include "NOD_inverse_eval_path.hh"
#include "NOD_inverse_eval_run.hh"
#include "NOD_node_in_compute_context.hh"

#include "BKE_anim_data.hh"
#include "BKE_compute_contexts.hh"
#include "BKE_idprop.hh"
#include "BKE_modifier.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_type_conversions.hh"

#include "BLI_map.hh"
#include "BLI_math_euler.hh"
#include "BLI_math_matrix.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"

#include "DEG_depsgraph.hh"

#include "DNA_anim_types.h"

#include "ED_node.hh"

#include "RNA_access.hh"
#include "RNA_path.hh"

#include "MOD_nodes.hh"

namespace blender::nodes::inverse_eval {

static bool is_supported_value_node(const bNode &node)
{
  return ELEM(node.type,
              SH_NODE_VALUE,
              FN_NODE_INPUT_VECTOR,
              FN_NODE_INPUT_BOOL,
              FN_NODE_INPUT_INT,
              FN_NODE_INPUT_ROTATION);
}

static Vector<int> get_global_node_sort_vector(const ComputeContext *initial_context,
                                               const bNode &initial_node)
{
  Vector<int> vec;
  vec.append(initial_node.runtime->toposort_right_to_left_index);
  for (const ComputeContext *context = initial_context; context; context = context->parent()) {
    if (const auto *group_context = dynamic_cast<const bke::GroupNodeComputeContext *>(context)) {
      const bNode *caller_group_node = group_context->caller_group_node();
      BLI_assert(caller_group_node != nullptr);
      vec.append(caller_group_node->runtime->toposort_right_to_left_index);
    }
  }
  std::reverse(vec.begin(), vec.end());
  return vec;
}

/**
 * Defines a partial order of #NodeInContext that can be used to evaluate nodes right to left
 * (upstream).
 * - Downstream nodes are sorted before upstream nodes.
 * - Nodes inside a node group are sorted before the group node.
 */
struct NodeInContextUpstreamComparator {
  bool operator()(const NodeInContext &a, const NodeInContext &b) const
  {
    const Vector<int> a_sort_vec = get_global_node_sort_vector(a.context, *a.node);
    const Vector<int> b_sort_vec = get_global_node_sort_vector(b.context, *b.node);
    const int common_length = std::min(a_sort_vec.size(), b_sort_vec.size());
    const Span<int> a_common = Span<int>(a_sort_vec).take_front(common_length);
    const Span<int> b_common = Span<int>(b_sort_vec).take_front(common_length);
    if (a_common == b_common) {
      return a_sort_vec.size() < b_sort_vec.size();
    }
    return std::lexicographical_compare(
        b_common.begin(), b_common.end(), a_common.begin(), a_common.end());
  }
};

static void traverse_upstream(
    const Span<SocketInContext> initial_sockets,
    ResourceScope &scope,
    FunctionRef<void(const NodeInContext &ctx_node,
                     Vector<const bNodeSocket *> &r_modified_inputs)> evaluate_node,
    FunctionRef<bool(const SocketInContext &ctx_from, const SocketInContext &ctx_to)>
        propagate_value,
    FunctionRef<void(const NodeInContext &ctx_node, Vector<const bNodeSocket *> &r_sockets)>
        get_inputs_to_propagate,
    Set<SocketInContext> &r_final_sockets,
    Set<NodeInContext> &r_final_value_nodes)
{
  Set<NodeInContext> scheduled_nodes_set;
  std::priority_queue<NodeInContext, std::vector<NodeInContext>, NodeInContextUpstreamComparator>
      scheduled_nodes_queue;

  const auto schedule_node = [&](const NodeInContext &ctx_node) {
    if (scheduled_nodes_set.add(ctx_node)) {
      scheduled_nodes_queue.push(ctx_node);
    }
  };

  const auto forward_group_node_output_into_group = [&](const SocketInContext &ctx_output_socket) {
    const ComputeContext *context = ctx_output_socket.context;
    const bNode &group_node = ctx_output_socket.socket->owner_node();
    const bNodeTree *group = reinterpret_cast<const bNodeTree *>(group_node.id);
    if (!group) {
      return;
    }
    group->ensure_topology_cache();
    if (group->has_available_link_cycle()) {
      return;
    }
    const bNode *group_output = group->group_output_node();
    if (!group_output) {
      return;
    }
    const ComputeContext &group_context = scope.construct<bke::GroupNodeComputeContext>(
        context, group_node, group_node.owner_tree());
    propagate_value(
        ctx_output_socket,
        {&group_context, &group_output->input_socket(ctx_output_socket.socket->index())});
    schedule_node({&group_context, group_output});
  };

  const auto forward_group_input_to_parent = [&](const SocketInContext &ctx_output_socket) {
    const auto *group_context = dynamic_cast<const bke::GroupNodeComputeContext *>(
        ctx_output_socket.context);
    if (!group_context) {
      return;
    }
    const bNodeTree &caller_tree = *group_context->caller_tree();
    caller_tree.ensure_topology_cache();
    const bNode &caller_node = *group_context->caller_group_node();
    const bNodeSocket &caller_input_socket = caller_node.input_socket(
        ctx_output_socket.socket->index());
    const ComputeContext *parent_context = ctx_output_socket.context->parent();
    propagate_value(ctx_output_socket, {parent_context, &caller_input_socket});
    schedule_node({parent_context, &caller_node});
  };

  const auto forward_input = [&](const SocketInContext &ctx_input_socket) {
    const ComputeContext *context = ctx_input_socket.context;
    if (!ctx_input_socket.socket->is_logically_linked()) {
      r_final_sockets.add(ctx_input_socket);
      return;
    }
    for (const bNodeLink *link : ctx_input_socket.socket->directly_linked_links()) {
      if (!link->is_used()) {
        continue;
      }
      const bNode &origin_node = *link->fromnode;
      const bNodeSocket &origin_socket = *link->fromsock;
      if (!propagate_value(ctx_input_socket, {context, &origin_socket})) {
        continue;
      }
      schedule_node({context, &origin_node});
      if (origin_node.is_group()) {
        forward_group_node_output_into_group({context, &origin_socket});
        continue;
      }
      if (origin_node.is_group_input()) {
        forward_group_input_to_parent({context, &origin_socket});
        continue;
      }
    }
  };

  for (const SocketInContext &ctx_socket : initial_sockets) {
    if (ctx_socket.socket->is_input()) {
      forward_input(ctx_socket);
    }
    else {
      const bNode &node = ctx_socket.socket->owner_node();
      if (node.is_group()) {
        forward_group_node_output_into_group(ctx_socket);
      }
      else if (node.is_group_input()) {
        forward_group_input_to_parent(ctx_socket);
      }
      else {
        schedule_node({ctx_socket.context, &ctx_socket.socket->owner_node()});
      }
    }
  }

  /* Reused in multiple places to avoid allocating it multiple times. Should be cleared before
   * using it. */
  Vector<const bNodeSocket *> sockets_vec;

  while (!scheduled_nodes_queue.empty()) {
    const NodeInContext ctx_node = scheduled_nodes_queue.top();
    scheduled_nodes_queue.pop();

    const bNode &node = *ctx_node.node;
    const ComputeContext *context = ctx_node.context;

    if (is_supported_value_node(node)) {
      r_final_value_nodes.add(ctx_node);
    }
    else if (node.is_reroute()) {
      propagate_value({context, &node.output_socket(0)}, {context, &node.input_socket(0)});
      forward_input({context, &node.input_socket(0)});
    }
    else if (node.is_group()) {
      const bke::GroupNodeComputeContext group_context{context, node, node.owner_tree()};
      sockets_vec.clear();
      get_inputs_to_propagate(ctx_node, sockets_vec);
      for (const bNodeSocket *socket : sockets_vec) {
        forward_input({context, socket});
      }
    }
    else if (node.is_group_output()) {
      sockets_vec.clear();
      get_inputs_to_propagate(ctx_node, sockets_vec);
      for (const bNodeSocket *socket : sockets_vec) {
        forward_input({context, socket});
      }
    }
    else {
      sockets_vec.clear();
      evaluate_node(ctx_node, sockets_vec);
      for (const bNodeSocket *input_socket : sockets_vec) {
        forward_input({context, input_socket});
      }
    }
  }
}

std::optional<ElemVariant> get_elem_variant_for_socket_type(const eNodeSocketDatatype type)
{
  switch (type) {
    case SOCK_FLOAT:
      return {{FloatElem()}};
    case SOCK_INT:
      return {{IntElem()}};
    case SOCK_BOOLEAN:
      return {{BoolElem()}};
    case SOCK_VECTOR:
      return {{VectorElem()}};
    case SOCK_ROTATION:
      return {{RotationElem()}};
    case SOCK_MATRIX:
      return {{MatrixElem()}};
    default:
      return std::nullopt;
  }
}

std::optional<ElemVariant> convert_socket_elem(const bNodeSocket &old_socket,
                                               const bNodeSocket &new_socket,
                                               const ElemVariant &old_elem)
{
  const eNodeSocketDatatype old_type = eNodeSocketDatatype(old_socket.type);
  const eNodeSocketDatatype new_type = eNodeSocketDatatype(new_socket.type);
  if (old_type == new_type) {
    return old_elem;
  }
  if (ELEM(old_type, SOCK_INT, SOCK_FLOAT, SOCK_BOOLEAN) &&
      ELEM(new_type, SOCK_INT, SOCK_FLOAT, SOCK_BOOLEAN))
  {
    std::optional<ElemVariant> new_elem = get_elem_variant_for_socket_type(new_type);
    if (old_elem) {
      new_elem->set_all();
    }
    return new_elem;
  }
  switch (old_type) {
    case SOCK_MATRIX: {
      const MatrixElem &transform_elem = std::get<MatrixElem>(old_elem.elem);
      if (new_type == SOCK_ROTATION) {
        return ElemVariant{transform_elem.rotation};
      }
      break;
    }
    default:
      break;
  }
  return std::nullopt;
}

std::optional<SocketValueVariant> convert_socket_value(const bNodeSocket &old_socket,
                                                       const bNodeSocket &new_socket,
                                                       const SocketValueVariant &old_value)
{
  const eNodeSocketDatatype old_type = eNodeSocketDatatype(old_socket.type);
  const eNodeSocketDatatype new_type = eNodeSocketDatatype(new_socket.type);
  if (old_type == new_type) {
    return old_value;
  }
  const CPPType *old_cpp_type = old_socket.typeinfo->base_cpp_type;
  const CPPType *new_cpp_type = new_socket.typeinfo->base_cpp_type;
  if (!old_cpp_type || !new_cpp_type) {
    return std::nullopt;
  }
  const bke::DataTypeConversions &type_conversions = bke::get_implicit_type_conversions();
  if (type_conversions.is_convertible(*old_cpp_type, *new_cpp_type)) {
    const void *old_value_ptr = old_value.get_single_ptr_raw();
    SocketValueVariant new_value;
    void *new_value_ptr = new_value.allocate_single(new_type);
    type_conversions.convert_to_uninitialized(
        *old_cpp_type, *new_cpp_type, old_value_ptr, new_value_ptr);
    return new_value;
  }
  return std::nullopt;
}

namespace traverse_elem {

static void evaluate_node(const NodeInContext &ctx_node,
                          Vector<const bNodeSocket *> &r_modified_inputs,
                          Map<SocketInContext, ElemVariant> &elem_by_socket)
{
  const bNode &node = *ctx_node.node;
  const bke::bNodeType &ntype = *node.typeinfo;
  if (!ntype.eval_inverse_elem) {
    /* Node does not support inverse evaluation. */
    return;
  }
  Vector<SocketElem> input_elems;
  Map<const bNodeSocket *, ElemVariant> elem_by_local_socket;
  for (const bNodeSocket *output_socket : node.output_sockets()) {
    if (const ElemVariant *elem = elem_by_socket.lookup_ptr({ctx_node.context, output_socket})) {
      elem_by_local_socket.add(output_socket, *elem);
    }
  }
  InverseElemEvalParams params{node, elem_by_local_socket, input_elems};
  ntype.eval_inverse_elem(params);
  for (const SocketElem &input_elem : input_elems) {
    if (input_elem.elem) {
      elem_by_socket.add({ctx_node.context, input_elem.socket}, input_elem.elem);
      r_modified_inputs.append(input_elem.socket);
    }
  }
}

static bool propagate_value(const SocketInContext &ctx_from,
                            const SocketInContext &ctx_to,
                            Map<SocketInContext, ElemVariant> &elem_by_socket)
{
  const ElemVariant *from_elem = elem_by_socket.lookup_ptr(ctx_from);
  if (!from_elem) {
    return false;
  }
  const std::optional<ElemVariant> to_elem = convert_socket_elem(
      *ctx_from.socket, *ctx_to.socket, *from_elem);
  if (!to_elem || !*to_elem) {
    return false;
  }
  elem_by_socket.lookup_or_add(ctx_to, *to_elem).merge(*to_elem);
  return true;
}

static void get_inputs_to_propagate(const NodeInContext &ctx_node,
                                    Vector<const bNodeSocket *> &r_sockets,
                                    Map<SocketInContext, ElemVariant> &elem_by_socket)
{
  for (const bNodeSocket *socket : ctx_node.node->input_sockets()) {
    if (elem_by_socket.contains({ctx_node.context, socket})) {
      r_sockets.append(socket);
    }
  }
}

}  // namespace traverse_elem

LocalInverseEvalPath find_local_inverse_eval_path(const bNodeTree &tree,
                                                  const SocketElem &initial_socket_elem)
{
  BLI_assert(!tree.has_available_link_cycle());

  tree.ensure_topology_cache();

  ResourceScope scope;
  Map<SocketInContext, ElemVariant> elem_by_socket;
  elem_by_socket.add({nullptr, initial_socket_elem.socket}, initial_socket_elem.elem);

  Set<SocketInContext> final_sockets;
  Set<NodeInContext> final_value_nodes;

  traverse_upstream(
      {{nullptr, initial_socket_elem.socket}},
      scope,
      /* Evaluate node. */
      [&](const NodeInContext &ctx_node, Vector<const bNodeSocket *> &r_modified_inputs) {
        traverse_elem::evaluate_node(ctx_node, r_modified_inputs, elem_by_socket);
      },
      /* Propagate value. */
      [&](const SocketInContext &ctx_from, const SocketInContext &ctx_to) {
        return traverse_elem::propagate_value(ctx_from, ctx_to, elem_by_socket);
      },
      /* Get input sockets to propagate. */
      [&](const NodeInContext &ctx_node, Vector<const bNodeSocket *> &r_sockets) {
        traverse_elem::get_inputs_to_propagate(ctx_node, r_sockets, elem_by_socket);
      },
      final_sockets,
      final_value_nodes);

  LocalInverseEvalPath path;

  for (const SocketInContext &ctx_socket : final_sockets) {
    if (ctx_socket.context) {
      /* Context should be empty because we only handle top-level sockets here. */
      continue;
    }
    const ElemVariant *elem = elem_by_socket.lookup_ptr(ctx_socket);
    if (!elem || !*elem) {
      continue;
    }
    path.final_input_sockets.append({ctx_socket.socket, *elem});
  }

  for (const NodeInContext ctx_node : final_value_nodes) {
    if (ctx_node.context) {
      /* Context should be empty because we only handle top-level nodes here. */
      continue;
    }
    const bNodeSocket &socket = ctx_node.node->output_socket(0);
    const ElemVariant *elem = elem_by_socket.lookup_ptr({nullptr, &socket});
    if (!elem || !*elem) {
      continue;
    }
    path.final_value_nodes.append({ctx_node.node, *elem});
  }

  for (const int group_input_index : tree.interface_inputs().index_range()) {
    const eNodeSocketDatatype type = eNodeSocketDatatype(
        tree.interface_inputs()[group_input_index]->socket_typeinfo()->type);
    std::optional<ElemVariant> elem = get_elem_variant_for_socket_type(type);
    if (!elem) {
      continue;
    }
    for (const bNode *node : tree.group_input_nodes()) {
      const bNodeSocket &socket = node->output_socket(group_input_index);
      if (const ElemVariant *socket_elem = elem_by_socket.lookup_ptr({nullptr, &socket})) {
        elem->merge(*socket_elem);
      }
    }
    if (!*elem) {
      continue;
    }
    path.final_group_inputs.append({group_input_index, *elem});
  }

  for (auto &&item : elem_by_socket.items()) {
    const bNodeSocket &socket = *item.key.socket;
    const ElemVariant &elem = item.value;
    path.intermediate_sockets.append({&socket, elem});
  }

  return path;
}

InverseElemEvalParams::InverseElemEvalParams(
    const bNode &node,
    const Map<const bNodeSocket *, ElemVariant> &elem_by_socket,
    Vector<SocketElem> &input_elems)
    : elem_by_socket_(elem_by_socket), input_elems_(input_elems), node(node)
{
}

InverseEvalParams::InverseEvalParams(
    const bNode &node,
    const Map<const bNodeSocket *, bke::SocketValueVariant> &socket_values,
    Map<const bNodeSocket *, bke::SocketValueVariant> &updated_socket_values)
    : socket_values_(socket_values), updated_socket_values_(updated_socket_values), node(node)
{
}

void foreach_node_on_inverse_eval_path(
    const ComputeContext &initial_context,
    const SocketElem &initial_socket_elem,
    FunctionRef<void(const ComputeContext &context, const bNode &node)> fn)
{
  BLI_assert(initial_socket_elem.socket->is_input());
  if (!initial_socket_elem.elem) {
    return;
  }
  ResourceScope scope;
  Map<SocketInContext, ElemVariant> elem_by_socket;
  elem_by_socket.add({&initial_context, initial_socket_elem.socket}, initial_socket_elem.elem);

  Set<SocketInContext> final_sockets;
  Set<NodeInContext> final_value_nodes;

  traverse_upstream(
      {{&initial_context, initial_socket_elem.socket}},
      scope,
      /* Evaluate node. */
      [&](const NodeInContext &ctx_node, Vector<const bNodeSocket *> &r_modified_inputs) {
        traverse_elem::evaluate_node(ctx_node, r_modified_inputs, elem_by_socket);
        fn(*ctx_node.context, *ctx_node.node);
      },
      /* Propagate value. */
      [&](const SocketInContext &ctx_from, const SocketInContext &ctx_to) {
        return traverse_elem::propagate_value(ctx_from, ctx_to, elem_by_socket);
      },
      /* Get input sockets to propagate. */
      [&](const NodeInContext &ctx_node, Vector<const bNodeSocket *> &r_sockets) {
        traverse_elem::get_inputs_to_propagate(ctx_node, r_sockets, elem_by_socket);
      },
      final_sockets,
      final_value_nodes);
}

static bool set_socket_value(bNodeSocket &socket, const SocketValueVariant &value_variant)
{
  switch (socket.type) {
    case SOCK_FLOAT: {
      const float value = value_variant.get<float>();
      auto *default_value = socket.default_value_typed<bNodeSocketValueFloat>();
      default_value->value = std::min(std::max(value, default_value->min), default_value->max);
      return true;
    }
    case SOCK_INT: {
      const int value = value_variant.get<int>();
      auto *default_value = socket.default_value_typed<bNodeSocketValueInt>();
      default_value->value = std::min(std::max(value, default_value->min), default_value->max);
      return true;
    }
    case SOCK_BOOLEAN: {
      const bool value = value_variant.get<bool>();
      auto *default_value = socket.default_value_typed<bNodeSocketValueBoolean>();
      default_value->value = value;
      return true;
    }
    case SOCK_VECTOR: {
      const float3 value = value_variant.get<float3>();
      auto *default_value = socket.default_value_typed<bNodeSocketValueVector>();
      *reinterpret_cast<float3 *>(default_value->value) = value;
      return true;
    }
    case SOCK_ROTATION: {
      const math::Quaternion value = value_variant.get<math::Quaternion>();
      auto *default_value = socket.default_value_typed<bNodeSocketValueRotation>();
      *reinterpret_cast<float3 *>(default_value->value_euler) = float3(math::to_euler(value));
      return true;
    }
  }
  return false;
}

static bool set_value_node_value(bNode &node, const SocketValueVariant &value_variant)
{
  switch (node.type) {
    case SH_NODE_VALUE: {
      bNodeSocket &socket = node.output_socket(0);
      auto *default_value = socket.default_value_typed<bNodeSocketValueFloat>();
      default_value->value = value_variant.get<float>();
      return true;
    }
    case FN_NODE_INPUT_INT: {
      NodeInputInt &storage = *static_cast<NodeInputInt *>(node.storage);
      storage.integer = value_variant.get<int>();
      return true;
    }
    case FN_NODE_INPUT_VECTOR: {
      NodeInputVector &storage = *static_cast<NodeInputVector *>(node.storage);
      *reinterpret_cast<float3 *>(storage.vector) = value_variant.get<float3>();
      return true;
    }
    case FN_NODE_INPUT_BOOL: {
      NodeInputBool &storage = *static_cast<NodeInputBool *>(node.storage);
      storage.boolean = value_variant.get<bool>();
      return true;
    }
    case FN_NODE_INPUT_ROTATION: {
      NodeInputRotation &storage = *static_cast<NodeInputRotation *>(node.storage);
      *reinterpret_cast<float3 *>(storage.rotation_euler) = float3(
          math::to_euler(value_variant.get<math::Quaternion>()));
      return true;
    }
  }
  return false;
}

using DriverValueVariant = std::variant<float, int, bool>;
static bool set_rna_property_inverse(bContext &C,
                                     ID &id,
                                     const StringRefNull rna_path,
                                     const DriverValueVariant &value);

[[nodiscard]] static bool try_set_driver_source_value(bContext &C,
                                                      ID &id,
                                                      const StringRefNull rna_path,
                                                      const std::optional<int> index,
                                                      const DriverValueVariant &value_variant)
{
  AnimData *adt = BKE_animdata_from_id(&id);
  if (!adt) {
    return false;
  }
  LISTBASE_FOREACH (FCurve *, driver, &adt->drivers) {
    if (driver->rna_path != rna_path) {
      continue;
    }
    if (index.has_value()) {
      if (driver->array_index != *index) {
        continue;
      }
    }
    if (!driver->driver) {
      continue;
    }
    if (!ELEM(driver->driver->type,
              DRIVER_TYPE_AVERAGE,
              DRIVER_TYPE_SUM,
              DRIVER_TYPE_MIN,
              DRIVER_TYPE_MAX))
    {
      continue;
    }
    if (BLI_listbase_count(&driver->driver->variables) != 1) {
      continue;
    }
    DriverVar &driver_var = *static_cast<DriverVar *>(driver->driver->variables.first);
    if (driver_var.type != DVAR_TYPE_SINGLE_PROP) {
      continue;
    }
    if (driver_var.num_targets != 1) {
      continue;
    }
    DriverTarget &driver_target = driver_var.targets[0];
    return set_rna_property_inverse(C, *driver_target.id, driver_target.rna_path, value_variant);
  }
  return false;
}

static bool set_rna_property_inverse(bContext &C,
                                     ID &id,
                                     const StringRefNull rna_path,
                                     const DriverValueVariant &value_variant)
{
  if (!ID_IS_EDITABLE(&id)) {
    return false;
  }

  PointerRNA id_ptr = RNA_id_pointer_create(&id);
  PointerRNA value_ptr;
  PropertyRNA *prop;
  int index;
  if (!RNA_path_resolve_property_full(&id_ptr, rna_path.c_str(), &value_ptr, &prop, &index)) {
    return false;
  }

  const std::optional<std::string> rna_path_without_index = RNA_path_from_ID_to_property(
      &value_ptr, prop);
  if (!rna_path_without_index) {
    return false;
  }

  std::optional<int> index_opt;
  if (index >= 0) {
    index_opt = index;
  }
  if (try_set_driver_source_value(C, id, *rna_path_without_index, index_opt, value_variant)) {
    return true;
  }

  const PropertyType dst_type = RNA_property_type(prop);
  const int array_len = RNA_property_array_length(&value_ptr, prop);

  switch (dst_type) {
    case PROP_FLOAT: {
      const float value = std::visit([](auto v) { return float(v); }, value_variant);
      if (array_len == 0) {
        RNA_property_float_set(&value_ptr, prop, value);
        RNA_property_update(&C, &value_ptr, prop);
        return true;
      }
      if (index >= 0 && index < array_len) {
        RNA_property_float_set_index(&value_ptr, prop, index, value);
        RNA_property_update(&C, &value_ptr, prop);
        return true;
      }
      break;
    }
    case PROP_INT: {
      const int value = std::visit([](auto v) { return int(v); }, value_variant);
      if (array_len == 0) {
        RNA_property_int_set(&value_ptr, prop, value);
        RNA_property_update(&C, &value_ptr, prop);
        return true;
      }
      if (index >= 0 && index < array_len) {
        RNA_property_int_set_index(&value_ptr, prop, index, value);
        RNA_property_update(&C, &value_ptr, prop);
        return true;
      }
      break;
    }
    case PROP_BOOLEAN: {
      const bool value = std::visit([](auto v) { return bool(v); }, value_variant);
      if (array_len == 0) {
        RNA_property_boolean_set(&value_ptr, prop, value);
        RNA_property_update(&C, &value_ptr, prop);
        return true;
      }
      if (index >= 0 && index < array_len) {
        RNA_property_boolean_set_index(&value_ptr, prop, index, value);
        RNA_property_update(&C, &value_ptr, prop);
        return true;
      }
      break;
    }
    default:
      break;
  };

  return false;
}

static bool set_modifier_value(bContext &C,
                               Object &object,
                               NodesModifierData &nmd,
                               const bNodeTreeInterfaceSocket &interface_socket,
                               const SocketValueVariant &value_variant)
{
  DEG_id_tag_update(&object.id, ID_RECALC_GEOMETRY);

  const std::string main_prop_rna_path = fmt::format(
      "modifiers[\"{}\"][\"{}\"]", nmd.modifier.name, interface_socket.identifier);

  switch (interface_socket.socket_typeinfo()->type) {
    case SOCK_FLOAT: {
      const float value = value_variant.get<float>();
      return set_rna_property_inverse(C, object.id, main_prop_rna_path, value);
    }
    case SOCK_INT: {
      const int value = value_variant.get<int>();
      return set_rna_property_inverse(C, object.id, main_prop_rna_path, value);
    }
    case SOCK_BOOLEAN: {
      const bool value = value_variant.get<bool>();
      return set_rna_property_inverse(C, object.id, main_prop_rna_path, value);
    }
    case SOCK_VECTOR: {
      const float3 value = value_variant.get<float3>();
      bool any_success = false;
      for (const int i : IndexRange(3)) {
        const std::string rna_path = fmt::format("{}[{}]", main_prop_rna_path, i);
        any_success |= set_rna_property_inverse(C, object.id, rna_path, value[i]);
      }
      return any_success;
    }
    case SOCK_ROTATION: {
      const math::Quaternion rotation = value_variant.get<math::Quaternion>();
      const math::EulerXYZ euler = math::to_euler(rotation);
      const float3 euler_vec{euler};
      bool any_success = false;
      for (const int i : IndexRange(3)) {
        const std::string rna_path = fmt::format("{}[{}]", main_prop_rna_path, i);
        any_success |= set_rna_property_inverse(C, object.id, rna_path, euler_vec[i]);
      }
      return any_success;
    }
  }
  return false;
}

std::optional<SocketValueVariant> get_logged_socket_value(geo_eval_log::GeoTreeLog &tree_log,
                                                          const bNodeSocket &socket)
{
  switch (socket.type) {
    case SOCK_FLOAT: {
      if (const std::optional<float> value = tree_log.find_primitive_socket_value<float>(socket)) {
        return SocketValueVariant{*value};
      }
      break;
    }
    case SOCK_INT: {
      if (const std::optional<int> value = tree_log.find_primitive_socket_value<int>(socket)) {
        return SocketValueVariant{*value};
      }
      break;
    }
    case SOCK_BOOLEAN: {
      if (const std::optional<bool> value = tree_log.find_primitive_socket_value<bool>(socket)) {
        return SocketValueVariant{*value};
      }
      break;
    }
    case SOCK_VECTOR: {
      if (const std::optional<float3> value = tree_log.find_primitive_socket_value<float3>(socket))
      {
        return SocketValueVariant{*value};
      }
      break;
    }
    case SOCK_ROTATION: {
      if (const std::optional<math::Quaternion> value =
              tree_log.find_primitive_socket_value<math::Quaternion>(socket))
      {
        return SocketValueVariant{*value};
      }
      break;
    }
    case SOCK_MATRIX: {
      if (const std::optional<float4x4> value = tree_log.find_primitive_socket_value<float4x4>(
              socket))
      {
        return SocketValueVariant{*value};
      }
      break;
    }
  }
  return std::nullopt;
}

bool try_change_link_target_and_update_source(bContext &C,
                                              Object &object,
                                              NodesModifierData &nmd,
                                              geo_eval_log::GeoModifierLog &eval_log,
                                              const ComputeContext *initial_context,
                                              const bNodeLink &initial_link,
                                              const SocketValueVariant &new_value)
{
  nmd.node_group->ensure_topology_cache();

  ResourceScope scope;
  Map<SocketInContext, SocketValueVariant> value_by_socket;

  const std::optional<SocketValueVariant> initial_converted_value = convert_socket_value(
      *initial_link.tosock, *initial_link.fromsock, new_value);
  if (!initial_converted_value) {
    return false;
  }
  value_by_socket.add({initial_context, initial_link.fromsock}, *initial_converted_value);

  Set<SocketInContext> final_sockets;
  Set<NodeInContext> final_value_nodes;

  traverse_upstream(
      {{initial_context, initial_link.fromsock}},
      scope,
      /* Evaluate node. */
      [&](const NodeInContext &ctx_node, Vector<const bNodeSocket *> &r_modified_inputs) {
        const bNode &node = *ctx_node.node;
        const ComputeContext *context = ctx_node.context;
        const bke::bNodeType &ntype = *node.typeinfo;
        if (!ntype.eval_inverse) {
          /* Node does not support inverse evaluation. */
          return;
        }
        if (!context) {
          /* We need a context here to access the tree log. */
          return;
        }
        geo_eval_log::GeoTreeLog &tree_log = eval_log.get_tree_log(context->hash());
        tree_log.ensure_socket_values();
        Map<const bNodeSocket *, SocketValueVariant> old_socket_values;
        for (const bNodeSocket *socket : node.input_sockets()) {
          if (!socket->is_available()) {
            continue;
          }
          if (const std::optional<SocketValueVariant> value = get_logged_socket_value(tree_log,
                                                                                      *socket))
          {
            old_socket_values.add(socket, *value);
          }
        }
        for (const bNodeSocket *socket : node.output_sockets()) {
          if (!socket->is_available()) {
            continue;
          }
          if (const SocketValueVariant *value = value_by_socket.lookup_ptr({context, socket})) {
            old_socket_values.add(socket, *value);
          }
          else if (const std::optional<SocketValueVariant> value = get_logged_socket_value(
                       tree_log, *socket))
          {
            old_socket_values.add(socket, *value);
          }
        }

        Map<const bNodeSocket *, SocketValueVariant> updated_socket_values;
        InverseEvalParams params{node, old_socket_values, updated_socket_values};
        ntype.eval_inverse(params);
        for (auto &&item : updated_socket_values.items()) {
          const bNodeSocket &socket = *item.key;
          value_by_socket.add({context, &socket}, std::move(item.value));
          r_modified_inputs.append(&socket);
        }
      },
      /* Propagate value. */
      [&](const SocketInContext &ctx_from, const SocketInContext &ctx_to) {
        const SocketValueVariant *from_value = value_by_socket.lookup_ptr(ctx_from);
        if (!from_value) {
          return false;
        }
        const std::optional<SocketValueVariant> converted_value = convert_socket_value(
            *ctx_from.socket, *ctx_to.socket, *from_value);
        if (!converted_value) {
          return false;
        }
        value_by_socket.add(ctx_to, std::move(*converted_value));
        return true;
      },
      /* Get input sockets to propagate. */
      [&](const NodeInContext &ctx_node, Vector<const bNodeSocket *> &r_sockets) {
        for (const bNodeSocket *socket : ctx_node.node->input_sockets()) {
          if (value_by_socket.contains({ctx_node.context, socket})) {
            r_sockets.append(socket);
          }
        }
      },
      final_sockets,
      final_value_nodes);

  Vector<std::pair<bNodeTree *, bNodeSocket *>> modified_sockets;
  Vector<std::pair<bNodeTree *, bNode *>> modified_nodes;

  for (const SocketInContext &ctx_socket : final_sockets) {
    if (const SocketValueVariant *value = value_by_socket.lookup_ptr(ctx_socket)) {
      bNodeSocket &socket_mutable = const_cast<bNodeSocket &>(*ctx_socket.socket);
      if (set_socket_value(socket_mutable, *value)) {
        modified_sockets.append({&socket_mutable.owner_tree(), &socket_mutable});
      }
    }
  }
  for (const NodeInContext &ctx_node : final_value_nodes) {
    if (const SocketValueVariant *value = value_by_socket.lookup_ptr(
            {ctx_node.context, &ctx_node.node->output_socket(0)}))
    {
      bNode &node_mutable = const_cast<bNode &>(*ctx_node.node);
      if (set_value_node_value(node_mutable, *value)) {
        modified_nodes.append({&node_mutable.owner_tree(), &node_mutable});
      }
    }
  }

  const bke::ModifierComputeContext modifier_context{nullptr, nmd.modifier.name};
  for (const bNode *group_input_node : nmd.node_group->group_input_nodes()) {
    for (const bNodeSocket *socket : group_input_node->output_sockets().drop_back(1)) {
      if (const SocketValueVariant *value = value_by_socket.lookup_ptr(
              {&modifier_context, socket}))
      {
        set_modifier_value(
            C, object, nmd, *nmd.node_group->interface_inputs()[socket->index()], *value);
      }
    }
  }

  for (auto &&[tree, socket] : modified_sockets) {
    BKE_ntree_update_tag_socket_property(tree, socket);
  }
  for (auto &&[tree, node] : modified_nodes) {
    BKE_ntree_update_tag_node_property(tree, node);
  }

  return true;
}

}  // namespace blender::nodes::inverse_eval
