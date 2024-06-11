/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <queue>

#include "NOD_inverse_eval_path.hh"
#include "NOD_inverse_eval_run.hh"

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

#include "ED_node.hh"

#include "MOD_nodes.hh"

namespace blender::nodes::inverse_eval {

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
      return {{TransformElem()}};
    default:
      return std::nullopt;
  }
}

static bool is_supported_value_node(const bNode &node)
{
  return ELEM(node.type,
              SH_NODE_VALUE,
              FN_NODE_INPUT_VECTOR,
              FN_NODE_INPUT_BOOL,
              FN_NODE_INPUT_INT,
              FN_NODE_INPUT_ROTATION);
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
  switch (old_type) {
    case SOCK_MATRIX: {
      const TransformElem &transform_elem = std::get<TransformElem>(old_elem.elem);
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

LocalInverseEvalPath find_local_inverse_eval_path(const bNodeTree &tree,
                                                  const SocketElem &initial_socket_elem)
{
  BLI_assert(!tree.has_available_link_cycle());

  tree.ensure_topology_cache();

  Map<const bNodeSocket *, ElemVariant> elem_by_socket_map;
  Set<const bNodeSocket *> final_sockets;
  Set<int> final_group_inputs;
  Set<const bNode *> final_value_nodes;

  Stack<SocketElem> sockets_to_handle;
  sockets_to_handle.push(initial_socket_elem);

  while (!sockets_to_handle.is_empty()) {
    const SocketElem socket_elem = sockets_to_handle.pop();
    const bNodeSocket &socket = *socket_elem.socket;

    ElemVariant &elem_variant = elem_by_socket_map.lookup_or_add_cb(&socket, [&]() {
      return *get_elem_variant_for_socket_type(eNodeSocketDatatype(socket.type));
    });
    const ElemVariant old_elem_variant = elem_variant;
    elem_variant.merge(socket_elem.elem);
    if (elem_variant == old_elem_variant) {
      /* Nothing changed. */
      continue;
    }

    if (socket.is_input()) {
      const Span<const bNodeLink *> links = socket.directly_linked_links();
      if (links.size() >= 2) {
        /* Can't continue propagation. */
        continue;
      }
      const bNodeSocket *origin_socket = nullptr;
      if (links.size() == 1) {
        const bNodeLink &link = *links[0];
        if (link.is_used()) {
          origin_socket = link.fromsock;
        }
      }
      if (origin_socket) {
        const std::optional<ElemVariant> origin_elem = convert_socket_elem(
            socket, *origin_socket, elem_variant);
        if (origin_elem) {
          sockets_to_handle.push(SocketElem{origin_socket, *origin_elem});
        }
        else {
          /* Can't continue propagation. */
          continue;
        }
      }
      else {
        final_sockets.add(&socket);
      }
    }
    else {
      const bNode &node = socket.owner_node();
      if (is_supported_value_node(node)) {
        final_value_nodes.add(&node);
      }
      else if (node.is_group_input()) {
        final_group_inputs.add(socket.index());
      }
      else if (node.is_reroute()) {
        sockets_to_handle.push({&node.input_socket(0), elem_variant});
      }
      else {
        const bke::bNodeType &ntype = *node.typeinfo;
        if (!ntype.eval_inverse_elem) {
          /* Node does not support inverse propagation. */
          continue;
        }
        Vector<SocketElem> input_elems;
        InverseElemEvalParams params{node, elem_by_socket_map, input_elems};
        ntype.eval_inverse_elem(params);
        for (const SocketElem &input_elem : input_elems) {
          if (input_elem.elem) {
            sockets_to_handle.push(input_elem);
          }
        }
      }
    }
  }

  LocalInverseEvalPath path;

  for (const bNodeSocket *socket : final_sockets) {
    const ElemVariant &elem = elem_by_socket_map.lookup(socket);
    if (!elem) {
      continue;
    }
    path.final_input_sockets.append({socket, elem});
  }

  for (const bNode *value_node : final_value_nodes) {
    const bNodeSocket &socket = value_node->output_socket(0);
    const ElemVariant &elem = elem_by_socket_map.lookup(&socket);
    if (!elem) {
      continue;
    }
    path.final_value_nodes.append({value_node, elem});
  }

  for (const int group_input_index : final_group_inputs) {
    const eNodeSocketDatatype type = eNodeSocketDatatype(
        tree.interface_inputs()[group_input_index]->socket_typeinfo()->type);
    std::optional<ElemVariant> elem = get_elem_variant_for_socket_type(type);
    if (!elem) {
      continue;
    }
    for (const bNode *node : tree.group_input_nodes()) {
      const bNodeSocket &socket = node->output_socket(group_input_index);
      if (const ElemVariant *socket_elem = elem_by_socket_map.lookup_ptr(&socket)) {
        elem->merge(*socket_elem);
      }
    }
    if (!*elem) {
      continue;
    }
    path.final_group_inputs.append({group_input_index, *elem});
  }

  for (auto &&item : elem_by_socket_map.items()) {
    const bNodeSocket &socket = *item.key;
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

struct NodeInContext {
  const ComputeContext *context = nullptr;
  const bNode *node = nullptr;

  uint64_t hash() const
  {
    return get_default_hash(this->context, this->node);
  }

  friend bool operator<(const NodeInContext &a, const NodeInContext &b)
  {
    const Vector<int> a_sort_vec = get_global_node_sort_vector(a.context, *a.node);
    const Vector<int> b_sort_vec = get_global_node_sort_vector(b.context, *b.node);
    return std::lexicographical_compare(
        b_sort_vec.begin(), b_sort_vec.end(), a_sort_vec.begin(), a_sort_vec.end());
  }

  BLI_STRUCT_EQUALITY_OPERATORS_2(NodeInContext, context, node)
};

struct SocketInContext {
  const ComputeContext *context = nullptr;
  const bNodeSocket *socket = nullptr;

  uint64_t hash() const
  {
    return get_default_hash(this->context, this->socket);
  }

  BLI_STRUCT_EQUALITY_OPERATORS_2(SocketInContext, context, socket)
};

GlobalInverseEvalPath find_global_inverse_eval_path(const ComputeContext *initial_context,
                                                    const SocketElem &initial_socket_elem)
{
  BLI_assert(initial_socket_elem.socket->is_input());
  if (!initial_socket_elem.elem) {
    return {};
  }
  Map<SocketInContext, ElemVariant> elem_by_socket;

  Set<NodeInContext> added_nodes;
  std::priority_queue<NodeInContext> nodes_to_handle;

  const auto update_input_elem_and_forward =
      [&](const ComputeContext *context, const bNodeSocket &socket, const ElemVariant &new_elem) {
        ElemVariant &elem = elem_by_socket.lookup_or_add({context, &socket}, new_elem);
        elem.merge(new_elem);
        const ElemVariant elem_to_forward = elem;
        for (const bNodeLink *link : socket.directly_linked_links()) {
          if (!link->is_used()) {
            continue;
          }
          const bNodeSocket &origin_socket = *link->fromsock;
          const bNode &origin_node = *link->fromnode;
          if (origin_node.is_group()) {
            /* Not yet supported. */
          }
          else {
            const std::optional<ElemVariant> converted_elem = convert_socket_elem(
                socket, origin_socket, elem_to_forward);
            if (!converted_elem) {
              continue;
            }
            ElemVariant &origin_elem = elem_by_socket.lookup_or_add({context, &origin_socket},
                                                                    *converted_elem);
            origin_elem.merge(*converted_elem);
            const NodeInContext origin_node_in_context{context, &origin_node};
            if (added_nodes.add(origin_node_in_context)) {
              nodes_to_handle.push(origin_node_in_context);
            }
          }
        }
      };

  update_input_elem_and_forward(
      initial_context, *initial_socket_elem.socket, initial_socket_elem.elem);

  GlobalInverseEvalPath path;
  while (!nodes_to_handle.empty()) {
    const NodeInContext node_in_context = nodes_to_handle.top();
    nodes_to_handle.pop();

    const bNode &node = *node_in_context.node;
    const ComputeContext *context = node_in_context.context;
    path.ordered_nodes.append({context, &node});

    if (is_supported_value_node(node)) {
      /* Inverse evaluation ends here. */
    }
    else if (node.is_reroute()) {
      const ElemVariant elem = elem_by_socket.lookup({context, &node.output_socket(0)});
      update_input_elem_and_forward(context, node.input_socket(0), elem);
    }
    else if (node.is_group_input()) {
      if (const auto *group_context = dynamic_cast<const bke::GroupNodeComputeContext *>(context))
      {
        const bNode *caller_group_node = group_context->caller_group_node();
        BLI_assert(caller_group_node);
        const ComputeContext *caller_context = context->parent();

        for (const bNodeSocket *socket : node.output_sockets()) {
          if (const ElemVariant *elem = elem_by_socket.lookup_ptr({context, socket})) {
            if (*elem) {
              const bNodeSocket &caller_input_socket = caller_group_node->input_socket(
                  socket->index());
              update_input_elem_and_forward(caller_context, caller_input_socket, *elem);
            }
          }
        }
      }
    }
    else {
      const bke::bNodeType &ntype = *node.typeinfo;
      if (!ntype.eval_inverse_elem) {
        /* Node does not support inverse evaluation. */
        continue;
      }
      Vector<SocketElem> input_elems;
      Map<const bNodeSocket *, ElemVariant> elem_by_local_socket;
      for (const bNodeSocket *output_socket : node.output_sockets()) {
        if (const ElemVariant *elem = elem_by_socket.lookup_ptr({context, output_socket})) {
          elem_by_local_socket.add(output_socket, *elem);
        }
      }
      InverseElemEvalParams params{node, elem_by_local_socket, input_elems};
      ntype.eval_inverse_elem(params);
      for (const SocketElem &input_elem : input_elems) {
        if (input_elem.elem) {
          update_input_elem_and_forward(context, *input_elem.socket, input_elem.elem);
        }
      }
    }
  }

  return path;
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

static bool set_modifier_value(Object &object,
                               NodesModifierData &nmd,
                               const bNodeTreeInterfaceSocket &interface_socket,
                               const SocketValueVariant &value_variant)
{
  DEG_id_tag_update(&object.id, ID_RECALC_GEOMETRY);

  /* TODO: Take min/max into account. */
  switch (interface_socket.socket_typeinfo()->type) {
    case SOCK_FLOAT: {
      const float value = value_variant.get<float>();
      IDProperty *prop = IDP_GetPropertyFromGroup(nmd.settings.properties,
                                                  interface_socket.identifier);
      if (prop && prop->type == IDP_FLOAT) {
        IDP_Float(prop) = value;
        return true;
      }
      break;
    }
    case SOCK_INT: {
      const int value = value_variant.get<float>();
      IDProperty *prop = IDP_GetPropertyFromGroup(nmd.settings.properties,
                                                  interface_socket.identifier);
      if (prop && prop->type == IDP_INT) {
        IDP_Int(prop) = value;
        return true;
      }
      break;
    }
    case SOCK_BOOLEAN: {
      const bool value = value_variant.get<bool>();
      IDProperty *prop = IDP_GetPropertyFromGroup(nmd.settings.properties,
                                                  interface_socket.identifier);
      if (prop && prop->type == IDP_BOOLEAN) {
        IDP_Bool(prop) = value;
        return true;
      }
      break;
    }
    case SOCK_VECTOR: {
      const float3 value = value_variant.get<float3>();
      IDProperty *prop = IDP_GetPropertyFromGroup(nmd.settings.properties,
                                                  interface_socket.identifier);
      if (prop && prop->type == IDP_ARRAY && prop->len == 3 && prop->subtype == IDP_FLOAT) {
        *static_cast<float3 *>(IDP_Array(prop)) = value;
      }
      break;
    }
    case SOCK_ROTATION: {
      const math::Quaternion rotation = value_variant.get<math::Quaternion>();
      const math::EulerXYZ euler = math::to_euler(rotation);
      IDProperty *prop = IDP_GetPropertyFromGroup(nmd.settings.properties,
                                                  interface_socket.identifier);
      if (prop && prop->type == IDP_ARRAY && prop->len == 3 && prop->subtype == IDP_FLOAT) {
        *static_cast<float3 *>(IDP_Array(prop)) = float3(euler);
        return true;
      }
      break;
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

  /* TODO */
  return {};
}

bool try_change_link_target_and_update_source(Object &object,
                                              NodesModifierData &nmd,
                                              geo_eval_log::GeoModifierLog &eval_log,
                                              const ComputeContext *initial_context,
                                              const bNodeLink &initial_link,
                                              const SocketValueVariant &new_value)
{
  Map<SocketInContext, SocketValueVariant> value_by_socket;

  Set<NodeInContext> added_nodes;
  std::priority_queue<NodeInContext> nodes_to_handle;

  Vector<std::pair<bNodeTree *, bNodeSocket *>> modified_sockets;
  Vector<std::pair<bNodeTree *, bNode *>> modified_nodes;

  const auto set_input_value_and_forward = [&](const ComputeContext *context,
                                               const bNodeSocket &socket,
                                               const SocketValueVariant &new_value) {
    value_by_socket.add_overwrite({context, &socket}, new_value);
    if (!socket.is_logically_linked()) {
      bNodeSocket &socket_mutable = const_cast<bNodeSocket &>(socket);
      set_socket_value(socket_mutable, new_value);
      modified_sockets.append({&socket_mutable.owner_tree(), &socket_mutable});
      return;
    }
    for (const bNodeLink *link : socket.directly_linked_links()) {
      if (!link->is_used()) {
        continue;
      }
      const bNodeSocket &origin_socket = *link->fromsock;
      const bNode &origin_node = *link->fromnode;
      if (origin_node.is_group()) {
        /* Not yet supported. */
      }
      else {
        const std::optional<SocketValueVariant> converted_value = convert_socket_value(
            socket, origin_socket, new_value);
        if (!converted_value) {
          continue;
        }
        value_by_socket.add_overwrite({context, &origin_socket}, *converted_value);
        const NodeInContext origin_node_in_context{context, &origin_node};
        if (added_nodes.add(origin_node_in_context)) {
          nodes_to_handle.push(origin_node_in_context);
        }
      }
    }
  };

  const std::optional<SocketValueVariant> initial_converted_value = convert_socket_value(
      *initial_link.tosock, *initial_link.fromsock, new_value);
  if (!initial_converted_value) {
    return false;
  }
  value_by_socket.add({initial_context, initial_link.fromsock}, *initial_converted_value);
  const NodeInContext initial_node_in_context{initial_context, initial_link.fromnode};
  added_nodes.add(initial_node_in_context);
  nodes_to_handle.push(initial_node_in_context);

  while (!nodes_to_handle.empty()) {
    const NodeInContext node_in_context = nodes_to_handle.top();
    nodes_to_handle.pop();

    const bNode &node = *node_in_context.node;
    const ComputeContext *context = node_in_context.context;

    if (is_supported_value_node(node)) {
      const SocketValueVariant &new_value = value_by_socket.lookup(
          {context, &node.output_socket(0)});
      bNode &node_mutable = const_cast<bNode &>(node);
      set_value_node_value(node_mutable, new_value);
      modified_nodes.append({&node_mutable.owner_tree(), &node_mutable});
    }
    else if (node.is_reroute()) {
      const SocketValueVariant &value = value_by_socket.lookup({context, &node.output_socket(0)});
      set_input_value_and_forward(context, node.input_socket(0), value);
    }
    else if (node.is_group_input()) {
      if (const auto *group_context = dynamic_cast<const bke::GroupNodeComputeContext *>(context))
      {
        const bNode *caller_group_node = group_context->caller_group_node();
        BLI_assert(caller_group_node);
        const ComputeContext *caller_context = context->parent();

        for (const bNodeSocket *socket : node.output_sockets()) {
          if (const SocketValueVariant *value = value_by_socket.lookup_ptr({context, socket})) {
            const bNodeSocket &caller_input_socket = caller_group_node->input_socket(
                socket->index());
            set_input_value_and_forward(caller_context, caller_input_socket, *value);
          }
        }
      }
      else if (dynamic_cast<const bke::ModifierComputeContext *>(context)) {
        for (const bNodeSocket *socket : node.output_sockets()) {
          if (const SocketValueVariant *value = value_by_socket.lookup_ptr({context, socket})) {
            const bNodeTreeInterfaceSocket &interface_socket =
                *node.owner_tree().interface_inputs()[socket->index()];
            set_modifier_value(object, nmd, interface_socket, *value);
          }
        }
      }
    }
    else {
      const bke::bNodeType &ntype = *node.typeinfo;
      if (!ntype.eval_inverse) {
        /* Node does not support inverse evaluation. */
        continue;
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
        else if (const std::optional<SocketValueVariant> value = get_logged_socket_value(tree_log,
                                                                                         *socket))
        {
          old_socket_values.add(socket, *value);
        }
      }

      Map<const bNodeSocket *, SocketValueVariant> updated_socket_values;
      InverseEvalParams params{node, old_socket_values, updated_socket_values};
      ntype.eval_inverse(params);
      for (auto &&item : updated_socket_values.items()) {
        const bNodeSocket &socket = *item.key;
        const SocketValueVariant &value = item.value;
        set_input_value_and_forward(context, socket, value);
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
