/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

#include <iostream>

#include "WM_api.hh"
#include "WM_types.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"

#include "BKE_compute_contexts.hh"
#include "BKE_context.h"
#include "BKE_idprop.hh"
#include "BKE_modifier.h"
#include "BKE_node_runtime.hh"
#include "BKE_object.h"

#include "BLI_math_matrix.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_rotation.hh"
#include "BLI_math_vector.h"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "NOD_geometry_nodes_log.hh"

#include "MOD_nodes.hh"

#include "view3d_intern.h" /* own include */

namespace blender::ed::view3d {

struct NodeGizmoData {
  wmGizmo *gizmo;
};

struct GeometryNodesGizmoGroup {
  Map<int, std::unique_ptr<NodeGizmoData>> arrow_gizmo_by_node_id;
  Map<int, std::unique_ptr<NodeGizmoData>> dial_gizmo_by_node_id;
};

struct FloatValuePath {
  PointerRNA owner;
  PropertyRNA *property;
  int index = -1;

  float get()
  {
    if (this->index == -1) {
      return RNA_property_float_get(&this->owner, this->property);
    }
    return RNA_property_float_get_index(&this->owner, this->property, this->index);
  }

  void set_and_update(bContext *C, const float value)
  {
    if (this->index == -1) {
      RNA_property_float_set(&this->owner, this->property, value);
    }
    else {
      RNA_property_float_set_index(&this->owner, this->property, this->index, value);
    }
    RNA_property_update(C, &this->owner, this->property);
  }
};

struct SocketItem {
  const bNodeSocket &socket;
  int elem_index = -1;
};

static std::optional<FloatValuePath> find_float_value_path_for_input_socket(
    const SocketItem &input_socket,
    const ComputeContext &compute_context,
    const Object &object,
    const NodesModifierData &nmd);

static std::optional<FloatValuePath> find_float_value_path_for_output_socket(
    const SocketItem &output_socket,
    const ComputeContext &compute_context,
    const Object &object,
    const NodesModifierData &nmd)
{
  const bNodeTree &ntree = output_socket.socket.owner_tree();
  const bNode &node = output_socket.socket.owner_node();
  if (node.is_reroute()) {
    return find_float_value_path_for_input_socket(
        {node.input_socket(0), output_socket.elem_index}, compute_context, object, nmd);
  }
  if (node.type == SH_NODE_VALUE) {
    PointerRNA owner = RNA_pointer_create(const_cast<ID *>(&ntree.id),
                                          &RNA_NodeSocket,
                                          const_cast<bNodeSocket *>(&output_socket.socket));
    PropertyRNA *prop = RNA_struct_find_property(&owner, "default_value");
    return FloatValuePath{owner, prop};
  }
  if (node.type == FN_NODE_INPUT_VECTOR) {
    if (output_socket.elem_index < 0 || output_socket.elem_index >= 3) {
      return std::nullopt;
    }
    PointerRNA owner = RNA_pointer_create(
        const_cast<ID *>(&ntree.id), &RNA_Node, const_cast<bNode *>(&node));
    PropertyRNA *prop = RNA_struct_find_property(&owner, "vector");
    return FloatValuePath{owner, prop, output_socket.elem_index};
  }
  if (node.type == SH_NODE_SEPXYZ) {
    const int axis = output_socket.socket.index();
    const bNodeSocket &input_socket = node.input_socket(0);
    return find_float_value_path_for_input_socket(
        {input_socket, axis}, compute_context, object, nmd);
  }
  if (node.type == SH_NODE_COMBXYZ) {
    const int axis = output_socket.elem_index;
    if (axis < 0 || axis >= 3) {
      return std::nullopt;
    }
    const bNodeSocket &input_socket = node.input_socket(axis);
    return find_float_value_path_for_input_socket({input_socket}, compute_context, object, nmd);
  }
  if (node.type == NODE_GROUP_INPUT) {
    if (dynamic_cast<const bke::ModifierComputeContext *>(&compute_context)) {
      const StringRefNull input_identifier =
          ntree.interface_inputs()[output_socket.socket.index()]->identifier;
      IDProperty *id_property = IDP_GetPropertyFromGroup(nmd.settings.properties,
                                                         input_identifier.c_str());
      if (id_property == nullptr) {
        return std::nullopt;
      }
      PointerRNA owner = RNA_pointer_create(
          const_cast<ID *>(&object.id), &RNA_NodesModifier, const_cast<NodesModifierData *>(&nmd));
      PropertyRNA *prop = reinterpret_cast<PropertyRNA *>(id_property);
      if (output_socket.elem_index == -1) {
        if (id_property->type != IDP_FLOAT) {
          return std::nullopt;
        }
        return FloatValuePath{owner, prop};
      }
      else {
        if (id_property->type != IDP_ARRAY) {
          return std::nullopt;
        }
        if (id_property->subtype != IDP_FLOAT) {
          return std::nullopt;
        }
        if (id_property->len <= output_socket.elem_index) {
          return std::nullopt;
        }
        return FloatValuePath{owner, prop, output_socket.elem_index};
      }
    }
  }
  return std::nullopt;
}

static std::optional<FloatValuePath> find_float_value_path_for_input_socket(
    const SocketItem &input_socket,
    const ComputeContext &compute_context,
    const Object &object,
    const NodesModifierData &nmd)
{
  const bNodeTree &ntree = input_socket.socket.owner_tree();
  const Span<const bNodeLink *> links = input_socket.socket.directly_linked_links();
  if ((input_socket.socket.flag & SOCK_HIDE_VALUE) == 0 &&
      input_socket.socket.type == SOCK_FLOAT && links.is_empty())
  {
    PointerRNA owner = RNA_pointer_create(const_cast<ID *>(&ntree.id),
                                          &RNA_NodeSocket,
                                          const_cast<bNodeSocket *>(&input_socket.socket));
    PropertyRNA *prop = RNA_struct_find_property(&owner, "default_value");
    return FloatValuePath{owner, prop};
  }
  if (links.size() != 1) {
    return std::nullopt;
  }
  const bNodeLink &link = *links[0];
  if (link.is_muted()) {
    return std::nullopt;
  }
  const bNodeSocket &origin_socket = *link.fromsock;
  if (!origin_socket.is_available()) {
    return std::nullopt;
  }
  return find_float_value_path_for_output_socket(
      {origin_socket, input_socket.elem_index}, compute_context, object, nmd);
}

static Vector<FloatValuePath> find_float_values_paths(const bNodeSocket &gizmo_value_socket,
                                                      const ComputeContext &compute_context,
                                                      const Object &object,
                                                      const NodesModifierData &nmd)
{
  Vector<FloatValuePath> value_paths;
  const Span<const bNodeLink *> links = gizmo_value_socket.directly_linked_links();
  for (const bNodeLink *link : links) {
    if (link->is_muted()) {
      continue;
    }
    const bNodeSocket &origin_socket = *link->fromsock;
    if (!origin_socket.is_available()) {
      continue;
    }
    if (std::optional<FloatValuePath> value_path = find_float_value_path_for_output_socket(
            {origin_socket}, compute_context, object, nmd))
    {
      value_paths.append(*value_path);
    }
  }
  return value_paths;
}

static bool WIDGETGROUP_geometry_nodes_poll(const bContext *C, wmGizmoGroupType * /*gzgt*/)
{
  if (Object *object = CTX_data_active_object(C)) {
    if (ModifierData *md = BKE_object_active_modifier(object)) {
      if (md->type == eModifierType_Nodes) {
        NodesModifierData *nmd = reinterpret_cast<NodesModifierData *>(md);
        return nmd->node_group != nullptr;
      }
    }
  }
  return false;
}

static void WIDGETGROUP_geometry_nodes_setup(const bContext * /*C*/, wmGizmoGroup *gzgroup)
{
  GeometryNodesGizmoGroup *gzgroup_data = MEM_new<GeometryNodesGizmoGroup>(__func__);
  gzgroup->customdata = gzgroup_data;
  gzgroup->customdata_free = [](void *data) {
    auto *gzgroup_data = static_cast<GeometryNodesGizmoGroup *>(data);
    MEM_delete(gzgroup_data);
  };
}

static void WIDGETGROUP_geometry_nodes_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  auto *gzgroup_data = static_cast<GeometryNodesGizmoGroup *>(gzgroup->customdata);

  // for (const std::unique_ptr<NodeGizmoData> &node_gizmo_data :
  //      gzgroup_data->gizmo_by_node_id.values())
  // {
  //   wmGizmo *gz = node_gizmo_data->gizmo;
  //   if (gz->interaction_data != nullptr) {
  //     return;
  //   }
  // }

  Object *ob = CTX_data_active_object(C);
  const NodesModifierData &nmd = *reinterpret_cast<const NodesModifierData *>(
      BKE_object_active_modifier(ob));
  if (!nmd.runtime->eval_log) {
    return;
  }
  namespace geo_eval_log = nodes::geo_eval_log;
  bNodeTree &ntree = *nmd.node_group;

  bke::ModifierComputeContext compute_context{nullptr, nmd.modifier.name};
  geo_eval_log::GeoTreeLog &tree_log = nmd.runtime->eval_log->get_tree_log(compute_context.hash());
  tree_log.ensure_socket_values();

  Map<int, std::unique_ptr<NodeGizmoData>> new_arrow_gizmo_by_node_id;
  Map<int, std::unique_ptr<NodeGizmoData>> new_dial_gizmo_by_node_id;

  for (bNode *gizmo_node : ntree.nodes_by_type("GeometryNodeGizmoArrow")) {
    bNodeSocket &value_input = gizmo_node->input_socket(0);
    bNodeSocket &position_input = gizmo_node->input_socket(1);
    bNodeSocket &direction_input = gizmo_node->input_socket(2);

    Vector<FloatValuePath> value_paths = find_float_values_paths(
        value_input, compute_context, *ob, nmd);
    if (value_paths.is_empty()) {
      continue;
    }

    auto *position_value_log = dynamic_cast<geo_eval_log::GenericValueLog *>(
        tree_log.find_socket_value_log(position_input));
    auto *direction_value_log = dynamic_cast<geo_eval_log::GenericValueLog *>(
        tree_log.find_socket_value_log(direction_input));
    if (ELEM(nullptr, position_value_log, direction_value_log)) {
      continue;
    }
    const float3 position = *position_value_log->value.get<float3>();
    const float3 direction = math::normalize(*direction_value_log->value.get<float3>());

    std::unique_ptr<NodeGizmoData> node_gizmo_data;
    if (gzgroup_data->arrow_gizmo_by_node_id.contains(gizmo_node->identifier)) {
      node_gizmo_data = gzgroup_data->arrow_gizmo_by_node_id.pop(gizmo_node->identifier);
    }
    else {
      node_gizmo_data = std::make_unique<NodeGizmoData>();
      wmGizmo *gz = WM_gizmo_new("GIZMO_GT_arrow_3d", gzgroup, nullptr);
      node_gizmo_data->gizmo = gz;
      copy_v4_fl4(gz->color, 1.0f, 0.0f, 0.0f, 1.0f);
      copy_v4_fl4(gz->color_hi, 0.0f, 1.0f, 0.0f, 1.0f);
    }

    if (node_gizmo_data->gizmo->interaction_data == nullptr) {
      const math::Quaternion rotation = math::from_vector(
          math::normalize(direction), math::AxisSigned::Z_NEG, math::Axis::X);
      float4x4 mat = math::from_rotation<float4x4>(rotation);
      mat.location() = position;
      mat = float4x4(ob->object_to_world) * mat;
      copy_m4_m4(node_gizmo_data->gizmo->matrix_basis, mat.ptr());

      struct UserData {
        bContext *C;
        Vector<float> initial_values;
        Vector<FloatValuePath> value_paths;
        float gizmo_value = 0.0f;
      } *user_data = MEM_new<UserData>(__func__);
      /* The code that calls `value_set_fn` has the context, but its not passed into the callback
       * currently. */
      user_data->C = const_cast<bContext *>(C);
      for (FloatValuePath &value_path : value_paths) {
        user_data->initial_values.append(value_path.get());
      }
      user_data->value_paths = std::move(value_paths);

      wmGizmoPropertyFnParams params{};
      params.user_data = user_data;
      params.free_fn = [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop) {
        MEM_delete(static_cast<UserData *>(gz_prop->custom_func.user_data));
      };
      params.value_set_fn =
          [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop, const void *value_ptr) {
            UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
            const float new_gizmo_value = *static_cast<const float *>(value_ptr);
            user_data->gizmo_value = new_gizmo_value;
            for (const int i : user_data->value_paths.index_range()) {
              user_data->value_paths[i].set_and_update(
                  user_data->C, user_data->initial_values[i] + new_gizmo_value);
            }
          };
      params.value_get_fn = [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop, void *value_ptr) {
        UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
        *static_cast<float *>(value_ptr) = user_data->gizmo_value;
      };

      wmGizmoProperty *gz_prop = WM_gizmo_target_property_find(node_gizmo_data->gizmo, "offset");
      if (gz_prop->custom_func.free_fn) {
        gz_prop->custom_func.free_fn(node_gizmo_data->gizmo, gz_prop);
      }
      WM_gizmo_target_property_def_func(node_gizmo_data->gizmo, "offset", &params);
    }

    new_arrow_gizmo_by_node_id.add(gizmo_node->identifier, std::move(node_gizmo_data));
  }
  for (bNode *gizmo_node : ntree.nodes_by_type("GeometryNodeGizmoDial")) {
    bNodeSocket &value_input = gizmo_node->input_socket(0);
    bNodeSocket &position_input = gizmo_node->input_socket(1);
    bNodeSocket &direction_input = gizmo_node->input_socket(2);

    Vector<FloatValuePath> value_paths = find_float_values_paths(
        value_input, compute_context, *ob, nmd);
    if (value_paths.is_empty()) {
      continue;
    }

    auto *position_value_log = dynamic_cast<geo_eval_log::GenericValueLog *>(
        tree_log.find_socket_value_log(position_input));
    auto *direction_value_log = dynamic_cast<geo_eval_log::GenericValueLog *>(
        tree_log.find_socket_value_log(direction_input));
    if (ELEM(nullptr, position_value_log, direction_value_log)) {
      continue;
    }
    const float3 position = *position_value_log->value.get<float3>();
    const float3 direction = math::normalize(*direction_value_log->value.get<float3>());

    std::unique_ptr<NodeGizmoData> node_gizmo_data;
    if (gzgroup_data->dial_gizmo_by_node_id.contains(gizmo_node->identifier)) {
      node_gizmo_data = gzgroup_data->dial_gizmo_by_node_id.pop(gizmo_node->identifier);
    }
    else {
      node_gizmo_data = std::make_unique<NodeGizmoData>();
      wmGizmo *gz = WM_gizmo_new("GIZMO_GT_dial_3d", gzgroup, nullptr);
      node_gizmo_data->gizmo = gz;
      copy_v4_fl4(gz->color, 1.0f, 0.0f, 0.0f, 1.0f);
      copy_v4_fl4(gz->color_hi, 0.0f, 1.0f, 0.0f, 1.0f);
    }

    if (node_gizmo_data->gizmo->interaction_data == nullptr) {
      const math::Quaternion rotation = math::from_vector(
          math::normalize(direction), math::AxisSigned::Z_POS, math::Axis::X);
      float4x4 mat = math::from_rotation<float4x4>(rotation);
      mat.location() = position;
      mat = float4x4(ob->object_to_world) * mat;
      copy_m4_m4(node_gizmo_data->gizmo->matrix_basis, mat.ptr());

      struct UserData {
        bContext *C;
        Vector<float> initial_values;
        Vector<FloatValuePath> value_paths;
        float gizmo_value = 0.0f;
      } *user_data = MEM_new<UserData>(__func__);
      /* The code that calls `value_set_fn` has the context, but its not passed into the callback
       * currently. */
      user_data->C = const_cast<bContext *>(C);
      for (FloatValuePath &value_path : value_paths) {
        user_data->initial_values.append(value_path.get());
      }
      user_data->value_paths = std::move(value_paths);

      wmGizmoPropertyFnParams params{};
      params.user_data = user_data;
      params.free_fn = [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop) {
        MEM_delete(static_cast<UserData *>(gz_prop->custom_func.user_data));
      };
      params.value_set_fn =
          [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop, const void *value_ptr) {
            UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
            const float new_gizmo_value = *static_cast<const float *>(value_ptr);
            user_data->gizmo_value = new_gizmo_value;
            for (const int i : user_data->value_paths.index_range()) {
              user_data->value_paths[i].set_and_update(
                  user_data->C, user_data->initial_values[i] + new_gizmo_value);
            }
          };
      params.value_get_fn = [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop, void *value_ptr) {
        UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
        *static_cast<float *>(value_ptr) = user_data->gizmo_value;
      };

      wmGizmoProperty *gz_prop = WM_gizmo_target_property_find(node_gizmo_data->gizmo, "offset");
      if (gz_prop->custom_func.free_fn) {
        gz_prop->custom_func.free_fn(node_gizmo_data->gizmo, gz_prop);
      }
      WM_gizmo_target_property_def_func(node_gizmo_data->gizmo, "offset", &params);
    }

    new_dial_gizmo_by_node_id.add(gizmo_node->identifier, std::move(node_gizmo_data));
  }

  for (std::unique_ptr<NodeGizmoData> &node_gizmo_data :
       gzgroup_data->arrow_gizmo_by_node_id.values())
  {
    WM_gizmo_unlink(&gzgroup->gizmos,
                    gzgroup->parent_gzmap,
                    node_gizmo_data->gizmo,
                    const_cast<bContext *>(C));
  }
  for (std::unique_ptr<NodeGizmoData> &node_gizmo_data :
       gzgroup_data->dial_gizmo_by_node_id.values())
  {
    WM_gizmo_unlink(&gzgroup->gizmos,
                    gzgroup->parent_gzmap,
                    node_gizmo_data->gizmo,
                    const_cast<bContext *>(C));
  }

  gzgroup_data->arrow_gizmo_by_node_id = std::move(new_arrow_gizmo_by_node_id);
  gzgroup_data->dial_gizmo_by_node_id = std::move(new_dial_gizmo_by_node_id);
}

static void WIDGETGROUP_geometry_nodes_draw_prepare(const bContext * /*C*/,
                                                    wmGizmoGroup * /*gzgroup*/)
{
  // GeometryNodesGizmoGroup *gzgroup_data = static_cast<GeometryNodesGizmoGroup *>(
  //     gzgroup->customdata);
  // Object *ob = CTX_data_active_object(C);
  // for (auto item : gzgroup_data->gizmo_by_node_id.items()) {
  //   wmGizmo *gz = item.value->gizmo;
  //   normalize_m4_m4(gz->matrix_basis, ob->object_to_world);
  // }
}

}  // namespace blender::ed::view3d

void VIEW3D_GGT_geometry_nodes(wmGizmoGroupType *gzgt)
{
  using namespace blender::ed::view3d;

  gzgt->name = "Geometry Nodes Widgets";
  gzgt->idname = "VIEW3D_GGT_geometry_nodes";

  gzgt->flag |= (WM_GIZMOGROUPTYPE_PERSISTENT | WM_GIZMOGROUPTYPE_3D | WM_GIZMOGROUPTYPE_DEPTH_3D);

  gzgt->poll = WIDGETGROUP_geometry_nodes_poll;
  gzgt->setup = WIDGETGROUP_geometry_nodes_setup;
  gzgt->setup_keymap = WM_gizmogroup_setup_keymap_generic_maybe_drag;
  gzgt->refresh = WIDGETGROUP_geometry_nodes_refresh;
  gzgt->draw_prepare = WIDGETGROUP_geometry_nodes_draw_prepare;
}
