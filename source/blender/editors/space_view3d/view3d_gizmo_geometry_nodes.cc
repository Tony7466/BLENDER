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

#include "NOD_geometry_nodes_gizmos.hh"
#include "NOD_geometry_nodes_log.hh"

#include "MOD_nodes.hh"

#include "UI_resources.hh"

#include "ED_gizmo_library.hh"

#include "view3d_intern.h" /* own include */

namespace blender::ed::view3d {
namespace geo_eval_log = nodes::geo_eval_log;

struct NodeGizmoData {
  wmGizmo *gizmo;
};

struct GeometryNodesGizmoGroup {
  Map<int, std::unique_ptr<NodeGizmoData>> arrow_gizmo_by_node_id;
  Map<int, std::unique_ptr<NodeGizmoData>> dial_gizmo_by_node_id;
};

struct SocketItem {
  const bNodeSocket &socket;
  int elem_index = -1;
};

struct FloatValuePath {
  PointerRNA owner;
  PropertyRNA *property;
  std::optional<int> index;

  float get()
  {
    const PropertyType prop_type = RNA_property_type(this->property);
    const bool prop_is_array = RNA_property_array_check(this->property);
    if (prop_is_array) {
      BLI_assert(this->index.has_value());
      if (prop_type == PROP_FLOAT) {
        return RNA_property_float_get_index(&this->owner, this->property, *this->index);
      }
      else if (prop_type == PROP_INT) {
        return RNA_property_int_get_index(&this->owner, this->property, *this->index);
      }
    }
    else {
      if (prop_type == PROP_FLOAT) {
        return RNA_property_float_get(&this->owner, this->property);
      }
      else if (prop_type == PROP_INT) {
        return RNA_property_int_get(&this->owner, this->property);
      }
    }
    BLI_assert_unreachable();
    return 0.0f;
  }

  void set_and_update(bContext *C, const float value)
  {
    const PropertyType prop_type = RNA_property_type(this->property);
    const bool prop_is_array = RNA_property_array_check(this->property);

    if (prop_is_array) {
      BLI_assert(this->index.has_value());
      if (prop_type == PROP_FLOAT) {
        RNA_property_float_set_index(&this->owner, this->property, *this->index, value);
      }
      else if (prop_type == PROP_INT) {
        RNA_property_int_set_index(&this->owner, this->property, *this->index, value);
      }
    }
    else {
      if (prop_type == PROP_FLOAT) {
        RNA_property_float_set(&this->owner, this->property, value);
      }
      else if (prop_type == PROP_INT) {
        RNA_property_int_set(&this->owner, this->property, value);
      }
    }
    RNA_property_update(C, &this->owner, this->property);
  }
};

struct GizmoFloatVariable {
  FloatValuePath path;
  float derivative;
};

static Vector<GizmoFloatVariable> find_float_values_paths(const bNodeSocket &gizmo_value_socket,
                                                          const ComputeContext &compute_context,
                                                          const Object &object,
                                                          const NodesModifierData &nmd)
{
  BLI_assert(gizmo_value_socket.is_input());
  const bNodeTree &ntree = *nmd.node_group;
  Vector<GizmoFloatVariable> value_paths;
  const Span<const bNodeLink *> links = gizmo_value_socket.directly_linked_links();
  for (const bNodeLink *link : links) {
    if (link->is_muted()) {
      continue;
    }
    const bNodeSocket *origin_socket = link->fromsock;
    if (!origin_socket->is_available()) {
      continue;
    }
    float derivative = 1.0f;
    const bNode &origin_node = origin_socket->owner_node();
    if (origin_node.type == GEO_NODE_GIZMO_VARIABLE) {
      geo_eval_log::GeoTreeLog &tree_log = nmd.runtime->eval_log->get_tree_log(
          compute_context.hash());
      tree_log.ensure_socket_values();
      auto *derivative_value_log = dynamic_cast<geo_eval_log::GenericValueLog *>(
          tree_log.find_socket_value_log(origin_node.input_socket(1)));
      if (derivative_value_log == nullptr) {
        continue;
      }
      derivative = *derivative_value_log->value.get<float>();

      const bNodeSocket *input_socket = &origin_node.input_socket(0);
      if (input_socket->directly_linked_links().size() != 1) {
        continue;
      }
      const bNodeLink *origin_link = input_socket->directly_linked_links()[0];
      if (link->is_muted() || !link->fromsock->is_available()) {
        continue;
      }
      origin_socket = origin_link->fromsock;
    }

    const std::optional<nodes::gizmos::GizmoSource> gizmo_source_opt =
        nodes::gizmos::find_scalar_gizmo_source(*origin_socket);
    if (!gizmo_source_opt) {
      continue;
    }

    FloatValuePath value_path;

    ID *ntree_id = const_cast<ID *>(&ntree.id);

    if (const auto *gizmo_source = std::get_if<nodes::gizmos::InputSocketGizmoSource>(
            &*gizmo_source_opt))
    {
      value_path.owner = RNA_pointer_create(
          ntree_id, &RNA_NodeSocket, const_cast<bNodeSocket *>(gizmo_source->input_socket));
      value_path.property = RNA_struct_find_property(&value_path.owner, "default_value");
    }
    else if (const auto *gizmo_source = std::get_if<nodes::gizmos::ValueNodeGizmoSource>(
                 &*gizmo_source_opt))
    {
      switch (gizmo_source->value_node->type) {
        case SH_NODE_VALUE: {
          value_path.owner = RNA_pointer_create(
              ntree_id,
              &RNA_NodeSocket,
              const_cast<bNodeSocket *>(&gizmo_source->value_node->output_socket(0)));
          value_path.property = RNA_struct_find_property(&value_path.owner, "default_value");
          break;
        }
        case FN_NODE_INPUT_VECTOR: {
          value_path.owner = RNA_pointer_create(
              ntree_id, &RNA_Node, const_cast<bNode *>(gizmo_source->value_node));
          value_path.property = RNA_struct_find_property(&value_path.owner, "vector");
          value_path.index = *gizmo_source->elem_index;
          break;
        }
        case FN_NODE_INPUT_INT: {
          value_path.owner = RNA_pointer_create(
              ntree_id, &RNA_Node, const_cast<bNode *>(gizmo_source->value_node));
          value_path.property = RNA_struct_find_property(&value_path.owner, "integer");
          break;
        }
      }
    }
    else if (const auto *gizmo_source = std::get_if<nodes::gizmos::GroupInputGizmoSource>(
                 &*gizmo_source_opt))
    {
      const StringRefNull input_identifier =
          ntree.interface_inputs()[gizmo_source->interface_input_index]->identifier;
      IDProperty *id_property = IDP_GetPropertyFromGroup(nmd.settings.properties,
                                                         input_identifier.c_str());
      if (id_property == nullptr) {
        continue;
      }
      value_path.owner = RNA_pointer_create(
          const_cast<ID *>(&object.id), &RNA_NodesModifier, const_cast<NodesModifierData *>(&nmd));
      value_path.property = reinterpret_cast<PropertyRNA *>(id_property);
      value_path.index = gizmo_source->elem_index;
    }
    else {
      BLI_assert_unreachable();
    }

    value_paths.append({value_path, derivative});
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

  Object *ob = CTX_data_active_object(C);
  const NodesModifierData &nmd = *reinterpret_cast<const NodesModifierData *>(
      BKE_object_active_modifier(ob));
  if (!nmd.runtime->eval_log) {
    return;
  }

  bNodeTree &ntree = *nmd.node_group;
  ntree.ensure_topology_cache();

  bke::ModifierComputeContext compute_context{nullptr, nmd.modifier.name};
  geo_eval_log::GeoTreeLog &tree_log = nmd.runtime->eval_log->get_tree_log(compute_context.hash());
  tree_log.ensure_socket_values();

  Map<int, std::unique_ptr<NodeGizmoData>> new_arrow_gizmo_by_node_id;
  Map<int, std::unique_ptr<NodeGizmoData>> new_dial_gizmo_by_node_id;

  for (bNode *gizmo_node : ntree.nodes_by_type("GeometryNodeGizmoArrow")) {
    bNodeSocket &value_input = gizmo_node->input_socket(0);
    bNodeSocket &position_input = gizmo_node->input_socket(1);
    bNodeSocket &direction_input = gizmo_node->input_socket(2);

    Vector<GizmoFloatVariable> variables = find_float_values_paths(
        value_input, compute_context, *ob, nmd);
    if (variables.is_empty()) {
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
      UI_GetThemeColor3fv(TH_GIZMO_PRIMARY, gz->color);
      UI_GetThemeColor3fv(TH_GIZMO_HI, gz->color_hi);
    }

    const bool is_interacting = node_gizmo_data->gizmo->interaction_data != nullptr;

    struct UserData {
      bContext *C;
      Vector<float> initial_values;
      Vector<GizmoFloatVariable> variables;
      float gizmo_value = 0.0f;
    };

    wmGizmoProperty *gz_prop = WM_gizmo_target_property_find(node_gizmo_data->gizmo, "offset");
    if (is_interacting) {
      UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
      for (const int i : user_data->variables.index_range()) {
        const GizmoFloatVariable &new_variable = variables[i];
        GizmoFloatVariable &variable = user_data->variables[i];
        variable.derivative = new_variable.derivative;
      }
    }
    else {
      const math::Quaternion rotation = math::from_vector(
          math::normalize(direction), math::AxisSigned::Z_NEG, math::Axis::X);
      float4x4 mat = math::from_rotation<float4x4>(rotation);
      mat.location() = position;
      mat = float4x4(ob->object_to_world) * mat;
      copy_m4_m4(node_gizmo_data->gizmo->matrix_basis, mat.ptr());

      UserData *user_data = MEM_new<UserData>(__func__);
      /* The code that calls `value_set_fn` has the context, but its not passed into the callback
       * currently. */
      user_data->C = const_cast<bContext *>(C);
      for (GizmoFloatVariable &variable : variables) {
        user_data->initial_values.append(variable.path.get());
      }
      user_data->variables = std::move(variables);

      wmGizmoPropertyFnParams params{};
      params.user_data = user_data;
      params.free_fn = [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop) {
        MEM_delete(static_cast<UserData *>(gz_prop->custom_func.user_data));
      };
      params.value_set_fn = [](const wmGizmo * /*gz*/,
                               wmGizmoProperty *gz_prop,
                               const void *value_ptr) {
        UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
        const float new_gizmo_value = *static_cast<const float *>(value_ptr);
        user_data->gizmo_value = new_gizmo_value;
        for (const int i : user_data->variables.index_range()) {
          GizmoFloatVariable &variable = user_data->variables[i];
          variable.path.set_and_update(
              user_data->C, user_data->initial_values[i] + new_gizmo_value * variable.derivative);
        }
      };
      params.value_get_fn = [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop, void *value_ptr) {
        UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
        *static_cast<float *>(value_ptr) = user_data->gizmo_value;
      };

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

    Vector<GizmoFloatVariable> variables = find_float_values_paths(
        value_input, compute_context, *ob, nmd);
    if (variables.is_empty()) {
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
      UI_GetThemeColor3fv(TH_GIZMO_PRIMARY, gz->color);
      UI_GetThemeColor3fv(TH_GIZMO_HI, gz->color_hi);
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
        Vector<GizmoFloatVariable> variables;
        float gizmo_value = 0.0f;
      } *user_data = MEM_new<UserData>(__func__);
      /* The code that calls `value_set_fn` has the context, but its not passed into the callback
       * currently. */
      user_data->C = const_cast<bContext *>(C);
      for (GizmoFloatVariable &variable : variables) {
        user_data->initial_values.append(variable.path.get());
      }
      user_data->variables = std::move(variables);

      wmGizmoPropertyFnParams params{};
      params.user_data = user_data;
      params.free_fn = [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop) {
        MEM_delete(static_cast<UserData *>(gz_prop->custom_func.user_data));
      };
      params.value_set_fn = [](const wmGizmo * /*gz*/,
                               wmGizmoProperty *gz_prop,
                               const void *value_ptr) {
        UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
        const float new_gizmo_value = *static_cast<const float *>(value_ptr);
        user_data->gizmo_value = new_gizmo_value;
        for (const int i : user_data->variables.index_range()) {
          GizmoFloatVariable &variable = user_data->variables[i];
          variable.path.set_and_update(
              user_data->C, user_data->initial_values[i] + new_gizmo_value * variable.derivative);
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
