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

#include "BKE_context.h"
#include "BKE_modifier.h"
#include "BKE_node_runtime.hh"
#include "BKE_object.h"

#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"

#include "view3d_intern.h" /* own include */

namespace blender::ed::view3d {

struct NodeGizmoData {
  wmGizmo *gizmo;
};

struct GeometryNodesGizmoGroup {
  Map<int, std::unique_ptr<NodeGizmoData>> gizmo_by_node_id;
};

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
  std::cout << __func__ << "\n";

  auto *gzgroup_data = static_cast<GeometryNodesGizmoGroup *>(gzgroup->customdata);
  Object *ob = CTX_data_active_object(C);
  const NodesModifierData &nmd = *reinterpret_cast<const NodesModifierData *>(
      BKE_object_active_modifier(ob));
  bNodeTree &ntree = *nmd.node_group;

  Map<int, std::unique_ptr<NodeGizmoData>> new_gizmo_by_node_id;

  for (bNode *gizmo_node : ntree.nodes_by_type("GeometryNodeGizmoArrow")) {
    bNodeSocket &value_input = gizmo_node->input_socket(0);
    bNodeSocket &position_input = gizmo_node->input_socket(1);

    std::unique_ptr<NodeGizmoData> node_gizmo_data;
    if (gzgroup_data->gizmo_by_node_id.contains(gizmo_node->identifier)) {
      node_gizmo_data = gzgroup_data->gizmo_by_node_id.pop(gizmo_node->identifier);
    }
    else {
      node_gizmo_data = std::make_unique<NodeGizmoData>();
      wmGizmo *gz = WM_gizmo_new("GIZMO_GT_arrow_3d", gzgroup, nullptr);
      node_gizmo_data->gizmo = gz;
      copy_v4_fl4(gz->color, 1.0f, 0.0f, 0.0f, 1.0f);
      copy_v4_fl4(gz->color_hi, 0.0f, 1.0f, 0.0f, 1.0f);
    }

    copy_v3_v3(node_gizmo_data->gizmo->matrix_offset[3],
               position_input.default_value_typed<bNodeSocketValueVector>()->value);

    struct UserData {
      bNodeSocket *value_input_socket;
      float initial_value;
    } *user_data = MEM_new<UserData>(__func__);
    user_data->value_input_socket = &value_input;
    user_data->initial_value = value_input.default_value_typed<bNodeSocketValueFloat>()->value;

    wmGizmoPropertyFnParams params{};
    params.user_data = user_data;
    params.free_fn = [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop) {
      MEM_delete(static_cast<UserData *>(gz_prop->custom_func.user_data));
    };
    params.value_set_fn = [](const wmGizmo *gz, wmGizmoProperty *gz_prop, const void *value_ptr) {
      UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
      const float gizmo_value = *static_cast<const float *>(value_ptr);
      user_data->value_input_socket->default_value_typed<bNodeSocketValueFloat>()->value =
          gizmo_value + user_data->initial_value;
      WM_main_add_notifier(NC_NODE | NA_EDITED, nullptr);
    };
    params.value_get_fn = [](const wmGizmo *gz, wmGizmoProperty *gz_prop, void *value_ptr) {
      UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
      *(float *)value_ptr =
          user_data->value_input_socket->default_value_typed<bNodeSocketValueFloat>()->value -
          user_data->initial_value;
    };
    WM_gizmo_target_property_def_func(node_gizmo_data->gizmo, "offset", &params);

    new_gizmo_by_node_id.add(gizmo_node->identifier, std::move(node_gizmo_data));
  }

  for (std::unique_ptr<NodeGizmoData> &node_gizmo_data : gzgroup_data->gizmo_by_node_id.values()) {
    WM_gizmo_unlink(&gzgroup->gizmos,
                    gzgroup->parent_gzmap,
                    node_gizmo_data->gizmo,
                    const_cast<bContext *>(C));
  }

  gzgroup_data->gizmo_by_node_id = std::move(new_gizmo_by_node_id);
}

static void WIDGETGROUP_geometry_nodes_draw_prepare(const bContext *C, wmGizmoGroup *gzgroup)
{
  GeometryNodesGizmoGroup *gzgroup_data = static_cast<GeometryNodesGizmoGroup *>(
      gzgroup->customdata);
  Object *ob = CTX_data_active_object(C);
  for (auto item : gzgroup_data->gizmo_by_node_id.items()) {
    wmGizmo *gz = item.value->gizmo;
    normalize_m4_m4(gz->matrix_basis, ob->object_to_world);
  }
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
