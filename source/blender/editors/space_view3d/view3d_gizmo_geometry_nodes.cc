/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

#include <iostream>

#include "WM_types.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"

#include "BKE_context.h"
#include "BKE_node_runtime.hh"

#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"

#include "view3d_intern.h" /* own include */

namespace blender::ed::view3d {

struct GeometryNodesGizmoGroup {
  Vector<wmGizmo *> gizmos;
  Vector<float> gizmo_values;
};

static bool WIDGETGROUP_geometry_nodes_poll(const bContext *C, wmGizmoGroupType * /*gzgt*/)
{
  Object *object = CTX_data_active_object(C);
  LISTBASE_FOREACH (ModifierData *, md, &object->modifiers) {
    if (md->type == eModifierType_Nodes) {
      return true;
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

static void ensure_gizmo_amount(const bContext *C, wmGizmoGroup *gzgroup, const int new_gizmo_num)
{
  GeometryNodesGizmoGroup *gzgroup_data = static_cast<GeometryNodesGizmoGroup *>(
      gzgroup->customdata);
  if (new_gizmo_num < gzgroup_data->gizmos.size()) {
    for (const int i : gzgroup_data->gizmos.index_range().drop_front(new_gizmo_num)) {
      WM_gizmo_unlink(&gzgroup->gizmos,
                      gzgroup->parent_gzmap,
                      gzgroup_data->gizmos[i],
                      const_cast<bContext *>(C));
    }
    gzgroup_data->gizmos.resize(new_gizmo_num);
    gzgroup_data->gizmo_values.resize(new_gizmo_num);
  }
  else if (new_gizmo_num > gzgroup_data->gizmos.size()) {
    for (const int i : IndexRange(new_gizmo_num).drop_front(gzgroup_data->gizmos.size())) {
      wmGizmo *gz = WM_gizmo_new("GIZMO_GT_arrow_3d", gzgroup, nullptr);
      gzgroup_data->gizmo_values.append(0.0f);
      gzgroup_data->gizmos.append(gz);
      copy_v4_fl4(gz->color, 1.0f, 0.0f, 0.0f, 1.0f);
      copy_v4_fl4(gz->color_hi, 0.0f, 1.0f, 0.0f, 1.0f);

      struct UserData {
        GeometryNodesGizmoGroup *gzgroup_data;
        int index;
      } *user_data = MEM_new<UserData>(__func__);
      user_data->gzgroup_data = gzgroup_data;
      user_data->index = i;

      wmGizmoPropertyFnParams params{};
      params.user_data = user_data;
      params.free_fn = [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop) {
        MEM_delete(static_cast<UserData *>(gz_prop->custom_func.user_data));
      };
      params.value_set_fn =
          [](const wmGizmo *gz, wmGizmoProperty *gz_prop, const void *value_ptr) {
            UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
            user_data->gzgroup_data->gizmo_values[user_data->index] = *static_cast<const float *>(
                value_ptr);
          };
      params.value_get_fn = [](const wmGizmo *gz, wmGizmoProperty *gz_prop, void *value_ptr) {
        UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
        *static_cast<float *>(value_ptr) = user_data->gzgroup_data->gizmo_values[user_data->index];
      };
      WM_gizmo_target_property_def_func(gz, "offset", &params);
    }
  }
}

static void WIDGETGROUP_geometry_nodes_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  Object *ob = CTX_data_active_object(C);
  LISTBASE_FOREACH (ModifierData *, md, &ob->modifiers) {
    if (md->type != eModifierType_Nodes) {
      continue;
    }
    NodesModifierData &nmd = *reinterpret_cast<NodesModifierData *>(md);
    const int num = nmd.node_group ? nmd.node_group->runtime->nodes_by_id.size() : 0;
    ensure_gizmo_amount(C, gzgroup, num);
  }
}

static void WIDGETGROUP_geometry_nodes_draw_prepare(const bContext *C, wmGizmoGroup *gzgroup)
{
  GeometryNodesGizmoGroup *gzgroup_data = static_cast<GeometryNodesGizmoGroup *>(
      gzgroup->customdata);
  Object *ob = CTX_data_active_object(C);
  for (const int i : gzgroup_data->gizmos.index_range()) {
    wmGizmo *gz = gzgroup_data->gizmos[i];
    normalize_m4_m4(gz->matrix_basis, ob->object_to_world);
    gz->matrix_basis[3][0] += i;
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
