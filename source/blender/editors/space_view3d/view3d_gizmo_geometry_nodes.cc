/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

#include <iostream>

#include "WM_types.hh"

#include "DNA_modifier_types.h"

#include "BKE_context.h"

#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"

#include "view3d_intern.h" /* own include */

namespace blender::ed::view3d {

struct GeometryNodesGizmoGroup {
  wmGizmo *gizmo = nullptr;
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
  std::cout << __func__ << "\n";
  GeometryNodesGizmoGroup *gzgroup_data = MEM_new<GeometryNodesGizmoGroup>(__func__);
  wmGizmo *gz = WM_gizmo_new("GIZMO_GT_arrow_3d", gzgroup, nullptr);
  gzgroup_data->gizmo = gz;
  gzgroup->customdata = gzgroup_data;
  gzgroup->customdata_free = [](void *data) {
    auto *gzgroup_data = static_cast<GeometryNodesGizmoGroup *>(data);
    MEM_delete(gzgroup_data);
  };

  wmGizmoProperty *gz_prop = WM_gizmo_target_property_find(gz, "offset");
  {
    wmGizmoPropertyFnParams params{};
    static float my_val;
    params.value_set_fn = [](const wmGizmo *gz, wmGizmoProperty *gz_prop, const void *value_ptr) {
      const float value = *(float *)value_ptr;
      my_val = value * 2.0f;
      std::cout << "set " << value << "\n";
    };
    params.value_get_fn = [](const wmGizmo *gz, wmGizmoProperty *gz_prop, void *value_ptr) {
      std::cout << "get\n";
      *(float *)value_ptr = my_val / 2.0f;
    };
    WM_gizmo_target_property_def_func(gz, "offset", &params);
  }

  copy_v4_fl4(gz->color, 1.0f, 0.0f, 0.0f, 1.0f);
  copy_v4_fl4(gz->color_hi, 0.0f, 1.0f, 0.0f, 1.0f);
}

static void WIDGETGROUP_geometry_nodes_draw_prepare(const bContext *C, wmGizmoGroup *gzgroup)
{
  std::cout << __func__ << "\n";
  GeometryNodesGizmoGroup *gzgroup_data = static_cast<GeometryNodesGizmoGroup *>(
      gzgroup->customdata);
  Object *ob = CTX_data_active_object(C);
  wmGizmo *gz = gzgroup_data->gizmo;

  normalize_m4_m4(gz->matrix_basis, ob->object_to_world);
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
  gzgt->refresh = nullptr;
  gzgt->draw_prepare = WIDGETGROUP_geometry_nodes_draw_prepare;
}
