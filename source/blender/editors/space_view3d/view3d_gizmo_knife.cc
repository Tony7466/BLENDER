/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

#include "BKE_context.hh"

#include "ED_gizmo_library.hh"
#include "ED_gizmo_utils.hh"
#include "ED_screen.hh"

#include "RNA_access.hh"

#include "UI_resources.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "view3d_intern.h" /* own include */

/* -------------------------------------------------------------------- */
/** \name Spot knife Gizmos
 * \{ */

static void WIDGETGROUP_mesh_knife_setup(const bContext * /*C*/, wmGizmoGroup *gzgroup)
{
  wmGizmo *gz = WM_gizmo_new("GIZMO_GT_snap_3d", gzgroup, NULL);

  RNA_enum_set(
      gz->ptr, "snap_elements_force", SCE_SNAP_TO_VERTEX | SCE_SNAP_TO_EDGE | SCE_SNAP_TO_FACE);

  wmOperatorType *ot = WM_operatortype_find("MESH_OT_knife_tool", true);
  WM_gizmo_operator_set(gz, 0, ot, NULL);

  V3DSnapCursorState *snap_state = ED_view3d_cursor_snap_state_active_get();
  UI_GetThemeColorType4ubv(TH_HANDLE_SEL_VECT, SPACE_VIEW3D, snap_state->target_color);
  snap_state->flag |= (V3D_SNAPCURSOR_TOGGLE_ALWAYS_TRUE | V3D_SNAPCURSOR_OCCLUSION_ALWAYS_TRUE |
                       V3D_SNAPCURSOR_SNAP_EDIT_GEOM_ORIG_MATCHING_CAGE);
  snap_state->flag &= ~V3D_SNAPCURSOR_SNAP_EDIT_GEOM_FINAL;
}

static void WIDGETGROUP_mesh_knife_invoke_prepare(const bContext * /*C*/,
                                                  wmGizmoGroup * /*gzgroup*/,
                                                  wmGizmo *gz,
                                                  const wmEvent * /*event*/)
{
  wmGizmoOpElem *gzop = WM_gizmo_operator_get(gz, 0);
  RNA_boolean_set(&gzop->ptr, "wait_for_input", false);
}

void VIEW3D_GGT_mesh_knife_tool(wmGizmoGroupType *gzgt)
{
  gzgt->name = "knife Widget";
  gzgt->idname = "VIEW3D_GGT_mesh_knife_tool";

  gzgt->flag |= WM_GIZMOGROUPTYPE_3D | WM_GIZMOGROUPTYPE_SCALE;

  gzgt->gzmap_params.spaceid = SPACE_VIEW3D;
  gzgt->gzmap_params.regionid = RGN_TYPE_WINDOW;

  gzgt->poll = ED_gizmo_poll_or_unlink_delayed_from_tool;
  gzgt->setup = WIDGETGROUP_mesh_knife_setup;
  gzgt->invoke_prepare = WIDGETGROUP_mesh_knife_invoke_prepare;
}

/** \} */
