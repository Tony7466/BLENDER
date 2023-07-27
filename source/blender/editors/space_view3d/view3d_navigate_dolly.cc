/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

#include "BLI_math.h"

#include "BKE_context.h"
#include "BKE_report.h"

#include "DEG_depsgraph.h"

#include "WM_api.h"

#include "RNA_access.h"

#include "ED_screen.h"

#include "view3d_intern.h"
#include "view3d_navigate.hh" /* own include */

/* -------------------------------------------------------------------- */
/** \name View Dolly Operator
 *
 * Like zoom but translates the view offset along the view direction
 * which avoids #RegionView3D.dist approaching zero.
 * \{ */

/* This is an exact copy of #viewzoom_modal_keymap. */
void viewdolly_modal_keymap(wmKeyConfig *keyconf)
{
  static const EnumPropertyItem modal_items[] = {
      {VIEW_MODAL_CANCEL, "CANCEL", 0, "Cancel", ""},
      {VIEW_MODAL_CONFIRM, "CONFIRM", 0, "Confirm", ""},

      {VIEWROT_MODAL_SWITCH_ROTATE, "SWITCH_TO_ROTATE", 0, "Switch to Rotate"},
      {VIEWROT_MODAL_SWITCH_MOVE, "SWITCH_TO_MOVE", 0, "Switch to Move"},

      {0, nullptr, 0, nullptr, nullptr},
  };

  wmKeyMap *keymap = WM_modalkeymap_find(keyconf, "View3D Dolly Modal");

  /* This function is called for each space-type, only needs to add map once. */
  if (keymap && keymap->modal_items) {
    return;
  }

  keymap = WM_modalkeymap_ensure(keyconf, "View3D Dolly Modal", modal_items);

  /* assign map to operators */
  WM_modalkeymap_assign(keymap, "VIEW3D_OT_dolly");
}

static bool viewdolly_offset_lock_check(bContext *C, wmOperator *op)
{
  View3D *v3d = CTX_wm_view3d(C);
  RegionView3D *rv3d = CTX_wm_region_view3d(C);
  if (ED_view3d_offset_lock_check(v3d, rv3d)) {
    BKE_report(op->reports, RPT_WARNING, "Cannot dolly when the view offset is locked");
    return true;
  }
  return false;
}

static void view_dolly_to_vector_3d(ARegion *region,
                                    const float orig_ofs[3],
                                    const float dvec[3],
                                    float dfac)
{
  RegionView3D *rv3d = static_cast<RegionView3D *>(region->regiondata);
  //printf("OX: %f, OY: %f, OZ: %f\n", rv3d->ofs[0], rv3d->ofs[1], rv3d->ofs[2]);
  madd_v3_v3v3fl(rv3d->ofs, orig_ofs, dvec, -dfac);
}

static void viewdolly_apply(ViewOpsData *vod, const int xy[2], const bool delta_invert)
{
  float zfac;
  const int sgn = 1 - 2 * delta_invert;

  /* the factor 1.4 adjusts the input sensitivity */
  if (U.uiflag & USER_ZOOM_HORIZ) {
    zfac = 1.4f * (float)(sgn * (xy[0] - vod->init.event_xy[0])) / vod->region->winrct.xmax * vod->rv3d->dist;
  }
  else {
    zfac = 1.4f * (float)(sgn * (xy[1] - vod->init.event_xy[1])) / vod->region->winrct.ymax * vod->rv3d->dist;
  }

  view_dolly_to_vector_3d(vod->region, vod->init.ofs, vod->init.mousevec, zfac);

  if (RV3D_LOCK_FLAGS(vod->rv3d) & RV3D_BOXVIEW) {
    view3d_boxview_sync(vod->area, vod->region);
  }

  ED_view3d_camera_lock_sync(vod->depsgraph, vod->v3d, vod->rv3d);

  ED_region_tag_redraw(vod->region);
}

static int viewdolly_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  ViewOpsData *vod = static_cast<ViewOpsData *>(op->customdata);
  short event_code = VIEW_PASS;
  bool use_autokey = false;
  int ret = OPERATOR_RUNNING_MODAL;

  /* Execute the events. */
  if (event->type == EVT_MODAL_MAP) {
    switch (event->val) {
      case VIEW_MODAL_CONFIRM:
        event_code = VIEW_CONFIRM;
        break;
      case VIEWROT_MODAL_SWITCH_MOVE:
        WM_operator_name_call(C, "VIEW3D_OT_move", WM_OP_INVOKE_DEFAULT, nullptr, event);
        event_code = VIEW_CONFIRM;
        break;
      case VIEWROT_MODAL_SWITCH_ROTATE:
        WM_operator_name_call(C, "VIEW3D_OT_rotate", WM_OP_INVOKE_DEFAULT, nullptr, event);
        event_code = VIEW_CONFIRM;
        break;
    }
  }
  else {
    if (event->type == MOUSEMOVE) {
      event_code = VIEW_APPLY;
    }
    else if (event->type == vod->init.event_type) {
      if (event->val == KM_RELEASE) {
        event_code = VIEW_CONFIRM;
      }
    }
    else if (event->type == EVT_ESCKEY) {
      if (event->val == KM_PRESS) {
        event_code = VIEW_CANCEL;
      }
    }
  }

  switch (event_code) {
    case VIEW_APPLY: {
      viewdolly_apply(vod, event->xy, (U.uiflag & USER_ZOOM_INVERT) != 0);
      if (ED_screen_animation_playing(CTX_wm_manager(C))) {
        use_autokey = true;
      }
      break;
    }
    case VIEW_CONFIRM: {
      use_autokey = true;
      ret = OPERATOR_FINISHED;
      break;
    }
    case VIEW_CANCEL: {
      viewops_data_state_restore(vod);
      ret = OPERATOR_CANCELLED;
      break;
    }
  }

  if (use_autokey) {
    ED_view3d_camera_lock_autokey(vod->v3d, vod->rv3d, C, false, true);
  }

  if ((ret & OPERATOR_RUNNING_MODAL) == 0) {
    if (ret & OPERATOR_FINISHED) {
      ED_view3d_camera_lock_undo_push(op->type->name, vod->v3d, vod->rv3d, C);
    }
    viewops_data_free(C, vod);
    op->customdata = nullptr;
  }

  return ret;
}

static int viewdolly_exec(bContext *C, wmOperator *op)
{
  View3D *v3d;
  RegionView3D *rv3d;
  ScrArea *area;
  ARegion *region;
  float mousevec[3];

  const int delta = RNA_int_get(op->ptr, "delta");
  const bool use_cursor_init = RNA_boolean_get(op->ptr, "use_cursor_init");

  if (op->customdata) {
    ViewOpsData *vod = static_cast<ViewOpsData *>(op->customdata);

    area = vod->area;
    region = vod->region;
    copy_v3_v3(mousevec, vod->init.mousevec);
  }
  else {
    area = CTX_wm_area(C);
    region = CTX_wm_region(C);
    negate_v3_v3(mousevec, static_cast<RegionView3D *>(region->regiondata)->viewinv[2]);
    normalize_v3(mousevec);
  }

  v3d = static_cast<View3D *>(area->spacedata.first);
  rv3d = static_cast<RegionView3D *>(region->regiondata);

  /* overwrite the mouse vector with the view direction (zoom into the center) */
  if (!use_cursor_init) {
    normalize_v3_v3(mousevec, rv3d->viewinv[2]);
    negate_v3(mousevec);
  }

  view_dolly_to_vector_3d(region, rv3d->ofs, mousevec, delta);

  if (RV3D_LOCK_FLAGS(rv3d) & RV3D_BOXVIEW) {
    view3d_boxview_sync(area, region);
  }

  ED_view3d_camera_lock_sync(CTX_data_ensure_evaluated_depsgraph(C), v3d, rv3d);

  ED_region_tag_redraw(region);

  viewops_data_free(C, static_cast<ViewOpsData *>(op->customdata));
  op->customdata = nullptr;

  return OPERATOR_FINISHED;
}

/* copied from viewzoom_invoke(), changes here may apply there */
static int viewdolly_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  ViewOpsData *vod;

  if (viewdolly_offset_lock_check(C, op)) {
    return OPERATOR_CANCELLED;
  }

  /* true by default, so dolly moves to pointer location */
  const bool use_cursor_init = RNA_boolean_get(op->ptr, "use_cursor_init");
  vod = viewops_data_create(C, event, V3D_OP_MODE_DOLLY, use_cursor_init);
  op->customdata = vod;

  ED_view3d_smooth_view_force_finish(C, vod->v3d, vod->region);

  /* needs to run before 'viewops_data_create' so the backup 'rv3d->ofs' is correct */
  /* switch from camera view when: */
  if (vod->rv3d->persp != RV3D_PERSP) {
    if (vod->rv3d->persp == RV3D_CAMOB) {
      /* ignore rv3d->lpersp because dolly only makes sense in perspective mode */
      const Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
      ED_view3d_persp_switch_from_camera(depsgraph, vod->v3d, vod->rv3d, RV3D_PERSP);
    }
    else {
      vod->rv3d->persp = RV3D_PERSP;
    }
    ED_region_tag_redraw(vod->region);
  }

  /* if one or the other zoom position is not set, set from event */
  if (!RNA_struct_property_is_set(op->ptr, "mx") || !RNA_struct_property_is_set(op->ptr, "my")) {
    RNA_int_set(op->ptr, "mx", event->xy[0]);
    RNA_int_set(op->ptr, "my", event->xy[1]);
  }

  if (RNA_struct_property_is_set(op->ptr, "delta")) {
    return viewdolly_exec(C, op);
  }
  else {
    /* overwrite the mouse vector with the view direction (zoom into the center) */
    if (!use_cursor_init) {
      negate_v3_v3(vod->init.mousevec, vod->rv3d->viewinv[2]);
      normalize_v3(vod->init.mousevec);
    }

    if (ELEM(event->type, MOUSEZOOM, MOUSEPAN)) {
      int prev_xy[2];
      //printf("EX:%d, EY:%d, PEX:%d, PEY:%d\n", event->xy[0], event->xy[1], event->prev_xy[0], event->prev_xy[1]);
      copy_v2_v2_int(prev_xy, event->prev_xy);
      if (event->type == MOUSEZOOM) {
        /* Set y move = x move as MOUSEZOOM uses only x axis to pass magnification value */
        prev_xy[1] = event->xy[1] - (event->xy[0] - event->prev_xy[0]);
      }
      
      viewdolly_apply(vod, prev_xy, ((U.uiflag & USER_ZOOM_INVERT) != 0)
                      ^ ((event->flag & WM_EVENT_SCROLL_INVERT) !=0));
      
      viewops_data_free(C, static_cast<ViewOpsData *>(op->customdata));
      op->customdata = nullptr;
      
      return OPERATOR_FINISHED;
    }

    /* add temp handler */
    WM_event_add_modal_handler(C, op);
    
    return OPERATOR_RUNNING_MODAL;
  }
}

void VIEW3D_OT_dolly(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Dolly View";
  ot->description = "Dolly in/out in the view";
  ot->idname = viewops_operator_idname_get(V3D_OP_MODE_DOLLY);

  /* api callbacks */
  ot->invoke = viewdolly_invoke;
  ot->exec = viewdolly_exec;
  ot->modal = viewdolly_modal;
  ot->poll = view3d_zoom_or_dolly_poll;
  ot->cancel = view3d_navigate_cancel_fn;

  /* flags */
  ot->flag = OPTYPE_BLOCKING | OPTYPE_GRAB_CURSOR_XY;

  /* properties */
  view3d_operator_properties_common(
      ot, V3D_OP_PROP_DELTA | V3D_OP_PROP_MOUSE_CO | V3D_OP_PROP_USE_MOUSE_INIT);
}

/** \} */
