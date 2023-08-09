/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

#include "BLI_math.h"
#include "BLI_rect.h"

#include "BKE_context.h"
#include "BKE_screen.h"

#include "DEG_depsgraph_query.h"

#include "WM_api.hh"

#include "RNA_access.h"

#include "ED_screen.hh"

#include "PIL_time.h"

#include "view3d_intern.h"
#include "view3d_navigate.hh" /* own include */

/* -------------------------------------------------------------------- */
/** \name View Zoom Operator
 * \{ */

/* #viewdolly_modal_keymap has an exact copy of this, apply fixes to both. */
void viewzoom_modal_keymap(wmKeyConfig *keyconf)
{
  static const EnumPropertyItem modal_items[] = {
      {VIEW_MODAL_CANCEL, "CANCEL", 0, "Cancel", ""},
      {VIEW_MODAL_CONFIRM, "CONFIRM", 0, "Confirm", ""},

      {VIEWROT_MODAL_SWITCH_ROTATE, "SWITCH_TO_ROTATE", 0, "Switch to Rotate"},
      {VIEWROT_MODAL_SWITCH_MOVE, "SWITCH_TO_MOVE", 0, "Switch to Move"},

      {0, nullptr, 0, nullptr, nullptr},
  };

  wmKeyMap *keymap = WM_modalkeymap_find(keyconf, "View3D Zoom Modal");

  /* This function is called for each space-type, only needs to add map once. */
  if (keymap && keymap->modal_items) {
    return;
  }

  keymap = WM_modalkeymap_ensure(keyconf, "View3D Zoom Modal", modal_items);

  /* assign map to operators */
  WM_modalkeymap_assign(keymap, "VIEW3D_OT_zoom");
}

/**
 * \param zoom_xy: Optionally zoom to window location
 * (coords compatible w/ #wmEvent.mval).
 */
static void view_zoom_to_vector_3d(const View3D *v3d,
                                      ARegion *region,
                                      const float orig_ofs[3],
                                      const float orig_dist,
                                      const float orig_mval[2],
                                      float delta)
{
  RegionView3D *rv3d = static_cast<RegionView3D *>(region->regiondata);
  float dist_range[2];
  float ctr_xy[2];
  float v[3];
  
  ED_view3d_dist_range_get(v3d, dist_range);
  rv3d->dist = orig_dist - delta;
  CLAMP(rv3d->dist, dist_range[0], dist_range[1]);
  
  ctr_xy[0] = orig_mval[0] - region->winx / 2;
  ctr_xy[1] = orig_mval[1] - region->winy / 2;
  
  if (rv3d->persp != RV3D_ORTHO) {
    ED_view3d_win_to_delta(region, ctr_xy, -delta, v);
  }
  else {
    ED_view3d_win_to_delta(region, ctr_xy, -delta/rv3d->dist, v);
  }
  add_v3_v3v3(rv3d->ofs, orig_ofs, v);
}

static void viewzoom_apply(ViewOpsData *vod, const int move_xy[2],
                           const eViewZoom_Style zoomstyle, const bool delta_invert, const int zoomctr_xy[2])
{
  const int sgn = 1 - 2 * delta_invert;
  
  if ((vod->rv3d->persp == RV3D_CAMOB) && !ED_view3d_camera_lock_check(vod->v3d, vod->rv3d)) {
    float scale;
    
    if (U.uiflag & USER_ZOOM_HORIZ) {
      scale = 1.0f + float(sgn * move_xy[0]) / UI_SCALE_FAC / 300.0f;
    }
    else {
      scale = 1.0f + float(sgn * move_xy[1]) / UI_SCALE_FAC / 300.0f;
    }
    
    ED_view3d_camera_view_zoom_scale(vod->rv3d, scale);
  }
  else {
    float delta;
    
    if (zoomstyle == USER_ZOOM_CONTINUE) {
      double time = PIL_check_seconds_timer();
      float move_t = float(time - vod->prev.time);
      vod->prev.time = time;
      
      if (U.uiflag & USER_ZOOM_HORIZ) {
        delta = float(sgn * move_xy[0]) / UI_SCALE_FAC / 130.0f;
      }
      else {
        delta = float(sgn * move_xy[1]) / UI_SCALE_FAC / 130.0f;
      }
      delta *= move_t * vod->init.dist;
      delta = vod->init.zfac += delta;
    }
    else if (zoomstyle == USER_ZOOM_SCALE) {
      /* method which zooms based on how far you move the mouse */
      float delta_xy[2];
      float radii[2];
      int ctr_xy[2];
      
      ctr_xy[0] = BLI_rcti_cent_x(&vod->region->winrct);
      ctr_xy[1] = BLI_rcti_cent_y(&vod->region->winrct);
      
      /* initial radius */
      radii[0] = max_ff(len_v2v2_int(vod->init.event_xy, ctr_xy), 2.0f);
      
      delta_xy[0] = float(vod->init.event_xy[0] - ctr_xy[0] + move_xy[0]);
      delta_xy[1] = float(vod->init.event_xy[1] - ctr_xy[1] + move_xy[1]);
      /* current radius */
      radii[1] = max_ff(len_v2(delta_xy), 2.0f);
      
      delta = (radii[1] / radii[0] - 1.0f) * vod->init.dist * 2.0f;
    }
    else { /* USER_ZOOM_DOLLY */
      if (U.uiflag & USER_ZOOM_HORIZ) {
        delta = float(sgn * move_xy[0]) / UI_SCALE_FAC * vod->init.dist / 130.0f;
      }
      else {
        delta = float(sgn * move_xy[1]) / UI_SCALE_FAC * vod->init.dist / 130.0f;
      }
    }
    
    {
      const float zoomctr_f_xy[2] = {float(zoomctr_xy[0]), float(zoomctr_xy[1])};
      view_zoom_to_vector_3d(vod->v3d, vod->region, vod->init.ofs, vod->init.dist, zoomctr_f_xy, delta);
    }
    
    if (RV3D_LOCK_FLAGS(vod->rv3d) & RV3D_BOXVIEW) {
      view3d_boxview_sync(vod->area, vod->region);
    }
  }
  
  ED_view3d_camera_lock_sync(vod->depsgraph, vod->v3d, vod->rv3d);

  ED_region_tag_redraw(vod->region);
}

static int viewzoom_modal_impl(bContext *C,
                               ViewOpsData *vod,
                               const eV3D_OpEvent event_code,
                               const int xy[2])
{
  bool use_autokey = false;
  int ret = OPERATOR_RUNNING_MODAL;
  int move_xy[2];
  int zoomctr_xy[2];

  switch (event_code) {
    case VIEW_APPLY: {
      if ((vod->rv3d->persp == RV3D_CAMOB) && !ED_view3d_camera_lock_check(vod->v3d, vod->rv3d)) {
        sub_v2_v2v2_int(move_xy, xy, vod->prev.event_xy);
        copy_v2_v2_int(vod->prev.event_xy, xy);
      }
      else {
        sub_v2_v2v2_int(move_xy, xy, vod->init.event_xy);
        copy_v2_v2_int(zoomctr_xy, vod->init.event_xy_offset);
      }
      viewzoom_apply(vod, move_xy, (eViewZoom_Style)U.viewzoom,
                     (U.uiflag & USER_ZOOM_INVERT) != 0, zoomctr_xy);
      
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
      vod->state_restore();
      ret = OPERATOR_CANCELLED;
      break;
    }
    case VIEW_PASS:
      break;
  }

  if (use_autokey) {
    ED_view3d_camera_lock_autokey(vod->v3d, vod->rv3d, C, false, true);
  }

  return ret;
}

static int viewzoom_apply_step(bContext *C, PointerRNA *ptr, ViewOpsData *vod, const int zoomctr_xy[2])
{
  View3D *v3d;
  RegionView3D *rv3d;
  ScrArea *area;
  ARegion *region;

  const int delta = RNA_int_get(ptr, "delta");

  /* execution always based on input device event */
  area = vod->area;
  region = vod->region;
  v3d = static_cast<View3D *>(area->spacedata.first);
  rv3d = static_cast<RegionView3D *>(region->regiondata);
  
  if ((rv3d->persp == RV3D_CAMOB) && !ED_view3d_camera_lock_check(v3d, rv3d)) {
    const float scale = 1.0f + 0.1f * delta;
    ED_view3d_camera_view_zoom_scale(rv3d, scale);
  }
  else {
    const float zoomctr_f_xy[2] = {float(zoomctr_xy[0]), float(zoomctr_xy[1])};
    view_zoom_to_vector_3d(v3d, region, rv3d->ofs, rv3d->dist, zoomctr_f_xy, delta);
    
    if (RV3D_LOCK_FLAGS(rv3d) & RV3D_BOXVIEW) {
      view3d_boxview_sync(area, region);
    }
  }
  
  ED_view3d_camera_lock_sync(CTX_data_ensure_evaluated_depsgraph(C), v3d, rv3d);
  ED_view3d_camera_lock_autokey(v3d, rv3d, C, false, true);

  ED_region_tag_redraw(region);

  return OPERATOR_FINISHED;
}

static int viewzoom_exec(bContext *C, wmOperator *op)
{
  View3D *v3d;
  RegionView3D *rv3d;
  ScrArea *area;
  ARegion *region;

  const int delta = RNA_int_get(op->ptr, "delta");

  area = CTX_wm_area(C);
  region = CTX_wm_region(C);
  float zoomctr_f_xy[2];
    
  const bool use_cursor_init = RNA_boolean_get(op->ptr, "use_cursor_init");
  if (use_cursor_init) {
    zoomctr_f_xy[0] = float(RNA_int_get(op->ptr, "mx"));
    zoomctr_f_xy[1] = float(RNA_int_get(op->ptr, "my"));
  }
  else {
    zoomctr_f_xy[0] = float(region->winx / 2);
    zoomctr_f_xy[1] = float(region->winy / 2);
  }

  v3d = static_cast<View3D *>(area->spacedata.first);
  rv3d = static_cast<RegionView3D *>(region->regiondata);

  view_zoom_to_vector_3d(v3d, region, rv3d->ofs, rv3d->dist, zoomctr_f_xy, delta);

  if (RV3D_LOCK_FLAGS(rv3d) & RV3D_BOXVIEW) {
    view3d_boxview_sync(area, region);
  }

  ED_view3d_camera_lock_sync(CTX_data_ensure_evaluated_depsgraph(C), v3d, rv3d);
  ED_view3d_camera_lock_undo_grouped_push(op->type->name, v3d, rv3d, C);

  ED_region_tag_redraw(region);

  return OPERATOR_FINISHED;
}

static int viewzoom_invoke_impl(bContext *C,
                                ViewOpsData *vod,
                                const wmEvent *event,
                                PointerRNA *ptr)
{
  int zoomctr_xy[2];
  
  /* use_cursor_init is true by default */
  const bool use_cursor_init = RNA_boolean_get(ptr, "use_cursor_init");
  
  if (!use_cursor_init) {
    RNA_int_set(ptr, "mx", zoomctr_xy[0] = vod->region->winx / 2);
    RNA_int_set(ptr, "my", zoomctr_xy[1] = vod->region->winy / 2);
  }
  else {
    /* if one or the other zoom direction is not set */
    if (!RNA_struct_property_is_set(ptr, "mx") || !RNA_struct_property_is_set(ptr, "my")) {
      if (U.uiflag & USER_ZOOM_TO_MOUSEPOS) {
        copy_v2_v2_int(zoomctr_xy, event->mval);
      }
      else {
        zoomctr_xy[0] = vod->region->winx / 2;
        zoomctr_xy[1] = vod->region->winy / 2;
      }
      RNA_int_set(ptr, "mx", zoomctr_xy[0]);
      RNA_int_set(ptr, "my", zoomctr_xy[1]);
    }
    else {
      zoomctr_xy[0] = RNA_int_get(ptr, "mx");
      zoomctr_xy[1] = RNA_int_get(ptr, "my");
    }
  }
  
  if (RNA_struct_property_is_set(ptr, "delta")) {
    return viewzoom_apply_step(C, ptr, vod, zoomctr_xy);
  }
  else {
    /* do not set the delta property, as an integer it would be 0 here */
    if (ELEM(event->type, MOUSEZOOM, MOUSEPAN)) {
      int move_xy[2];
      bool zoominv;
      
      if (event->type == MOUSEPAN) {
        move_xy[0] = WM_event_absolute_delta_x(event);
        move_xy[1] = WM_event_absolute_delta_y(event);
        
        zoominv = (U.uiflag & USER_ZOOM_INVERT) != 0;
      }
      else { /* MOUSEZOOM */
        /* Set y move = x move as MOUSEZOOM uses only x axis to pass magnification value */
        move_xy[1] = move_xy[0] = WM_event_absolute_delta_x(event);
        zoominv = false;
      }
      
      viewzoom_apply(vod, move_xy, USER_ZOOM_DOLLY, zoominv, zoomctr_xy);
      ED_view3d_camera_lock_autokey(vod->v3d, vod->rv3d, C, false, true);
      
      return OPERATOR_FINISHED;
    }
  }

  if (U.viewzoom == USER_ZOOM_CONTINUE) {
    /* needs a timer to continue redrawing */
    vod->prev.time = PIL_check_seconds_timer();
    vod->timer = WM_event_timer_add(CTX_wm_manager(C), CTX_wm_window(C), TIMER, 0.01f);
  }
  
  vod->init.event_xy_offset[0] = zoomctr_xy[0];
  vod->init.event_xy_offset[1] = zoomctr_xy[1];
  vod->init.zfac = 0.0f;
  copy_v2_v2_int(vod->prev.event_xy, event->xy);

  return OPERATOR_RUNNING_MODAL;
}

/* viewdolly_invoke() copied this function, changes here may apply there */
static int viewzoom_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  return view3d_navigate_invoke_impl(C, op, event, &ViewOpsType_zoom);
}

void VIEW3D_OT_zoom(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Zoom View";
  ot->description = "Zoom in/out in the view";
  ot->idname = ViewOpsType_zoom.idname;

  /* api callbacks */
  ot->invoke = viewzoom_invoke;
  ot->exec = viewzoom_exec;
  ot->modal = view3d_navigate_modal_fn;
  ot->poll = view3d_zoom_or_dolly_poll;
  ot->cancel = view3d_navigate_cancel_fn;

  /* flags */
  ot->flag = OPTYPE_BLOCKING | OPTYPE_GRAB_CURSOR_XY;

  /* properties */
  view3d_operator_properties_common(
      ot, V3D_OP_PROP_DELTA | V3D_OP_PROP_MOUSE_CO | V3D_OP_PROP_USE_MOUSE_INIT);
}

/** \} */

const ViewOpsType ViewOpsType_zoom = {
    /*flag*/ (VIEWOPS_FLAG_DEPTH_NAVIGATE | VIEWOPS_FLAG_ZOOM_TO_MOUSE),
    /*idname*/ "VIEW3D_OT_zoom",
    /*poll_fn*/ view3d_zoom_or_dolly_poll,
    /*init_fn*/ viewzoom_invoke_impl,
    /*apply_fn*/ viewzoom_modal_impl,
};
