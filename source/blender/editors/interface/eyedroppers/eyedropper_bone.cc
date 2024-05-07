/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 *
 * Eyedropper (bones)
 */

#include "BKE_context.hh"
#include "BKE_idtype.hh"
#include "BKE_report.hh"
#include "BKE_screen.hh"

#include "BLT_translation.hh"

#include "BLI_assert.h"
#include "BLI_math_vector.h"
#include "BLI_string.h"

#include "RNA_access.hh"

#include "ED_outliner.hh"
#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "UI_interface.hh"

#include "eyedropper_intern.hh"
#include "interface_intern.hh"

namespace blender::ui {

struct BoneDropper {
  PointerRNA ptr;
  PropertyRNA *prop;
  short idcode;
  const char *idcode_name;
  bool is_undo;

  ScrArea *cursor_area; /* Area under the cursor */
  ARegionType *area_region_type;
  void *draw_handle_pixel;
  int name_pos[2];
  char name[200];
};

static void datadropper_draw_cb(const bContext * /*C*/, ARegion * /*region*/, void *arg)
{
  BoneDropper *ddr = static_cast<BoneDropper *>(arg);
  eyedropper_draw_cursor_text_region(ddr->name_pos, ddr->name);
}

static int bonedropper_init(bContext *C, wmOperator *op)
{
  int index_dummy;
  StructRNA *type;

  SpaceType *space_type;
  ARegionType *area_region_type;

  space_type = BKE_spacetype_from_id(SPACE_VIEW3D);
  area_region_type = BKE_regiontype_from_id(space_type, RGN_TYPE_WINDOW);

  BoneDropper *ddr = MEM_cnew<BoneDropper>(__func__);

  uiBut *button = UI_context_active_but_prop_get(C, &ddr->ptr, &ddr->prop, &index_dummy);

  if ((ddr->ptr.data == nullptr) || (ddr->prop == nullptr) ||
      (RNA_property_editable(&ddr->ptr, ddr->prop) == false) ||
      (RNA_property_type(ddr->prop) != PROP_POINTER))
  {
    MEM_freeN(ddr);
    return false;
  }
  op->customdata = ddr;

  ddr->is_undo = UI_but_flag_is_set(button, UI_BUT_UNDO);

  ddr->cursor_area = CTX_wm_area(C);
  ddr->area_region_type = area_region_type;
  ddr->draw_handle_pixel = ED_region_draw_cb_activate(
      area_region_type, datadropper_draw_cb, ddr, REGION_DRAW_POST_PIXEL);

  type = RNA_property_pointer_type(&ddr->ptr, ddr->prop);
  ddr->idcode = RNA_type_to_ID_code(type);
  BLI_assert(ddr->idcode != 0);
  /* Note we can translate here (instead of on draw time),
   * because this struct has very short lifetime. */
  ddr->idcode_name = TIP_(BKE_idtype_idcode_to_name(ddr->idcode));

  const PointerRNA ptr = RNA_property_pointer_get(&ddr->ptr, ddr->prop);
  // ddr->init_id = ptr.owner_id;

  return true;
}

static void bonedropper_exit(bContext *C, wmOperator *op)
{
  wmWindow *win = CTX_wm_window(C);

  WM_cursor_modal_restore(win);

  if (op->customdata) {
    BoneDropper *ddr = (BoneDropper *)op->customdata;

    if (ddr->area_region_type) {
      ED_region_draw_cb_exit(ddr->area_region_type, ddr->draw_handle_pixel);
    }

    MEM_freeN(op->customdata);

    op->customdata = nullptr;
  }

  WM_event_add_mousemove(win);
}

static void bonedropper_cancel(bContext *C, wmOperator *op)
{
  BoneDropper *ddr = static_cast<BoneDropper *>(op->customdata);
  // bonedropper_id_set(C, ddr, ddr->init_id);
  bonedropper_exit(C, op);
}

/* To switch the draw callback when region under mouse event changes */
static void bonedropper_set_draw_callback_region(ScrArea &area, BoneDropper &bdr)
{
  if (area.spacetype == bdr.cursor_area->spacetype) {
    return;
  }

  /* If the spacetype changed remove the old callback. */
  ED_region_draw_cb_exit(bdr.area_region_type, bdr.draw_handle_pixel);

  /* Redraw old area */
  ARegion *region = BKE_area_find_region_type(bdr.cursor_area, RGN_TYPE_WINDOW);
  ED_region_tag_redraw(region);

  /* Set draw callback in new region */
  ARegionType *art = BKE_regiontype_from_id(area.type, RGN_TYPE_WINDOW);

  bdr.cursor_area = &area;
  bdr.area_region_type = art;
  bdr.draw_handle_pixel = ED_region_draw_cb_activate(
      art, datadropper_draw_cb, &bdr, REGION_DRAW_POST_PIXEL);
}

static void bonedropper_id_sample_pt(
    bContext *C, wmWindow &win, ScrArea &area, BoneDropper &bdr, const int event_xy[2], ID **r_id)
{
  if (!ELEM(area.spacetype, SPACE_VIEW3D, SPACE_OUTLINER)) {
    return;
  }

  ARegion *region = BKE_area_find_region_xy(&area, RGN_TYPE_WINDOW, event_xy);

  if (!region) {
    return;
  }

  wmWindow *win_prev = CTX_wm_window(C);
  ScrArea *area_prev = CTX_wm_area(C);
  ARegion *region_prev = CTX_wm_region(C);

  bdr.name[0] = '\0';

  const int mval[2] = {event_xy[0] - region->winrct.xmin, event_xy[1] - region->winrct.ymin};
  Base *base;

  CTX_wm_window_set(C, &win);
  CTX_wm_area_set(C, &area);
  CTX_wm_region_set(C, region);

  /* Unfortunately it's necessary to always draw else we leave stale text. */
  ED_region_tag_redraw(region);

  switch (area.spacetype) {
    case SPACE_VIEW3D:
      base = ED_view3d_give_base_under_cursor(C, mval);
      break;
    case SPACE_OUTLINER:
      base = ED_outliner_give_base_under_cursor(C, mval);
      break;

    default:
      BLI_assert_unreachable();
      break;
  }

  if (base) {
    Object *ob = base->object;
    ID *id = nullptr;
    if (bdr.idcode == ID_OB) {
      id = (ID *)ob;
    }
    else if (ob->data) {
      if (GS(((ID *)ob->data)->name) == bdr.idcode) {
        id = (ID *)ob->data;
      }
      else {
        SNPRINTF(bdr.name, "Incompatible, expected a %s", bdr.idcode_name);
      }
    }

    PointerRNA idptr = RNA_id_pointer_create(id);

    if (id && RNA_property_pointer_poll(&bdr.ptr, bdr.prop, &idptr)) {
      SNPRINTF(bdr.name, "%s: %s", bdr.idcode_name, id->name + 2);
      *r_id = id;
    }

    copy_v2_v2_int(bdr.name_pos, mval);
  }

  CTX_wm_window_set(C, win_prev);
  CTX_wm_area_set(C, area_prev);
  CTX_wm_region_set(C, region_prev);
}

static bool bonedropper_id_sample(bContext *C, BoneDropper &ddr, const int event_xy[2])
{
  int event_xy_win[2];
  wmWindow *win;
  ScrArea *area;
  eyedropper_win_area_find(C, event_xy, event_xy_win, &win, &area);

  if (!win || !area) {
    return false;
  }

  ID *id = nullptr;
  bonedropper_id_sample_pt(C, *win, *area, ddr, event_xy_win, &id);
  // return bonedropper_id_set(C, ddr, id);
  return true;
}

/* main modal status check */
static int bonedropper_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  BoneDropper *bdr = (BoneDropper *)op->customdata;
  if (!bdr) {
    return OPERATOR_CANCELLED;
  }

  /* handle modal keymap */
  if (event->type == EVT_MODAL_MAP) {
    switch (event->val) {
      case EYE_MODAL_CANCEL:
        bonedropper_cancel(C, op);
        return OPERATOR_CANCELLED;
      case EYE_MODAL_SAMPLE_CONFIRM: {
        const bool is_undo = bdr->is_undo;
        const bool success = bonedropper_id_sample(C, *bdr, event->xy);
        bonedropper_exit(C, op);
        if (success) {
          /* Could support finished & undo-skip. */
          return is_undo ? OPERATOR_FINISHED : OPERATOR_CANCELLED;
        }
        BKE_report(op->reports, RPT_WARNING, "Failed to set value");
        return OPERATOR_CANCELLED;
      }
    }
  }
  else if (event->type == MOUSEMOVE) {
    ID *id = nullptr;

    int event_xy_win[2];
    wmWindow *win;
    ScrArea *area;
    eyedropper_win_area_find(C, event->xy, event_xy_win, &win, &area);

    if (win && area) {
      /* Set the region for eyedropper cursor text drawing */
      bonedropper_set_draw_callback_region(*area, *bdr);
      bonedropper_id_sample_pt(C, *win, *area, *bdr, event_xy_win, &id);
    }
  }

  return OPERATOR_RUNNING_MODAL;
}
static int bonedropper_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  /* init */
  if (bonedropper_init(C, op)) {
    wmWindow *win = CTX_wm_window(C);
    /* Workaround for de-activating the button clearing the cursor, see #76794 */
    UI_context_active_but_clear(C, win, CTX_wm_region(C));
    WM_cursor_modal_set(win, WM_CURSOR_EYEDROPPER);

    /* add temp handler */
    WM_event_add_modal_handler(C, op);

    return OPERATOR_RUNNING_MODAL;
  }
  return OPERATOR_CANCELLED;
}

static int bonedropper_exec(bContext *C, wmOperator *op)
{
  /* init */
  if (bonedropper_init(C, op)) {
    /* cleanup */
    bonedropper_exit(C, op);

    return OPERATOR_FINISHED;
  }
  return OPERATOR_CANCELLED;
}

static bool bonedropper_poll(bContext *C)
{
  PointerRNA ptr;
  PropertyRNA *prop;
  int index_dummy;

  if (CTX_wm_window(C) == nullptr) {
    return false;
  }

  uiBut *but = UI_context_active_but_prop_get(C, &ptr, &prop, &index_dummy);
  if (!but) {
    return false;
  }

  if ((but->type == UI_BTYPE_SEARCH_MENU) && (but->flag & UI_BUT_VALUE_CLEAR)) {
  }
  if (prop && RNA_property_type(prop) == PROP_POINTER) {
    StructRNA *type = RNA_property_pointer_type(&ptr, prop);
    const short idcode = RNA_type_to_ID_code(type);
    if ((idcode == ID_OB) || OB_DATA_SUPPORT_ID(idcode)) {
      return true;
    }
  }

  return false;
}

void UI_OT_eyedropper_bone(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Eyedropper Bone";
  ot->idname = "UI_OT_eyedropper_bone";
  ot->description = "Sample a bone from the 3D View to store in a property";

  /* api callbacks */
  ot->invoke = bonedropper_invoke;
  ot->modal = bonedropper_modal;
  ot->cancel = bonedropper_cancel;
  ot->exec = bonedropper_exec;
  ot->poll = bonedropper_poll;

  /* flags */
  ot->flag = OPTYPE_UNDO | OPTYPE_BLOCKING | OPTYPE_INTERNAL;

  /* properties */
}

}  // namespace blender::ui
