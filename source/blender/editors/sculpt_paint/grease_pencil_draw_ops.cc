/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#include "BKE_context.h"
#include "BKE_grease_pencil.hh"

#include "DEG_depsgraph_query.h"

#include "DNA_brush_types.h"
#include "DNA_grease_pencil_types.h"

#include "ED_grease_pencil_draw.h"
#include "ED_object.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "WM_api.h"
#include "WM_message.h"
#include "WM_toolsystem.h"

#include "paint_intern.h"

namespace blender::ed::sculpt_paint {


/* -------------------------------------------------------------------- */
/** \name Toggle Draw Mode
 * \{ */

static bool grease_pencil_poll(bContext *C)
{
  Object *object = CTX_data_active_object(C);
  if (object == nullptr || object->type != OB_GREASE_PENCIL) {
    return false;
  }
  return true;
}

static void grease_pencil_draw_mode_enter(bContext *C)
{
  Scene *scene = CTX_data_scene(C);
  wmMsgBus *mbus = CTX_wm_message_bus(C);

  Object *ob = CTX_data_active_object(C);
  BKE_paint_ensure(scene->toolsettings, (Paint **)&scene->toolsettings->gp_paint);
  GpPaint *grease_pencil_paint = scene->toolsettings->gp_paint;

  ob->mode = OB_MODE_PAINT_GPENCIL;

  /* Setup cursor color. BKE_paint_init() could be used, but creates an additional brush. */
  Paint *paint = BKE_paint_get_active_from_paintmode(scene, PAINT_MODE_GPENCIL);
  // copy_v3_v3_uchar(paint->paint_cursor_col, PAINT_CURSOR_GREASE_PENCIL);
  // paint->paint_cursor_col[3] = 128;

  // ED_paint_cursor_start(&grease_pencil_paint->paint, GREASE_PENCIL_mode_poll_view3d);
  paint_init_pivot(ob, scene);

  /* Necessary to change the object mode on the evaluated object. */
  DEG_id_tag_update(&ob->id, ID_RECALC_COPY_ON_WRITE);
  WM_msg_publish_rna_prop(mbus, &ob->id, ob, Object, mode);
  WM_event_add_notifier(C, NC_SCENE | ND_MODE, nullptr);
}

static void grease_pencil_draw_mode_exit(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  ob->mode = OB_MODE_OBJECT;
}

static int grease_pencil_draw_mode_toggle_exec(bContext *C, wmOperator *op)
{
  Object *ob = CTX_data_active_object(C);
  wmMsgBus *mbus = CTX_wm_message_bus(C);

  const bool is_mode_set = ob->mode == OB_MODE_PAINT_GPENCIL;

  if (is_mode_set) {
    if (!ED_object_mode_compat_set(C, ob, OB_MODE_PAINT_GPENCIL, op->reports)) {
      return OPERATOR_CANCELLED;
    }
  }

  if (is_mode_set) {
    grease_pencil_draw_mode_exit(C);
  }
  else {
    grease_pencil_draw_mode_enter(C);
  }

  WM_toolsystem_update_from_context_view3d(C);

  /* Necessary to change the object mode on the evaluated object. */
  DEG_id_tag_update(&ob->id, ID_RECALC_COPY_ON_WRITE);
  WM_msg_publish_rna_prop(mbus, &ob->id, ob, Object, mode);
  WM_event_add_notifier(C, NC_SCENE | ND_MODE, nullptr);
  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_draw_mode_toggle(wmOperatorType *ot)
{
  ot->name = "Grease Pencil Draw Mode Toggle";
  ot->idname = "GREASE_PENCIL_OT_draw_mode_toggle";
  ot->description = "Enter/Exit draw mode for grease pencil";

  ot->exec = grease_pencil_draw_mode_toggle_exec;
  ot->poll = grease_pencil_poll;

  ot->flag = OPTYPE_UNDO | OPTYPE_REGISTER;
}

/** \} */

}  // namespace blender::ed::sculpt_paint

/* -------------------------------------------------------------------- */
/** \name Registration
 * \{ */

void ED_operatortypes_grease_pencil_draw()
{
  using namespace blender::ed::sculpt_paint;
  WM_operatortype_append(GREASE_PENCIL_OT_draw_mode_toggle);
}

/** \} */