#include "BKE_context.h"
#include "DNA_scene_types.h"
#include "DNA_windowmanager_types.h"

#include "ED_onion_skin.h"
#include "MEM_guardedalloc.h"
#include "WM_types.h"

static void add_exec(bContext *C, wmOperator *op)
{
  Object *ob = CTX_data_active_object(C);
  ListBase selected = {NULL, NULL};
  CTX_data_selected_objects(C, &selected);
  Scene *scene = CTX_data_scene(C);
  
  BLI_freelistN(&selected);
}

static void ANIM_OT_onion_skin_add(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Add Onion Skin";
  ot->description = "foobar";
  ot->idname = "ANIM_OT_onion_skin_add";

  /* api callbacks */
  ot->exec = add_exec;
  /* ot->cancel = WM_gesture_box_cancel; */

  /* ot->poll = ed_markers_poll_markers_exist; */

  /* flags */
  ot->flag = OPTYPE_UNDO | OPTYPE_REGISTER;

  /* properties */
}

ED_operatortypes_onion_skin(void)
{
  WM_operatortype_append(ANIM_OT_onion_skin_add);
}
