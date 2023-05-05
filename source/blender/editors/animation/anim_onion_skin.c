#include "BKE_context.h"
#include "BKE_lib_id.h"
#include "BKE_mesh.h"
#include "DNA_scene_types.h"
#include "DNA_windowmanager_types.h"
#include "ED_onion_skin.h"
#include "MEM_guardedalloc.h"
#include "WM_api.h"
#include "WM_types.h"

static void graveyard()
{
  ListBase selected = {NULL, NULL};
  LISTBASE_FOREACH (CollectionPointerLink *, object_ptr_link, &selected) {
    /* Mesh *mesh = BKE_mesh_from_object(ob);
    if (!mesh) {
      continue;
    } */
    /* Mesh *copy = BKE_mesh_copy_for_eval(mesh); */
    /* Mesh *mesh_result = (Mesh *)BKE_id_copy_ex(
        NULL, &mesh->id, NULL, LIB_ID_CREATE_NO_MAIN | LIB_ID_CREATE_NO_USER_REFCOUNT); */
    /*
    OnionSkinMeshLink *link = MEM_callocN(sizeof(OnionSkinMeshLink), "onion skin mesh link");
    link->mesh = copy;
    BLI_addtail(&scene->onion_skin_cache.meshes, link); */
  }
  BLI_freelistN(&selected);
}

static int onion_skin_add_exec(bContext *C, wmOperator *op)
{
  Scene *scene = CTX_data_scene(C);

  Object *ob = CTX_data_active_object(C);
  if (!ob) {
    return OPERATOR_CANCELLED;
  }
  Mesh *mesh = BKE_mesh_from_object(ob);
  if (!mesh) {
    return OPERATOR_CANCELLED;
  }
  /* Mesh *copy = BKE_mesh_copy_for_eval(mesh); */
  Mesh *copy = (Mesh *)BKE_id_copy_ex(NULL, &mesh->id, NULL, LIB_ID_COPY_LOCALIZE);

  OnionSkinMeshLink *link = MEM_callocN(sizeof(OnionSkinMeshLink), "onion skin mesh link");
  link->mesh = copy;
  BLI_addtail(&scene->onion_skin_cache.meshes, link);

  return OPERATOR_FINISHED;
}

static void ANIM_OT_onion_skin_add(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Add Onion Skin";
  ot->description = "foobar";
  ot->idname = "ANIM_OT_onion_skin_add";

  /* api callbacks */
  ot->exec = onion_skin_add_exec;
  /* ot->cancel = WM_gesture_box_cancel; */

  /* ot->poll = ed_markers_poll_markers_exist; */

  /* flags */
  ot->flag = OPTYPE_UNDO | OPTYPE_REGISTER;

  /* properties */
}

void ED_operatortypes_onion_skin(void)
{
  WM_operatortype_append(ANIM_OT_onion_skin_add);
}
