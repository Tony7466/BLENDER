/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include "MEM_guardedalloc.h"

#include "BLI_math_matrix.h"
#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"

#include "BKE_context.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_paint.hh"
#include "BKE_report.hh"

#include "ED_sculpt.hh"

#include "transform.hh"
#include "transform_convert.hh"

/* -------------------------------------------------------------------- */
/** \name Sculpt Transform Creation
 * \{ */

static void createTransSculpt(bContext *C, TransInfo *t)
{
  using namespace blender::ed;
  TransData *td;

  Scene *scene = t->scene;
  if (!BKE_id_is_editable(CTX_data_main(C), &scene->id)) {
    BKE_report(t->reports, RPT_ERROR, "Linked data can't text-space transform");
    return;
  }

  BKE_view_layer_synced_ensure(t->scene, t->view_layer);
  Object &ob = *BKE_view_layer_active_object_get(t->view_layer);
  SculptSession &ss = *ob.sculpt;

  /* Avoid editing locked shapes. */
  if (t->mode != TFM_DUMMY && sculpt_paint::report_if_shape_key_is_locked(ob, t->reports)) {
    return;
  }

  {
    BLI_assert(t->data_container_len == 1);
    TransDataContainer *tc = t->data_container;
    tc->data_len = 1;
    tc->is_active = true;
    td = tc->data = MEM_cnew<TransData>(__func__);
    td->ext = tc->data_ext = MEM_cnew<TransDataExtension>(__func__);
  }

  View3DCursor *cursor = &scene->cursor;

  td->flag = TD_SELECTED;
  copy_v3_v3(td->center, cursor->location);
  td->ob = &ob;

  float obmat_inv[3][3];
  copy_m3_m4(obmat_inv, ob.object_to_world().ptr());
  invert_m3(obmat_inv);

  td->loc = cursor->location;
  copy_v3_v3(td->iloc, cursor->location);

  if (cursor->rotation_mode > 0) {
    td->ext->rot = cursor->rotation_euler;
    td->ext->rotAxis = nullptr;
    td->ext->rotAngle = nullptr;
    td->ext->quat = nullptr;

    copy_v3_v3(td->ext->irot, cursor->rotation_euler);
  }
  else if (cursor->rotation_mode == ROT_MODE_AXISANGLE) {
    td->ext->rot = nullptr;
    td->ext->rotAxis = cursor->rotation_axis;
    td->ext->rotAngle = &cursor->rotation_angle;
    td->ext->quat = nullptr;

    td->ext->irotAngle = cursor->rotation_angle;
    copy_v3_v3(td->ext->irotAxis, cursor->rotation_axis);
  }
  else {
    td->ext->rot = nullptr;
    td->ext->rotAxis = nullptr;
    td->ext->rotAngle = nullptr;
    td->ext->quat = cursor->rotation_quaternion;

    copy_qt_qt(td->ext->iquat, cursor->rotation_quaternion);
  }
  td->ext->rotOrder = cursor->rotation_mode;

  copy_m4_m4(td->ext->obmat, ob.object_to_world().ptr());
  copy_m3_m3(td->ext->l_smtx, obmat_inv);
  copy_m3_m4(td->ext->r_mtx, ob.object_to_world().ptr());
  copy_m3_m3(td->ext->r_smtx, obmat_inv);

  ss.pivot_scale[0] = 1.0f;
  ss.pivot_scale[1] = 1.0f;
  ss.pivot_scale[2] = 1.0f;
  td->ext->size = ss.pivot_scale;
  copy_v3_v3(ss.init_pivot_scale, ss.pivot_scale);
  copy_v3_v3(td->ext->isize, ss.init_pivot_scale);

  copy_m3_m3(td->smtx, obmat_inv);
  copy_m3_m4(td->mtx, ob.object_to_world().ptr());
  copy_m3_m4(td->axismtx, ob.object_to_world().ptr());

  BLI_assert(!(t->options & CTX_PAINT_CURVE));
  sculpt_paint::init_transform(C, ob, t->mval, t->undo_name);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Recalc Data object
 * \{ */

static void recalcData_sculpt(TransInfo *t)
{
  using namespace blender::ed;
  BKE_view_layer_synced_ensure(t->scene, t->view_layer);
  Object *ob = BKE_view_layer_active_object_get(t->view_layer);
  sculpt_paint::update_modal_transform(t->context, *ob);
  DEG_id_tag_update(&t->scene->id, ID_RECALC_SYNC_TO_EVAL);
}

static void special_aftertrans_update__sculpt(bContext *C, TransInfo *t)
{
  using namespace blender::ed;
  Scene *scene = t->scene;
  if (!BKE_id_is_editable(CTX_data_main(C), &scene->id)) {
    /* `sculpt_paint::init_transform` was not called in this case. */
    return;
  }

  BKE_view_layer_synced_ensure(t->scene, t->view_layer);
  Object *ob = BKE_view_layer_active_object_get(t->view_layer);
  BLI_assert(!(t->options & CTX_PAINT_CURVE));
  sculpt_paint::end_transform(C, *ob);
}

/** \} */

TransConvertTypeInfo TransConvertType_Sculpt = {
    /*flags*/ 0,
    /*create_trans_data*/ createTransSculpt,
    /*recalc_data*/ recalcData_sculpt,
    /*special_aftertrans_update*/ special_aftertrans_update__sculpt,
};
