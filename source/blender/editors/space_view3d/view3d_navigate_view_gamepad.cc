/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"

#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"

#include "WM_api.hh"

#include "ED_screen.hh"

#include "BLI_math_basis_types.hh"
#include "BLI_math_vector_types.hh"


#include "view3d_intern.h"
#include "view3d_navigate.hh"

#ifdef WITH_INPUT_GAMEPAD

void view3d_gamepad_fly(const wmGamepadAxisData &gamepad,
                        View3D * /*v3d*/,
                        RegionView3D *rv3d,
                        const bool /*use_precision*/,
                        const short /*protect_flag*/,
                        bool &r_has_translate,
                        bool &r_has_rotate)
{
  using float4 = blender::float4;
  using float3 = blender::float3;
  using float2 = blender::float2;

  float3 translation_vector{gamepad.left_thumb.value[0], 0.0, -gamepad.left_thumb.value[1]};
  blender::float4 view_inv;

  invert_qt_qt_normalized(view_inv, rv3d->viewquat);

  bool has_translation = translation_vector;
  if (has_translation) {
    const float speed = 50.0;
    translation_vector *= speed * gamepad.dt;
    mul_qt_v3(view_inv, translation_vector);
    sub_v3_v3(rv3d->ofs, translation_vector);
  }

  float3 rotation_vector{-gamepad.right_thumb.value[1],
                         gamepad.right_thumb.value[0],
                         -gamepad.left_trigger.value + gamepad.right_trigger.value};

  bool has_rotation = rotation_vector;
  if (has_rotation) {
    float4 rotation{};
    const float3 rotation_speed{1.5, 2.0, 2.0};
    rotation_vector *= rotation_speed * gamepad.dt;
    mul_qt_v3(view_inv, rotation_vector);

    eul_to_quat(rotation, rotation_vector);
    mul_qt_qtqt(rv3d->viewquat, rv3d->viewquat, rotation);
  }
  r_has_translate = has_translation;
  r_has_rotate = has_rotation;
}

static int gamepad_all_invoke_impl(bContext * /*C*/,
                                   ViewOpsData * /*vod*/,
                                   const wmEvent * /*event*/,
                                   PointerRNA * /*ptr*/)
{

  return OPERATOR_FINISHED;
}
static int gamepad_all_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  if (ELEM(event->type,
           GAMEPAD_LEFT_THUMB,
           GAMEPAD_RIGHT_THUMB,
           GAMEPAD_LEFT_TRIGGER,
           GAMEPAD_RIGHT_TRIGGER))
  {
    return OPERATOR_CANCELLED;
  }

  return view3d_navigate_invoke_impl(C, op, event, &ViewOpsType_gamepad_all);
}

void VIEW3D_OT_gamepad_all(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "NDOF Transform View";
  ot->description = "Pan and rotate the view with the 3D mouse";
  ot->idname = ViewOpsType_gamepad_all.idname;

  /* api callbacks */
  ot->invoke = gamepad_all_invoke;
  ot->poll = ED_operator_view3d_active;

  /* flags */
  ot->flag = 0;
}

const ViewOpsType ViewOpsType_gamepad_all = {
    /*flag*/ VIEWOPS_FLAG_ORBIT_SELECT,
    /*idname*/ "VIEW3D_OT_gamepad_all",
    /*poll_fn*/ nullptr,
    /*init_fn*/ gamepad_all_invoke_impl,
    /*apply_fn*/ nullptr,
};

#endif /* WITH_INPUT_GAMEPAD */
