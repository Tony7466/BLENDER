/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgpencil
 */

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "MEM_guardedalloc.h"

#include "BLI_hash.h"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_math_rotation.h"
#include "BLI_rand.h"
#include "BLI_time.h"
#include "BLI_utildefines.h"

#include "BLT_translation.hh"

#include "DNA_brush_types.h"
#include "DNA_gpencil_legacy_types.h"
#include "DNA_material_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_windowmanager_types.h"

#include "BKE_brush.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_deform.hh"
#include "BKE_gpencil_curve_legacy.h"
#include "BKE_gpencil_geom_legacy.h"
#include "BKE_gpencil_legacy.h"
#include "BKE_gpencil_update_cache_legacy.h"
#include "BKE_main.hh"
#include "BKE_material.h"
#include "BKE_paint.hh"
#include "BKE_report.hh"
#include "BKE_screen.hh"

#include "UI_view2d.hh"

#include "ED_gpencil_legacy.hh"
#include "ED_screen.hh"
#include "ED_view3d.hh"

#include "ANIM_keyframing.hh"

#include "GPU_immediate.hh"
#include "GPU_immediate_util.hh"
#include "GPU_state.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_prototypes.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "gpencil_intern.hh"

/* additional OPs */

static int gpencil_guide_rotate_exec(bContext *C, wmOperator *op)
{
  ToolSettings *ts = CTX_data_tool_settings(C);
  GP_Sculpt_Guide *guide = &ts->gp_sculpt.guide;
  float angle = RNA_float_get(op->ptr, "angle");
  bool increment = RNA_boolean_get(op->ptr, "increment");
  if (increment) {
    float oldangle = guide->angle;
    oldangle += angle;
    guide->angle = angle_compat_rad(oldangle, M_PI);
  }
  else {
    guide->angle = angle_compat_rad(angle, M_PI);
  }

  return OPERATOR_FINISHED;
}

void GPENCIL_OT_guide_rotate(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Rotate Guide Angle";
  ot->idname = "GPENCIL_OT_guide_rotate";
  ot->description = "Rotate guide angle";

  /* api callbacks */
  ot->exec = gpencil_guide_rotate_exec;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  PropertyRNA *prop;

  prop = RNA_def_boolean(ot->srna, "increment", true, "Increment", "Increment angle");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
  prop = RNA_def_float(
      ot->srna, "angle", 0.0f, -10000.0f, 10000.0f, "Angle", "Guide angle", -10000.0f, 10000.0f);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}
