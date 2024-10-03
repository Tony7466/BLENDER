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

/* ******************************************* */
/* 'Globals' and Defines */

/* values for tGPsdata->status */
enum eGPencil_PaintStatus {
  GP_STATUS_IDLING = 0, /* stroke isn't in progress yet */
  GP_STATUS_PAINTING,   /* a stroke is in progress */
  GP_STATUS_ERROR,      /* something wasn't correctly set up */
  GP_STATUS_DONE,       /* painting done */
};

/* Return flags for adding points to stroke buffer */
enum eGP_StrokeAdd_Result {
  GP_STROKEADD_INVALID = -2,  /* error occurred - insufficient info to do so */
  GP_STROKEADD_OVERFLOW = -1, /* error occurred - cannot fit any more points */
  GP_STROKEADD_NORMAL,        /* point was successfully added */
  GP_STROKEADD_FULL,          /* cannot add any more points to buffer */
};

/* Runtime flags */
enum eGPencil_PaintFlags {
  GP_PAINTFLAG_FIRSTRUN = (1 << 0), /* operator just started */
  GP_PAINTFLAG_SELECTMASK = (1 << 3),
  GP_PAINTFLAG_HARD_ERASER = (1 << 4),
  GP_PAINTFLAG_STROKE_ERASER = (1 << 5),
  GP_PAINTFLAG_REQ_VECTOR = (1 << 6),
};
ENUM_OPERATORS(eGPencil_PaintFlags, GP_PAINTFLAG_REQ_VECTOR)

/* Temporary Guide data */
struct tGPguide {
  /** guide spacing */
  float spacing;
  /** half guide spacing */
  float half_spacing;
  /** origin */
  float origin[2];
  /** rotated point */
  float rot_point[2];
  /** rotated point */
  float rot_angle;
  /** initial stroke direction */
  float stroke_angle;
  /** initial origin direction */
  float origin_angle;
  /** initial origin distance */
  float origin_distance;
  /** initial line for guides */
  float unit[2];
};

/* Temporary 'Stroke' Operation data
 *   "p" = op->customdata
 */
struct tGPsdata {
  bContext *C;

  /** main database pointer. */
  Main *bmain;
  /** current scene from context. */
  Scene *scene;
  Depsgraph *depsgraph;

  /** Current object. */
  Object *ob;
  /** Evaluated object. */
  Object *ob_eval;
  /** window where painting originated. */
  wmWindow *win;
  /** area where painting originated. */
  ScrArea *area;
  /** region where painting originated. */
  ARegion *region;
  /** needed for GP_STROKE_2DSPACE. */
  View2D *v2d;
  /** For operations that require occlusion testing. */
  ViewDepths *depths;
  /** for using the camera rect within the 3d view. */
  rctf *subrect;
  rctf subrect_data;

  /** settings to pass to gp_points_to_xy(). */
  GP_SpaceConversion gsc;

  /** pointer to owner of gp-datablock. */
  PointerRNA ownerPtr;
  /** gp-datablock layer comes from. */
  bGPdata *gpd;
  /** layer we're working on. */
  bGPDlayer *gpl;
  /** frame we're working on. */
  bGPDframe *gpf;

  /** projection-mode flags (toolsettings - eGPencil_Placement_Flags) */
  char *align_flag;

  /** current status of painting. */
  eGPencil_PaintStatus status;
  /** mode for painting. */
  eGPencil_PaintModes paintmode;
  /** flags that can get set during runtime (eGPencil_PaintFlags) */
  eGPencil_PaintFlags flags;

  /** radius of influence for eraser. */
  short radius;

  /** current mouse-position. */
  float mval[2];
  /** previous recorded mouse-position. */
  float mvalo[2];
  /** initial recorded mouse-position */
  float mvali[2];

  /** current stylus pressure. */
  float pressure;
  /** previous stylus pressure. */
  float opressure;

  /* These need to be doubles, as (at least under unix) they are in seconds since epoch,
   * float (and its 7 digits precision) is definitively not enough here!
   * double, with its 15 digits precision,
   * ensures us millisecond precision for a few centuries at least.
   */
  /** Used when converting to path. */
  double inittime;
  /** Used when converting to path. */
  double curtime;
  /** Used when converting to path. */
  double ocurtime;

  /** Inverted transformation matrix applying when converting coords from screen-space
   * to region space. */
  float imat[4][4];
  float mat[4][4];

  float diff_mat[4][4];

  /** custom color - hack for enforcing a particular color for track/mask editing. */
  float custom_color[4];

  /** radial cursor data for drawing eraser. */
  void *erasercursor;

  /* mat settings are only used for 3D view */
  /** current material */
  Material *material;
  /** current drawing brush */
  Brush *brush;
  /** default eraser brush */
  Brush *eraser;

  /** 1: line horizontal, 2: line vertical, other: not defined */
  short straight;
  /** lock drawing to one axis */
  int lock_axis;
  /** the stroke is no fill mode */
  bool disable_fill;

  RNG *rng;

  /** key used for invoking the operator */
  short keymodifier;
  /** shift modifier flag */
  bool shift;
  /** size in pixels for uv calculation */
  float totpixlen;
  /** Special mode for fill brush. */
  bool disable_stabilizer;
  /* guide */
  tGPguide guide;

  ReportList *reports;

  /** Random settings by stroke */
  GpRandomSettings random_settings;
};

/* ------------------------------- */

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
