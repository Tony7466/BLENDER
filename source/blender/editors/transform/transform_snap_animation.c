/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include "DNA_anim_types.h"

#include "BLI_math.h"

#include "BKE_context.h"
#include "BKE_nla.h"

#include "ED_markers.h"
#include "ED_screen.h"

#include "transform.h"
#include "transform_snap.h"

/* -------------------------------------------------------------------- */
/** \name Snapping in Anim Editors
 * \{ */

short getAnimEdit_SnapMode(TransInfo *t)
{
  eSnapMode autosnap = SCE_SNAP_TO_FRAME;
  autosnap = t->scene->toolsettings->snap_anim_mode;

  /* TODO: Each editor has its own snapping flag */
  /* if (t->spacetype == SPACE_ACTION) {
    SpaceAction *saction = (SpaceAction *)t->area->spacedata.first;

    if (saction) {
      autosnap = saction->autosnap;
    }
  }
  else if (t->spacetype == SPACE_GRAPH) {
    if ((t->mode == TFM_TRANSLATION) && transform_snap_is_active(t)) {
      return autosnap;
    }
    SpaceGraph *sipo = (SpaceGraph *)t->area->spacedata.first;
    if (sipo) {
      autosnap = sipo->autosnap;
    }
  }
  else if (t->spacetype == SPACE_NLA) {
    SpaceNla *snla = (SpaceNla *)t->area->spacedata.first;

    if (snla) {
      autosnap = snla->autosnap;
    }
  } */

  return autosnap;
}

void snapFrameTransform(TransInfo *t,
                        const eSnapMode autosnap,
                        const float val_initial,
                        const float val_final,
                        float *r_val_final)
{
  float deltax = val_final - val_initial;
  switch (autosnap) {
    case SCE_SNAP_TO_FRAME:
      *r_val_final = floorf(val_final + 0.5f);
      break;
    case SCE_SNAP_TO_MARKERS:
      /* Snap to nearest marker. */
      /* TODO: need some more careful checks for where data comes from. */
      *r_val_final = (float)ED_markers_find_nearest_marker_time(&t->scene->markers, val_final);
      break;
    case SCE_SNAP_TO_SECOND:
    case SCE_SNAP_TO_GRID: {
      const Scene *scene = t->scene;
      const double secf = FPS;
      if (autosnap == SCE_SNAP_TO_SECOND) {
        *r_val_final = floorf((val_final / secf) + 0.5) * secf;
      }
      else {
        deltax = (float)(floor((deltax / secf) + 0.5) * secf);
        *r_val_final = val_initial + deltax;
      }
      break;
    }
    case SCE_SNAP_ABS_GRID: {
      deltax = floorf(deltax + 0.5f);
      *r_val_final = val_initial + deltax;
      break;
    }
    default: {
      *r_val_final = val_initial;
      break;
    }
  }
}

void transform_snap_anim_flush_data(TransInfo *t,
                                    TransData *td,
                                    const eSnapMode autosnap,
                                    float *r_val_final)
{
  BLI_assert(t->scene->toolsettings->snap_flag_anim);

  float val = td->loc[0];
  float ival = td->iloc[0];
  AnimData *adt = !ELEM(t->spacetype, SPACE_NLA, SPACE_SEQ) ? td->extra : NULL;

  /* Convert frame to nla-action time (if needed) */
  if (adt) {
    val = BKE_nla_tweakedit_remap(adt, val, NLATIME_CONVERT_MAP);
    ival = BKE_nla_tweakedit_remap(adt, ival, NLATIME_CONVERT_MAP);
  }

  snapFrameTransform(t, autosnap, ival, val, &val);

  /* Convert frame out of nla-action time. */
  if (adt) {
    val = BKE_nla_tweakedit_remap(adt, val, NLATIME_CONVERT_UNMAP);
  }

  *r_val_final = val;
}

/** \} */
