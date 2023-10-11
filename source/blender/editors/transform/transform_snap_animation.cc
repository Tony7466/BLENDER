/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include "DNA_anim_types.h"

#include "BLI_math_matrix_types.hh"

#include "BKE_context.h"
#include "BKE_nla.h"

#include "ED_markers.hh"
#include "ED_screen.hh"

#include "transform.hh"
#include "transform_snap.hh"

using namespace blender;

/* -------------------------------------------------------------------- */
/** \name Snapping in Anim Editors
 * \{ */

void snapFrameTransform(TransInfo *t,
                        const eSnapMode snap_mode,
                        const float val_initial,
                        const float val_final,
                        float *r_val_final)
{
  float deltax = val_final - val_initial;
  /* This is needed for the FPS macro. */
  const Scene *scene = t->scene;
  const eSnapFlag snap_flag = t->tsnap.flag;

  switch (snap_mode) {
    case SCE_SNAP_TO_FRAME: {
      if (snap_flag & SCE_SNAP_ABS_TIME_STEP) {
        *r_val_final = floorf(val_final + 0.5f);
      }
      else {
        deltax = floorf(deltax + 0.5f);
        *r_val_final = val_initial + deltax;
      }
      break;
    }
    case SCE_SNAP_TO_SECOND: {
      if (snap_flag & SCE_SNAP_ABS_TIME_STEP) {
        *r_val_final = floorf((val_final / FPS) + 0.5) * FPS;
      }
      else {
        deltax = float(floor((deltax / FPS) + 0.5) * FPS);
        *r_val_final = val_initial + deltax;
      }
      break;
    }
    case SCE_SNAP_TO_MARKERS: {
      /* Snap to nearest marker. */
      /* TODO: need some more careful checks for where data comes from. */
      const float nearest_marker_time = float(
          ED_markers_find_nearest_marker_time(&t->scene->markers, val_final));
      *r_val_final = nearest_marker_time;
      break;
    }
    default: {
      *r_val_final = val_final;
      break;
    }
  }
}

static void transform_snap_anim_flush_data_ex(
    TransInfo *t, TransData *td, float val, const eSnapMode snap_mode, float *r_val_final)
{
  BLI_assert(t->tsnap.flag);

  float ival = td->iloc[0];
  AnimData *adt = static_cast<AnimData *>(!ELEM(t->spacetype, SPACE_NLA, SPACE_SEQ) ? td->extra :
                                                                                      nullptr);

  /* Convert frame to nla-action time (if needed) */
  if (adt) {
    val = BKE_nla_tweakedit_remap(adt, val, NLATIME_CONVERT_MAP);
    ival = BKE_nla_tweakedit_remap(adt, ival, NLATIME_CONVERT_MAP);
  }

  snapFrameTransform(t, snap_mode, ival, val, &val);

  /* Convert frame out of nla-action time. */
  if (adt) {
    val = BKE_nla_tweakedit_remap(adt, val, NLATIME_CONVERT_UNMAP);
  }

  *r_val_final = val;
}

void transform_snap_anim_flush_data(TransInfo *t,
                                    TransData *td,
                                    const eSnapMode snap_mode,
                                    float *r_val_final)
{
  transform_snap_anim_flush_data_ex(t, td, td->loc[0], snap_mode, r_val_final);
}

static void invert_snap(eSnapMode &snap_mode)
{
  if (snap_mode & SCE_SNAP_TO_FRAME) {
    snap_mode &= ~SCE_SNAP_TO_FRAME;
    snap_mode |= SCE_SNAP_TO_SECOND;
  }
  else if (snap_mode & SCE_SNAP_TO_SECOND) {
    snap_mode &= ~SCE_SNAP_TO_SECOND;
    snap_mode |= SCE_SNAP_TO_FRAME;
  }
}

/* WORKAROUND: The source position is based on the transformed elements.
 * However, at this stage, the transformation has not yet been applied.
 * So apply the transformation here. */
static float nla_transform_apply(TransInfo *t, float *vec, float ival)
{
  float4x4 mat = float4x4::identity();

  float values_final_prev[4];
  size_t values_final_size = sizeof(*t->values_final) * size_t(t->idx_max + 1);
  memcpy(values_final_prev, t->values_final, values_final_size);
  memcpy(t->values_final, vec, values_final_size);

  mat[3][0] = ival;
  transform_apply_matrix(t, mat.ptr());

  memcpy(t->values_final, values_final_prev, values_final_size);

  return mat.location()[0];
}

bool transform_snap_nla_calc(TransInfo *t, float *vec)
{
  TransDataContainer *tc = TRANS_DATA_CONTAINER_FIRST_SINGLE(t);

  eSnapMode snap_mode = t->tsnap.mode;
  if (t->modifiers & MOD_SNAP_INVERT) {
    invert_snap(snap_mode);
  }

  float best_dist = FLT_MAX, best_target = 0.0f, best_source = 0.0f;

  for (int i = 0; i < tc->data_len; i++) {
    TransData *td = &tc->data[i];
    float snap_source = td->iloc[0];
    float snap_target;
    float source_transformed = nla_transform_apply(t, vec, snap_source);

    transform_snap_anim_flush_data_ex(t, td, source_transformed, snap_mode, &snap_target);
    int dist = abs(snap_target - snap_source);
    if (dist != 0.0f && dist < best_dist) {
      best_dist = dist;
      best_target = snap_target;
      best_source = snap_source;
    }
  }

  t->tsnap.snap_target[0] = best_target;
  t->tsnap.snap_source[0] = best_source;
  return true;
}

/** \} */
