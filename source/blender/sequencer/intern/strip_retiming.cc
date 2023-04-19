/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. */

/** \file
 * \ingroup bke
 */

#include "MEM_guardedalloc.h"

#include "DNA_scene_types.h"
#include "DNA_sequence_types.h"

#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_span.hh"

#include "BKE_fcurve.h"
#include "BKE_movieclip.h"
#include "BKE_scene.h"
#include "BKE_sound.h"

#include "DNA_anim_types.h"
#include "DNA_sound_types.h"

#include "IMB_imbuf.h"

#include "RNA_prototypes.h"

#include "SEQ_channels.h"
#include "SEQ_iterator.h"
#include "SEQ_relations.h"
#include "SEQ_render.h"
#include "SEQ_retiming.h"
#include "SEQ_retiming.hh"
#include "SEQ_sequencer.h"
#include "SEQ_time.h"
#include "SEQ_transform.h"

#include "sequencer.h"
#include "strip_time.h"
#include "utils.h"

using blender::MutableSpan;

MutableSpan<SeqRetimingHandle> SEQ_retiming_handles_get(const Sequence *seq)
{
  blender::MutableSpan<SeqRetimingHandle> handles(seq->retiming_handles, seq->retiming_handle_num);
  return handles;
}

struct SeqRetimingHandle *SEQ_retiming_last_handle_get(const struct Sequence *seq)
{
  return seq->retiming_handles + seq->retiming_handle_num - 1;
}

int SEQ_retiming_handle_index_get(const Sequence *seq, const SeqRetimingHandle *handle)
{
  return handle - seq->retiming_handles;
}

static bool seq_retiming_is_last_handle(const Sequence *seq, const SeqRetimingHandle *handle)
{
  return SEQ_retiming_handle_index_get(seq, handle) == seq->retiming_handle_num - 1;
}

static const SeqRetimingHandle *retiming_find_segment_start_handle(const Sequence *seq,
                                                                   const int frame_index)
{
  const SeqRetimingHandle *start_handle = nullptr;
  for (auto const &handle : SEQ_retiming_handles_get(seq)) {
    if (seq_retiming_is_last_handle(seq, &handle)) {
      break;
    }
    if (handle.strip_frame_index > frame_index) {
      break;
    }

    start_handle = &handle;
  }

  return start_handle;
}

int SEQ_retiming_handles_count(const Sequence *seq)
{
  return seq->retiming_handle_num;
}

void SEQ_retiming_data_ensure(Sequence *seq)
{
  if (!SEQ_retiming_is_allowed(seq)) {
    return;
  }

  if (seq->retiming_handles != nullptr) {
    return;
  }

  seq->retiming_handles = (SeqRetimingHandle *)MEM_calloc_arrayN(
      2, sizeof(SeqRetimingHandle), __func__);
  SeqRetimingHandle *handle = seq->retiming_handles + 1;
  handle->strip_frame_index = seq->len;
  handle->retiming_factor = 1.0f;
  seq->retiming_handle_num = 2;
}

void SEQ_retiming_data_clear(Sequence *seq)
{
  seq->retiming_handles = nullptr;
  seq->retiming_handle_num = 0;
}

bool SEQ_retiming_is_active(const Sequence *seq)
{
  return seq->retiming_handle_num > 1;
}

bool SEQ_retiming_is_allowed(const Sequence *seq)
{
  return ELEM(seq->type,
              SEQ_TYPE_IMAGE,
              SEQ_TYPE_META,
              SEQ_TYPE_SCENE,
              SEQ_TYPE_MOVIE,
              SEQ_TYPE_MOVIECLIP,
              SEQ_TYPE_MASK);
}

static int seq_retiming_segment_length_get(const SeqRetimingHandle *start_handle)
{
  const SeqRetimingHandle *end_handle = start_handle + 1;
  return end_handle->strip_frame_index - start_handle->strip_frame_index;
}

static float seq_retiming_segment_step_get(const SeqRetimingHandle *start_handle)
{
  const SeqRetimingHandle *end_handle = start_handle + 1;
  const int segment_length = seq_retiming_segment_length_get(start_handle);
  const float segment_fac_diff = end_handle->retiming_factor - start_handle->retiming_factor;
  return segment_fac_diff / segment_length;
}

static void seq_retiming_segment_as_line_segment(const SeqRetimingHandle *start_handle,
                                                 double r_v1[2],
                                                 double r_v2[2])
{
  const SeqRetimingHandle *end_handle = start_handle + 1;
  r_v1[0] = start_handle->strip_frame_index;
  r_v1[1] = start_handle->retiming_factor;
  r_v2[0] = end_handle->strip_frame_index;
  r_v2[1] = end_handle->retiming_factor;
}

static void seq_retiming_line_segments_tangent_circle(const SeqRetimingHandle *start_handle,
                                                      double r_center[2],
                                                      double *radius)
{
  double s1_1[2], s1_2[2], s2_1[2], s2_2[2], p1_2[2];

  /* Get 2 segments. */
  seq_retiming_segment_as_line_segment(start_handle - 1, s1_1, s1_2);
  seq_retiming_segment_as_line_segment(start_handle + 1, s2_1, s2_2);
  /* Backup first segment end point - needed to calculate arc radius. */
  copy_v2_v2_db(p1_2, s1_2);
  /* Convert segments to vectors. */
  double v1[2], v2[2];
  sub_v2_v2v2_db(v1, s1_1, s1_2);
  sub_v2_v2v2_db(v2, s2_1, s2_2);
  /* Rotate segments by 90 degrees around seg. 1 end and start seg. 2 start point. */
  SWAP(float, v1[0], v1[1]);
  SWAP(float, v2[0], v2[1]);
  v1[0] *= -1;
  v2[0] *= -1;
  copy_v2_v2_db(s1_1, s1_2);
  add_v2_v2_db(s1_2, v1);
  copy_v2_v2_db(s2_2, s2_1);
  add_v2_v2_db(s2_2, v2);
  /* Get center and radius of arc segment between 2 linear segments. */
  double lambda, mu;
  int res = isect_seg_seg_v2_lambda_mu_db(
      s1_1, s1_2, s2_1, s2_2, &lambda, &mu);  // XXX may be colinear!
  r_center[0] = s1_1[0] + lambda * (s1_2[0] - s1_1[0]);
  r_center[1] = s1_1[1] + lambda * (s1_2[1] - s1_1[1]);
  *radius = len_v2v2_db(p1_2, r_center);
}

static void seq_retiming_evalualte_eased_segment()
{
  //
}

float seq_retiming_evaluate(const Sequence *seq, const int frame_index)
{
  const SeqRetimingHandle *start_handle = retiming_find_segment_start_handle(seq, frame_index);

  const int start_handle_index = start_handle - seq->retiming_handles;
  BLI_assert(start_handle_index < seq->retiming_handle_num);

  const int segment_frame_index = frame_index - start_handle->strip_frame_index;

  if (start_handle->flag != 1) {
    const float segment_step = seq_retiming_segment_step_get(start_handle);
    return start_handle->retiming_factor + segment_step * segment_frame_index;
  }

  /* Gradual speed change. */
  BLI_assert(start_handle_index > 0);
  BLI_assert(start_handle_index < seq->retiming_handle_num - 1);
  UNUSED_VARS_NDEBUG(start_handle_index);

  double c[2], r;
  seq_retiming_line_segments_tangent_circle(start_handle, c, &r);

  const int side = c[1] > start_handle->retiming_factor ? -1 : 1;
  const float y = c[1] + side * sqrt(pow(r, 2) - pow((frame_index - c[0]), 2));
  return y;
}

SeqRetimingHandle *SEQ_retiming_add_handle(const Scene *scene,
                                           Sequence *seq,
                                           const int timeline_frame)
{
  float frame_index = (timeline_frame - SEQ_time_start_frame_get(seq)) *
                      seq_time_media_playback_rate_factor_get(scene, seq);
  float value = seq_retiming_evaluate(seq, frame_index);

  const SeqRetimingHandle *closest_handle = retiming_find_segment_start_handle(seq, frame_index);
  if (closest_handle->strip_frame_index == frame_index) {
    return nullptr; /* Retiming handle already exists. */
  }

  SeqRetimingHandle *handles = seq->retiming_handles;
  size_t handle_count = SEQ_retiming_handles_count(seq);
  const int new_handle_index = closest_handle - handles + 1;
  BLI_assert(new_handle_index >= 0);
  BLI_assert(new_handle_index < handle_count);

  SeqRetimingHandle *new_handles = (SeqRetimingHandle *)MEM_callocN(
      (handle_count + 1) * sizeof(SeqRetimingHandle), __func__);
  if (new_handle_index > 0) {
    memcpy(new_handles, handles, new_handle_index * sizeof(SeqRetimingHandle));
  }
  if (new_handle_index < handle_count) {
    memcpy(new_handles + new_handle_index + 1,
           handles + new_handle_index,
           (handle_count - new_handle_index) * sizeof(SeqRetimingHandle));
  }
  MEM_freeN(handles);
  seq->retiming_handles = new_handles;
  seq->retiming_handle_num++;

  SeqRetimingHandle *added_handle = (new_handles + new_handle_index);
  added_handle->strip_frame_index = frame_index;
  added_handle->retiming_factor = value;

  return added_handle;
}

void SEQ_retiming_offset_handle(const Scene *scene,
                                Sequence *seq,
                                SeqRetimingHandle *handle,
                                const int offset)
{
  if (handle->strip_frame_index == 0) {
    return; /* First handle can not be moved. */
  }

  MutableSpan handles = SEQ_retiming_handles_get(seq);
  for (; handle < handles.end(); handle++) {
    handle->strip_frame_index += offset * seq_time_media_playback_rate_factor_get(scene, seq);
  }

  SEQ_time_update_meta_strip_range(scene, seq_sequence_lookup_meta_by_seq(scene, seq));
  seq_time_update_effects_strip_range(scene, seq_sequence_lookup_effects_by_seq(scene, seq));
}

static void seq_retiming_remove_handle_ex(Sequence *seq, SeqRetimingHandle *handle)
{
  SeqRetimingHandle *last_handle = SEQ_retiming_last_handle_get(seq);
  if (handle->strip_frame_index == 0 || handle == last_handle) {
    return; /* First and last handle can not be removed. */
  }

  size_t handle_count = SEQ_retiming_handles_count(seq);
  SeqRetimingHandle *handles = (SeqRetimingHandle *)MEM_callocN(
      (handle_count - 1) * sizeof(SeqRetimingHandle), __func__);

  const int handle_index = handle - seq->retiming_handles;
  memcpy(handles, seq->retiming_handles, (handle_index) * sizeof(SeqRetimingHandle));
  memcpy(handles + handle_index,
         seq->retiming_handles + handle_index + 1,
         (handle_count - handle_index - 1) * sizeof(SeqRetimingHandle));
  MEM_freeN(seq->retiming_handles);
  seq->retiming_handles = handles;
  seq->retiming_handle_num--;
}

/* This function removes gradual segment and creates retiming handle where it originally was.
 */
static void seq_retiming_remove_gradient(const Scene *scene,
                                         Sequence *seq,
                                         SeqRetimingHandle *handle)
{
  double s1_1[2], s1_2[2], s2_1[2], s2_2[2];

  /* Get 2 segments. */
  seq_retiming_segment_as_line_segment(handle - 1, s1_1, s1_2);
  seq_retiming_segment_as_line_segment(handle + 1, s2_1, s2_2);
  /* Find frame where handle originally was. */
  double lambda, mu;
  // XXX may be colinear - in that case the handle should be in middle of s1 and s2.
  int res = isect_seg_seg_v2_lambda_mu_db(s1_1, s1_2, s2_1, s2_2, &lambda, &mu);
  double new_handle_frame = s1_1[0] + lambda * (s1_2[0] - s1_1[0]) + SEQ_time_start_frame_get(seq);
  /* Create original linear handle. */
  SeqRetimingHandle *orig_handle = SEQ_retiming_add_handle(scene, seq, new_handle_frame);
  orig_handle->retiming_factor = s1_1[1] + lambda * (s1_2[1] - s1_1[1]);
  /* Remove handle to the left (gradual) and to the right (linear). */
  int orig_handle_index = SEQ_retiming_handle_index_get(seq, orig_handle);

  seq_retiming_remove_handle_ex(seq, seq->retiming_handles + orig_handle_index - 1);
  seq_retiming_remove_handle_ex(seq, seq->retiming_handles + orig_handle_index);
}

void SEQ_retiming_remove_handle(const Scene *scene, Sequence *seq, SeqRetimingHandle *handle)
{
  if ((handle->flag & 1) != 0) {
    seq_retiming_remove_gradient(scene, seq, handle);
    return;
  }
  SeqRetimingHandle *previous_handle = handle - 1;
  if ((previous_handle->flag & 1) != 0) {
    seq_retiming_remove_gradient(scene, seq, previous_handle);
    return;
  }
  seq_retiming_remove_handle_ex(seq, handle);
}

SeqRetimingHandle *SEQ_retiming_add_gradient(const Scene *scene,
                                             Sequence *seq,
                                             SeqRetimingHandle *handle,
                                             const int offset)
{
  int orig_handle_index = SEQ_retiming_handle_index_get(seq, handle);
  int orig_timeline_frame = SEQ_retiming_handle_timeline_frame_get(scene, seq, handle);
  SeqRetimingHandle *new_handle = SEQ_retiming_add_handle(
      scene, seq, orig_timeline_frame - offset);
  new_handle->flag |= 1;
  SEQ_retiming_add_handle(scene, seq, orig_timeline_frame + offset);
  seq_retiming_remove_handle_ex(seq, &SEQ_retiming_handles_get(seq)[orig_handle_index + 1]);
  return new_handle;
}

float SEQ_retiming_handle_speed_get(const Sequence *seq, const SeqRetimingHandle *handle)
{
  if (handle->strip_frame_index == 0) {
    return 1.0f;
  }

  const SeqRetimingHandle *handle_prev = handle - 1;

  const int frame_index_max = seq->len - 1;
  const int frame_retimed_prev = round_fl_to_int(handle_prev->retiming_factor * frame_index_max);
  const int frame_index_prev = handle_prev->strip_frame_index;
  const int frame_retimed = round_fl_to_int(handle->retiming_factor * frame_index_max);
  const int frame_index = handle->strip_frame_index;

  const int fragment_length_retimed = frame_retimed - frame_retimed_prev;
  const int fragment_length_original = frame_index - frame_index_prev;

  const float speed = float(fragment_length_retimed) / float(fragment_length_original);
  return speed;
}

float SEQ_retiming_handle_timeline_frame_get(const Scene *scene,
                                             const Sequence *seq,
                                             const SeqRetimingHandle *handle)
{
  return SEQ_time_start_frame_get(seq) +
         handle->strip_frame_index / seq_time_media_playback_rate_factor_get(scene, seq);
}
