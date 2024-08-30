/* SPDX-FileCopyrightText: 2021-2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spseq
 */

#include "BKE_context.hh"

#include "IMB_imbuf.hh"

#include "DNA_sequence_types.h"
#include "DNA_space_types.h"

#include "SEQ_channels.hh"
#include "SEQ_render.hh"
#include "SEQ_sequencer.hh"
#include "SEQ_thumbnail_cache.hh"
#include "SEQ_time.hh"

#include "sequencer_intern.hh" /* Own include. */

using namespace blender;

static float thumb_calc_first_timeline_frame(const Scene *scene,
                                             Sequence *seq,
                                             float frame_step,
                                             const rctf *view_area)
{
  int first_drawable_frame = max_iii(
      SEQ_time_left_handle_frame_get(scene, seq), seq->start, view_area->xmin);

  /* First frame should correspond to handle position. */
  if (first_drawable_frame == SEQ_time_left_handle_frame_get(scene, seq)) {
    return SEQ_time_left_handle_frame_get(scene, seq);
  }

  float aligned_frame_offset = int((first_drawable_frame - seq->start) / frame_step) * frame_step;
  return seq->start + aligned_frame_offset;
}

static float thumb_calc_next_timeline_frame(const Scene *scene,
                                            Sequence *seq,
                                            float last_frame,
                                            float frame_step)
{
  float next_frame = last_frame + frame_step;

  /* If handle position was displayed, align next frame with `seq->start`. */
  if (last_frame == SEQ_time_left_handle_frame_get(scene, seq)) {
    next_frame = seq->start + (int((last_frame - seq->start) / frame_step) + 1) * frame_step;
  }

  return next_frame;
}

static void seq_get_thumb_image_dimensions(Sequence *seq,
                                           float pixelx,
                                           float pixely,
                                           float *r_thumb_width,
                                           float thumb_height,
                                           float *r_image_width,
                                           float *r_image_height)
{
  float image_width = seq->strip->stripdata->orig_width;
  float image_height = seq->strip->stripdata->orig_height;

  /* Fix the dimensions to be max SEQ_THUMB_SIZE for x or y. */
  float aspect_ratio = image_width / image_height;
  if (image_width > image_height) {
    image_width = seq::SEQ_THUMB_SIZE;
    image_height = round_fl_to_int(image_width / aspect_ratio);
  }
  else {
    image_height = seq::SEQ_THUMB_SIZE;
    image_width = round_fl_to_int(image_height * aspect_ratio);
  }

  /* Calculate thumb dimensions. */
  aspect_ratio = image_width / image_height;
  float thumb_h_px = thumb_height / pixely;
  float thumb_width = aspect_ratio * thumb_h_px * pixelx;

  *r_thumb_width = thumb_width;
  if (r_image_width && r_image_height) {
    *r_image_width = image_width;
    *r_image_height = image_height;
  }
}

void get_seq_strip_thumbnails(View2D *v2d,
                              const bContext *C,
                              Scene *scene,
                              Sequence *seq,
                              float y1,
                              float y2,
                              float y_top,
                              float pixelx,
                              float pixely,
                              Vector<SeqThumbInfo> &r_thumbs)
{
  SpaceSeq *sseq = CTX_wm_space_seq(C);
  if ((sseq->flag & SEQ_SHOW_OVERLAY) == 0 ||
      (sseq->timeline_overlay.flag & SEQ_TIMELINE_SHOW_THUMBNAILS) == 0 ||
      !ELEM(seq->type, SEQ_TYPE_MOVIE, SEQ_TYPE_IMAGE))
  {
    return;
  }

  StripElem *se = seq->strip->stripdata;
  if (se->orig_height == 0 || se->orig_width == 0) {
    return;
  }

  /* If width of the strip too small ignore drawing thumbnails. */
  if ((y2 - y1) / pixely <= 20 * UI_SCALE_FAC) {
    return;
  }

  Editing *ed = SEQ_editing_get(scene);
  ListBase *channels = ed ? SEQ_channels_displayed_get(ed) : nullptr;

  float thumb_width, image_width, image_height;
  const float thumb_height = y2 - y1;
  seq_get_thumb_image_dimensions(
      seq, pixelx, pixely, &thumb_width, thumb_height, &image_width, &image_height);

  const float zoom_y = thumb_height / image_height;
  const float crop_x_multiplier = 1.0f / pixelx / (zoom_y / pixely);

  const float seq_left_handle = SEQ_time_left_handle_frame_get(scene, seq);
  const float seq_right_handle = SEQ_time_right_handle_frame_get(scene, seq);

  float upper_thumb_bound = SEQ_time_has_right_still_frames(scene, seq) ?
                                SEQ_time_content_end_frame_get(scene, seq) :
                                seq_right_handle;
  if (seq->type == SEQ_TYPE_IMAGE) {
    upper_thumb_bound = seq_right_handle;
  }

  float timeline_frame = thumb_calc_first_timeline_frame(scene, seq, thumb_width, &v2d->cur);

  /* Start drawing. */
  while (timeline_frame < upper_thumb_bound) {
    float thumb_x_end = timeline_frame + thumb_width;
    bool clipped = false;

    /* Checks to make sure that thumbs are loaded only when in view and within the confines of the
     * strip. Some may not be required but better to have conditions for safety as x1 here is
     * point to start caching from and not drawing. */
    if (timeline_frame > v2d->cur.xmax) {
      break;
    }

    /* Set the clipping bound to show the left handle moving over thumbs and not shift thumbs. */
    float cut_off = 0.0f;
    if (seq_left_handle > timeline_frame && seq_left_handle < thumb_x_end) {
      cut_off = seq_left_handle - timeline_frame;
      clipped = true;
    }

    /* Clip if full thumbnail cannot be displayed. */
    if (thumb_x_end > upper_thumb_bound) {
      thumb_x_end = upper_thumb_bound;
      clipped = true;
    }

    int cropx_min = int(cut_off * crop_x_multiplier);
    int cropx_max = int((thumb_x_end - timeline_frame) * crop_x_multiplier);
    if (cropx_max < 1) {
      break;
    }

    /* Get the thumbnail image. */
    ImBuf *ibuf = seq::thumbnail_cache_get(C, scene, seq, timeline_frame);
    if (ibuf == nullptr) {
      break;
    }

    SeqThumbInfo thumb = {};
    thumb.ibuf = ibuf;
    thumb.cropx_min = 0;
    thumb.cropx_max = ibuf->x - 1;
    if (clipped) {
      thumb.cropx_min = clamp_i(cropx_min, 0, ibuf->x - 1);
      thumb.cropx_max = clamp_i(cropx_max - 1, 0, ibuf->x - 1);
    }
    thumb.left_handle = seq_left_handle;
    thumb.right_handle = seq_right_handle;
    thumb.muted = channels ? SEQ_render_is_muted(channels, seq) :
                             false;  //@TODO: do this once per strip
    thumb.bottom = y1;
    thumb.top = y_top;
    thumb.x1 = timeline_frame + cut_off;
    thumb.x2 = thumb_x_end;
    thumb.y1 = y1;
    thumb.y2 = y2;
    r_thumbs.append(thumb);

    timeline_frame = thumb_calc_next_timeline_frame(scene, seq, timeline_frame, thumb_width);
  }
}
