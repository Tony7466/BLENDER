/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spseq
 */

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_math.h"

#include "DNA_anim_types.h"
#include "DNA_scene_types.h"

#include "BKE_context.h"
#include "BKE_report.h"
#include "BKE_scene.h"

#include "SEQ_iterator.h"
#include "SEQ_relations.h"
#include "SEQ_retiming.h"
#include "SEQ_retiming.hh"
#include "SEQ_sequencer.h"
#include "SEQ_time.h"
#include "SEQ_transform.h"

#include "WM_api.h"
#include "WM_toolsystem.h"

#include "RNA_define.h"

#include "UI_interface.h"
#include "UI_view2d.h"

#include "DEG_depsgraph.h"

/* Own include. */
#include "sequencer_intern.h"

using blender::MutableSpan;

static bool retiming_poll(bContext *C)
{
  const Editing *ed = SEQ_editing_get(CTX_data_scene(C));
  if (ed == nullptr) {
    return false;
  }
  Sequence *seq = ed->act_seq;
  if (seq == nullptr) {
    return false;
  }
  if (!SEQ_retiming_is_allowed(seq)) {
    CTX_wm_operator_poll_msg_set(C, "This strip type can not be retimed");
    return false;
  }
  return true;
}

static void retiming_handle_overlap(Scene *scene, Sequence *seq)
{
  ListBase *seqbase = SEQ_active_seqbase_get(SEQ_editing_get(scene));
  SeqCollection *strips = SEQ_collection_create(__func__);
  SEQ_collection_append_strip(seq, strips);
  SeqCollection *dependant = SEQ_collection_create(__func__);
  SEQ_collection_expand(scene, seqbase, strips, SEQ_query_strip_effect_chain);
  SEQ_collection_remove_strip(seq, dependant);
  SEQ_transform_handle_overlap(scene, seqbase, strips, dependant, true);
  SEQ_collection_free(strips);
  SEQ_collection_free(dependant);
}

static SeqRetimingHandle *closest_retiming_handle_get(const bContext *C,
                                                      const Sequence *seq,
                                                      const float mouse_x)
{
  const View2D *v2d = UI_view2d_fromcontext(C);
  const Scene *scene = CTX_data_scene(C);
  int best_distance = INT_MAX;
  SeqRetimingHandle *closest_handle = nullptr;

  // XXX this is not needed. Gizmo has own threshold which must be met in order to execute op
  // anyway.
  const float distance_threshold = 10 / UI_view2d_scale_get_x(v2d);
  const float mouse_x_view = UI_view2d_region_to_view_x(v2d, mouse_x);

  for (int i = 0; i < SEQ_retiming_handles_count(seq); i++) {
    SeqRetimingHandle *handle = seq->retiming_handles + i;
    const int distance = round_fl_to_int(
        fabsf(SEQ_retiming_handle_timeline_frame_get(scene, seq, handle) - mouse_x_view));

    if (distance < distance_threshold && distance < best_distance) {
      best_distance = distance;
      closest_handle = handle;
    }
  }
  return closest_handle;
}

/*-------------------------------------------------------------------- */
/** \name Retiming Reset
 * \{ */

static int sequencer_retiming_reset_exec(bContext *C, wmOperator * /* op */)
{
  Scene *scene = CTX_data_scene(C);
  const Editing *ed = SEQ_editing_get(scene);
  Sequence *seq = ed->act_seq;

  SEQ_retiming_data_clear(seq);

  retiming_handle_overlap(scene, seq);
  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
  return OPERATOR_FINISHED;
}

void SEQUENCER_OT_retiming_reset(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Reset Retiming";
  ot->description = "Reset strip retiming";
  ot->idname = "SEQUENCER_OT_retiming_reset";

  /* api callbacks */
  ot->exec = sequencer_retiming_reset_exec;
  ot->poll = retiming_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Move Handle
 * \{ */

static int sequencer_retiming_handle_move_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  /* Pass pressed modifier key to exec function. */
  op->customdata = POINTER_FROM_UINT(event->modifier & (KM_SHIFT | KM_CTRL));

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static SeqRetimingHandle *make_speed_transition(const Scene *scene,
                                                Sequence *seq,
                                                SeqRetimingHandle *handle,
                                                const int offset)
{
  SeqRetimingHandle *transition_handle = SEQ_retiming_add_transition(
      scene, seq, handle, abs(offset));
  if (transition_handle == nullptr) {
    return nullptr;
  }

  /* New gradient handle was created - update operator properties. */
  if (offset < 0) {
    return transition_handle;
  }

  return transition_handle + 1;
}

static int sequencer_retiming_handle_move_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  Scene *scene = CTX_data_scene(C);
  const ARegion *region = CTX_wm_region(C);
  const View2D *v2d = &region->v2d;
  const Editing *ed = SEQ_editing_get(scene);
  Sequence *seq = ed->act_seq;

  int handle_index = RNA_int_get(op->ptr, "handle_index");
  SeqRetimingHandle *handle = &SEQ_retiming_handles_get(seq)[handle_index];

  switch (event->type) {
    case MOUSEMOVE: {
      float mouse_x = UI_view2d_region_to_view_x(v2d, event->mval[0]);
      int offset = 0;

      offset = mouse_x - SEQ_retiming_handle_timeline_frame_get(scene, seq, handle);

      if (offset == 0) {
        return OPERATOR_RUNNING_MODAL;
      }

      uint8_t invoke_modifier = POINTER_AS_UINT(op->customdata);
      /* Add retiming gradient and move handle. */
      if (invoke_modifier != 0) {
        SeqRetimingHandle *new_handle = nullptr;
        if ((invoke_modifier & KM_SHIFT) != 0) {
          new_handle = make_speed_transition(scene, seq, handle, offset);
        }
        else if ((invoke_modifier & KM_CTRL) != 0) {
          new_handle = SEQ_retiming_add_freeze_frame(scene, seq, handle, abs(offset));
        }

        if (new_handle != nullptr) {
          handle = new_handle;
          RNA_int_set(op->ptr, "handle_index", SEQ_retiming_handle_index_get(seq, new_handle));
        }
        op->customdata = POINTER_FROM_UINT(0);
      }

      const bool handle_is_transition = SEQ_retiming_handle_is_transition_type(handle);
      const bool prev_handle_is_transition = SEQ_retiming_handle_is_transition_type(handle - 1);

      /* When working with transition, change handles when moving past pivot point. */
      if (handle_is_transition || prev_handle_is_transition) {
        SeqRetimingHandle *transition_start, *transition_end;
        if (handle_is_transition) {
          transition_start = handle;
          transition_end = handle + 1;
        }
        else {
          transition_start = handle - 1;
          transition_end = handle;
        }
        const int offset_l = mouse_x -
                             SEQ_retiming_handle_timeline_frame_get(scene, seq, transition_start);
        const int offset_r = mouse_x -
                             SEQ_retiming_handle_timeline_frame_get(scene, seq, transition_end);

        if (prev_handle_is_transition && offset_l < 0) {
          RNA_int_set(
              op->ptr, "handle_index", SEQ_retiming_handle_index_get(seq, transition_start));
        }
        if (handle_is_transition && offset_r > 0) {
          RNA_int_set(op->ptr, "handle_index", SEQ_retiming_handle_index_get(seq, transition_end));
        }
      }

      SEQ_retiming_offset_handle(scene, seq, handle, offset);

      SEQ_relations_invalidate_cache_raw(scene, seq);
      WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
      return OPERATOR_RUNNING_MODAL;
    }
    case LEFTMOUSE:
    case EVT_RETKEY:
    case EVT_SPACEKEY: {
      retiming_handle_overlap(scene, seq);
      DEG_id_tag_update(&scene->id, ID_RECALC_SEQUENCER_STRIPS);
      WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
      return OPERATOR_FINISHED;
    }

    case EVT_ESCKEY:
    case RIGHTMOUSE: {
      WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
      return OPERATOR_CANCELLED;
    }
  }
  return OPERATOR_RUNNING_MODAL;
}

void SEQUENCER_OT_retiming_handle_move(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Move Retiming Handle";
  ot->description = "Move retiming handle";
  ot->idname = "SEQUENCER_OT_retiming_handle_move";

  /* api callbacks */
  ot->invoke = sequencer_retiming_handle_move_invoke;
  ot->modal = sequencer_retiming_handle_move_modal;
  ot->poll = retiming_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  PropertyRNA *prop = RNA_def_int(ot->srna,
                                  "handle_index",
                                  0,
                                  0,
                                  INT_MAX,
                                  "Handle Index",
                                  "Index of handle to be moved",
                                  0,
                                  INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Add Handle
 * \{ */

static int sequesequencer_retiming_handle_add_exec(bContext *C, wmOperator *op)
{
  Scene *scene = CTX_data_scene(C);

  float timeline_frame;
  if (RNA_struct_property_is_set(op->ptr, "timeline_frame")) {
    timeline_frame = RNA_int_get(op->ptr, "timeline_frame");
  }
  else {
    timeline_frame = BKE_scene_frame_get(scene);
  }

  bool inserted = false;

  SeqCollection *strips = selected_strips_from_context(C);
  Sequence *seq;
  SEQ_ITERATOR_FOREACH (seq, strips) {
    SEQ_retiming_data_ensure(scene, seq);
    const int frame_index = BKE_scene_frame_get(scene) - SEQ_time_start_frame_get(seq);
    const SeqRetimingHandle *handle = SEQ_retiming_find_segment_start_handle(seq, frame_index);

    if (SEQ_retiming_handle_is_transition_type(handle)) {
      BKE_report(op->reports, RPT_WARNING, "Can not create handle inside of speed transition");
      continue;
    }

    const float end_frame = seq->start + SEQ_time_strip_length_get(scene, seq);
    if (seq->start < timeline_frame && end_frame > timeline_frame) {
      SEQ_retiming_add_handle(scene, seq, timeline_frame);
      inserted = true;
    }
  }

  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);

  return inserted ? OPERATOR_FINISHED : OPERATOR_PASS_THROUGH;
}

void SEQUENCER_OT_retiming_handle_add(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Add Retiming Handle";
  ot->description = "Add retiming Handle";
  ot->idname = "SEQUENCER_OT_retiming_handle_add";

  /* api callbacks */
  ot->exec = sequesequencer_retiming_handle_add_exec;
  ot->poll = retiming_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna,
              "timeline_frame",
              0,
              0,
              INT_MAX,
              "Timeline Frame",
              "Frame where handle will be added",
              0,
              INT_MAX);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Remove Handle
 * \{ */

static int sequencer_retiming_handle_remove_exec(bContext *C, wmOperator * /* op */)
{
  Scene *scene = CTX_data_scene(C);

  blender::Vector<Sequence *> strips_to_handle;

  for (const RetimingSelectionElem *elem : SEQ_retiming_selection_get(scene)) {
    strips_to_handle.append_non_duplicates(elem->seq);
    elem->handle->flag |= DELETE_HANDLE;
  }

  for (Sequence *seq : strips_to_handle) {
    for (int i = 0; i < seq->retiming_handle_num;) {
      SeqRetimingHandle *handle = seq->retiming_handles + i;

      if ((handle->flag & DELETE_HANDLE) == 0) {
        i++;
        continue;
      }

      SEQ_retiming_remove_handle(scene, seq, handle);
    }
    SEQ_relations_invalidate_cache_raw(scene, seq);
  }

  SEQ_retiming_selection_clear(SEQ_editing_get(scene));

  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
  return OPERATOR_FINISHED;
}

void SEQUENCER_OT_retiming_handle_remove(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Remove Retiming Handle";
  ot->description = "Remove retiming handle";
  ot->idname = "SEQUENCER_OT_retiming_handle_remove";

  /* api callbacks */
  ot->exec = sequencer_retiming_handle_remove_exec;
  ot->poll = retiming_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  PropertyRNA *prop = RNA_def_int(ot->srna,
                                  "handle_index",
                                  0,
                                  0,
                                  INT_MAX,
                                  "Handle Index",
                                  "Index of handle to be removed",
                                  0,
                                  INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Set Segment Speed
 * \{ */

static int sequencer_retiming_segment_speed_set_exec(bContext *C, wmOperator *op)
{
  Scene *scene = CTX_data_scene(C);
  const Editing *ed = SEQ_editing_get(scene);
  Sequence *seq = ed->act_seq;
  MutableSpan handles = SEQ_retiming_handles_get(seq);
  SeqRetimingHandle *handle = &handles[RNA_int_get(op->ptr, "handle_index")];

  SEQ_retiming_handle_speed_set(scene, seq, handle, RNA_float_get(op->ptr, "speed"));
  SEQ_relations_invalidate_cache_raw(scene, seq);
  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
  return OPERATOR_FINISHED;
}

static int sequencer_retiming_segment_speed_set_invoke(bContext *C,
                                                       wmOperator *op,
                                                       const wmEvent *event)
{
  const Scene *scene = CTX_data_scene(C);
  const Editing *ed = SEQ_editing_get(scene);
  const Sequence *seq = ed->act_seq;

  if (seq == nullptr) {
    return OPERATOR_CANCELLED;
  }

  MutableSpan handles = SEQ_retiming_handles_get(seq);
  SeqRetimingHandle *handle = nullptr;

  if (RNA_struct_property_is_set(op->ptr, "handle_index")) {
    const int handle_index = RNA_int_get(op->ptr, "handle_index");
    BLI_assert(handle_index < handles.size());
    handle = &handles[handle_index];
  }
  else {
    handle = closest_retiming_handle_get(C, seq, event->mval[0]);
  }

  if (handle == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "No handle available");
    return OPERATOR_CANCELLED;
  }

  RNA_float_set(op->ptr, "speed", SEQ_retiming_handle_speed_get(seq, handle) * 100.0f);
  RNA_int_set(op->ptr, "handle_index", SEQ_retiming_handle_index_get(seq, handle));
  return WM_operator_props_popup(C, op, event);
}

void SEQUENCER_OT_retiming_segment_speed_set(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Set Speed";
  ot->description = "Set speed of retimed segment";
  ot->idname = "SEQUENCER_OT_retiming_segment_speed_set";

  /* api callbacks */
  ot->invoke = sequencer_retiming_segment_speed_set_invoke;
  ot->exec = sequencer_retiming_segment_speed_set_exec;
  ot->poll = retiming_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  PropertyRNA *prop = RNA_def_int(ot->srna,
                                  "handle_index",
                                  0,
                                  0,
                                  INT_MAX,
                                  "Handle Index",
                                  "Index of handle to be removed",
                                  0,
                                  INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);

  prop = RNA_def_float(ot->srna,
                       "speed",
                       100.0f,
                       0.001f,
                       FLT_MAX,
                       "Speed",
                       "New speed of retimed segment",
                       0.1f,
                       FLT_MAX);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Handle Select
 * \{ */

static int sequencer_retiming_handle_select_exec(bContext *C, wmOperator *op)
{
  Scene *scene = CTX_data_scene(C);
  Editing *ed = SEQ_editing_get(scene);
  const View2D *v2d = UI_view2d_fromcontext(C);
  int handle_clicked;
  int mval[2] = {RNA_int_get(op->ptr, "mouse_x"), RNA_int_get(op->ptr, "mouse_y")};

  Sequence *seq = find_nearest_seq(scene, v2d, &handle_clicked, mval);

  if (seq == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "No handle available");
    return OPERATOR_CANCELLED;
  }

  SeqRetimingHandle *handle = closest_retiming_handle_get(C, seq, mval[0]);

  if (handle == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "No handle available");
    return OPERATOR_CANCELLED;
  }

  SEQ_retiming_selection_append(ed, seq, handle);

  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
  return OPERATOR_FINISHED;
}

static int sequencer_retiming_handle_select_invoke(bContext *C,
                                                   wmOperator *op,
                                                   const wmEvent *event)
{
  RNA_int_set(op->ptr, "mouse_x", event->mval[0]);
  RNA_int_set(op->ptr, "mouse_y", event->mval[1]);
  return sequencer_retiming_handle_select_exec(C, op);
}

void SEQUENCER_OT_retiming_handle_select(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Select Retiming Handle";
  ot->description = "Select Retiming Handle";
  ot->idname = "SEQUENCER_OT_retiming_handle_select";

  /* api callbacks */
  ot->exec = sequencer_retiming_handle_select_exec;
  ot->invoke = sequencer_retiming_handle_select_invoke;
  ot->poll = retiming_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  PropertyRNA *prop = RNA_def_int(
      ot->srna, "mouse_x", 0, INT_MIN, INT_MAX, "Mouse X", "", INT_MIN, INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  prop = RNA_def_int(ot->srna, "mouse_y", 0, INT_MIN, INT_MAX, "Mouse Y", "", INT_MIN, INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

/** \} */
