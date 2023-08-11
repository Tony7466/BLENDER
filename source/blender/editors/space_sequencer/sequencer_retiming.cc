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
#include "DNA_workspace_types.h"

#include "BKE_context.h"
#include "BKE_report.h"
#include "BKE_scene.h"

#include "ED_select_utils.hh"

#include "SEQ_iterator.h"
#include "SEQ_relations.h"
#include "SEQ_retiming.h"
#include "SEQ_retiming.hh"
#include "SEQ_sequencer.h"
#include "SEQ_time.h"
#include "SEQ_transform.h"

#include "WM_api.hh"
#include "WM_toolsystem.h"

#include "RNA_define.h"

#include "UI_interface.hh"
#include "UI_view2d.hh"

#include "DEG_depsgraph.h"

/* Own include. */
#include "sequencer_intern.h"
#include "sequencer_intern.hh"

using blender::MutableSpan;

bool sequencer_retiming_tool_is_active(const bContext *C)
{
  ScrArea *area = CTX_wm_area(C);
  if (area->runtime.is_tool_set) {
    return STREQ(area->runtime.tool->idname, "builtin.retime");
  }
  return false;
}

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

static void retiming_key_overlap(Scene *scene, Sequence *seq)
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

/*-------------------------------------------------------------------- */
/** \name Retiming Reset
 * \{ */

static int sequencer_retiming_reset_exec(bContext *C, wmOperator * /* op */)
{
  Scene *scene = CTX_data_scene(C);
  const Editing *ed = SEQ_editing_get(scene);
  Sequence *seq = ed->act_seq;

  SEQ_retiming_data_clear(seq);

  retiming_key_overlap(scene, seq);
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
/** \name Retiming Add Key
 * \{ */

static int sequencer_retiming_key_add_exec(bContext *C, wmOperator *op)
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
    const SeqRetimingKey *key = SEQ_retiming_find_segment_start_key(seq, frame_index);

    if (SEQ_retiming_key_is_transition_start(key)) {
      BKE_report(op->reports, RPT_WARNING, "Can not create key inside of speed transition");
      continue;
    }

    const float end_frame = seq->start + SEQ_time_strip_length_get(scene, seq);
    if (seq->start < timeline_frame && end_frame > timeline_frame) {
      SEQ_retiming_add_key(scene, seq, timeline_frame);
      inserted = true;
    }
  }
  SEQ_collection_free(strips);

  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);

  return inserted ? OPERATOR_FINISHED : OPERATOR_PASS_THROUGH;
}

void SEQUENCER_OT_retiming_key_add(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Add Retiming Key";
  ot->description = "Add retiming Key";
  ot->idname = "SEQUENCER_OT_retiming_key_add";

  /* api callbacks */
  ot->exec = sequencer_retiming_key_add_exec;
  ot->poll = retiming_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna,
              "timeline_frame",
              0,
              0,
              INT_MAX,
              "Timeline Frame",
              "Frame where key will be added",
              0,
              INT_MAX);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Add Freeze Frame
 * \{ */

bool freeze_frame_add_new_for_seq(
    bContext *C, wmOperator *op, Sequence *seq, const int timeline_frame, const int duration)
{
  Scene *scene = CTX_data_scene(C);
  SEQ_retiming_data_ensure(scene, seq);
  SeqRetimingKey *key = SEQ_retiming_add_key(scene, seq, timeline_frame);

  if (key == nullptr) {
    key = SEQ_retiming_key_get_by_timeline_frame(scene, seq, timeline_frame);
  }

  if (SEQ_retiming_key_is_transition_start(key)) {
    BKE_report(op->reports, RPT_WARNING, "Can not create key inside of speed transition");
    return false;
  }
  if (key == nullptr) {
    BKE_report(op->reports, RPT_WARNING, "Can not create freeze frame");
    return false;
  }

  SeqRetimingKey *freeze = SEQ_retiming_add_freeze_frame(scene, seq, key, duration);

  if (freeze == nullptr) {
    BKE_report(op->reports, RPT_WARNING, "Can not create freeze frame");
    return false;
  }

  SEQ_relations_invalidate_cache_raw(scene, seq);
  return true;
}

static bool freeze_frame_add_from_strip_selection(bContext *C, wmOperator *op, const int duration)
{
  Scene *scene = CTX_data_scene(C);
  SeqCollection *strips = selected_strips_from_context(C);
  const int timeline_frame = BKE_scene_frame_get(scene);
  bool success = false;

  Sequence *seq;
  SEQ_ITERATOR_FOREACH (seq, strips) {
    success |= freeze_frame_add_new_for_seq(C, op, seq, timeline_frame, duration);
  }
  SEQ_collection_free(strips);
  return success;
}

static bool freeze_frame_add_from_retiming_selection(bContext *C,
                                                     wmOperator *op,
                                                     const int duration)
{
  Scene *scene = CTX_data_scene(C);
  bool success = false;

  for (const RetimingSelectionElem elem : SEQ_retiming_selection_get(scene)) {
    const int timeline_frame = SEQ_retiming_key_timeline_frame_get(scene, elem.seq, elem.key);
    success |= freeze_frame_add_new_for_seq(C, op, elem.seq, timeline_frame, duration);
  }
  return success;
}

static int sequencer_retiming_freeze_frame_add_exec(bContext *C, wmOperator *op)
{
  Scene *scene = CTX_data_scene(C);
  bool success = false;

  const float fps = scene->r.frs_sec / scene->r.frs_sec_base;
  int duration = 4 * fps;

  if (RNA_property_is_set(op->ptr, RNA_struct_find_property(op->ptr, "duration"))) {
    duration = RNA_int_get(op->ptr, "duration");
  }

  if (sequencer_retiming_tool_is_active(C)) {
    success = freeze_frame_add_from_retiming_selection(C, op, duration);
  }
  else {
    success = freeze_frame_add_from_strip_selection(C, op, duration);
  }

  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);

  return success ? OPERATOR_FINISHED : OPERATOR_PASS_THROUGH;
}

void SEQUENCER_OT_retiming_freeze_frame_add(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Add Freeze Frame";
  ot->description = "Add freeze frame";
  ot->idname = "SEQUENCER_OT_retiming_freeze_frame_add";

  /* api callbacks */
  ot->exec = sequencer_retiming_freeze_frame_add_exec;
  ot->poll = retiming_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  RNA_def_int(ot->srna,
              "duration",
              0,
              0,
              INT_MAX,
              "Duration",
              "Duration of freeze frame segment",
              0,
              INT_MAX);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Add Speed Transition
 * \{ */

bool transition_add_new_for_seq(
    bContext *C, wmOperator *op, Sequence *seq, const int timeline_frame, const int duration)
{
  Scene *scene = CTX_data_scene(C);
  SEQ_retiming_data_ensure(scene, seq);
  SeqRetimingKey *key = SEQ_retiming_add_key(scene, seq, timeline_frame);

  if (key == nullptr) {
    key = SEQ_retiming_key_get_by_timeline_frame(scene, seq, timeline_frame);
  }

  SeqRetimingKey *transition = SEQ_retiming_add_transition(scene, seq, key, duration);

  if (transition == nullptr) {
    BKE_report(op->reports, RPT_WARNING, "Can not create transition");
    return false;
  }

  SEQ_relations_invalidate_cache_raw(scene, seq);
  return true;
}

static bool transition_add_from_retiming_selection(bContext *C, wmOperator *op, const int duration)
{
  Scene *scene = CTX_data_scene(C);
  bool success = false;

  for (const RetimingSelectionElem elem : SEQ_retiming_selection_get(scene)) {
    const int timeline_frame = SEQ_retiming_key_timeline_frame_get(scene, elem.seq, elem.key);
    success |= transition_add_new_for_seq(C, op, elem.seq, timeline_frame, duration);
  }
  return success;
}

static int sequencer_retiming_transition_add_exec(bContext *C, wmOperator *op)
{
  Scene *scene = CTX_data_scene(C);
  bool success = false;

  const float fps = scene->r.frs_sec / scene->r.frs_sec_base;
  int duration = 4 * fps;

  if (RNA_property_is_set(op->ptr, RNA_struct_find_property(op->ptr, "duration"))) {
    duration = RNA_int_get(op->ptr, "duration");
  }

  if (sequencer_retiming_tool_is_active(C)) {
    success = transition_add_from_retiming_selection(C, op, duration);
  }
  else {
    BKE_report(op->reports, RPT_WARNING, "Retiming key must be selected");
    return false;
  }

  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);

  return success ? OPERATOR_FINISHED : OPERATOR_PASS_THROUGH;
}

void SEQUENCER_OT_retiming_transition_add(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Add Speed Transition";
  ot->description = "Add smooth transition between 2 retimed segments";
  ot->idname = "SEQUENCER_OT_retiming_transition_add";

  /* api callbacks */
  ot->exec = sequencer_retiming_transition_add_exec;
  ot->poll = retiming_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  RNA_def_int(ot->srna,
              "duration",
              0,
              0,
              INT_MAX,
              "Duration",
              "Duration of freeze frame segment",
              0,
              INT_MAX);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Remove Key
 * \{ */

static int sequencer_retiming_key_remove_exec(bContext *C, wmOperator * /* op */)
{
  Scene *scene = CTX_data_scene(C);

  blender::Vector<Sequence *> strips_to_handle;

  for (const RetimingSelectionElem elem : SEQ_retiming_selection_get(scene)) {
    strips_to_handle.append_non_duplicates(elem.seq);
    elem.key->flag |= DELETE_KEY;
  }

  for (Sequence *seq : strips_to_handle) {
    for (int i = 0; i < seq->retiming_keys_num;) {
      SeqRetimingKey *key = seq->retiming_keys + i;
      i++;

      if ((key->flag & DELETE_KEY) == 0) {
        continue;
      }

      SEQ_retiming_remove_key(scene, seq, key);
    }
    SEQ_relations_invalidate_cache_raw(scene, seq);
  }

  SEQ_retiming_selection_clear(SEQ_editing_get(scene));

  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
  return OPERATOR_FINISHED;
}

void SEQUENCER_OT_retiming_key_remove(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Remove Retiming Key";
  ot->description = "Remove retiming key";
  ot->idname = "SEQUENCER_OT_retiming_key_remove";

  /* api callbacks */
  ot->exec = sequencer_retiming_key_remove_exec;
  ot->poll = retiming_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Set Segment Speed
 * \{ */

static int sequencer_retiming_segment_speed_set_exec(bContext *C, wmOperator *op)
{
  Scene *scene = CTX_data_scene(C);
  const Editing *ed = SEQ_editing_get(scene);

  if (BLI_listbase_is_empty(&ed->retiming_selection)) {
    BKE_report(op->reports, RPT_ERROR, "No key available");
    return OPERATOR_CANCELLED;
  }

  for (const RetimingSelectionElem elem : SEQ_retiming_selection_get(scene)) {
    SEQ_retiming_key_speed_set(scene, elem.seq, elem.key, RNA_float_get(op->ptr, "speed"));
    SEQ_relations_invalidate_cache_raw(scene, elem.seq);
  }

  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
  return OPERATOR_FINISHED;
}

void SEQUENCER_OT_retiming_segment_speed_set(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Set Speed";
  ot->description = "Set speed of retimed segment";
  ot->idname = "SEQUENCER_OT_retiming_segment_speed_set";

  /* api callbacks */
  ot->exec = sequencer_retiming_segment_speed_set_exec;
  ot->poll = retiming_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Key Select
 * \{ */

static int sequencer_retiming_key_select_exec(bContext *C, wmOperator *op)
{
  Scene *scene = CTX_data_scene(C);
  Editing *ed = SEQ_editing_get(scene);
  int mval[2] = {RNA_int_get(op->ptr, "mouse_x"), RNA_int_get(op->ptr, "mouse_y")};

  int hand;
  Sequence *seq_click_exact = find_nearest_seq(scene, UI_view2d_fromcontext(C), &hand, mval);
  Sequence *seq_key_owner = nullptr;
  const SeqRetimingKey *key = retiming_mousover_key_get(C, mval, &seq_key_owner);

  const bool wait_to_deselect_others = RNA_boolean_get(op->ptr, "wait_to_deselect_others");
  const bool toggle = RNA_boolean_get(op->ptr, "toggle");

  /* Clicking on already selected element falls on modal operation.
   * All strips are deselected on mouse button release unless extend mode is used. */
  if (key && SEQ_retiming_selection_contains(ed, seq_key_owner, key) && wait_to_deselect_others &&
      !toggle)
  {
    return OPERATOR_RUNNING_MODAL;
  }
  /* If click happened on strip, with no key, prevent immediately switching to previous tool.
   * That should happen on mouse release. */
  if (seq_key_owner != nullptr && key == nullptr && wait_to_deselect_others && !toggle) {
    return OPERATOR_RUNNING_MODAL;
  }

  if (seq_key_owner != nullptr && key == nullptr) {
    WM_toolsystem_ref_set_by_id(C, "builtin.select");  // prev tool
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  bool changed = false;
  if (RNA_boolean_get(op->ptr, "deselect_all")) {
    changed = SEQ_retiming_selection_clear(ed);
  }

  if (key == nullptr && seq_click_exact != nullptr) {
    if (changed) {
      WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
      return OPERATOR_FINISHED;
    }
    else {
      WM_toolsystem_ref_set_by_id(C, "builtin.select");
      WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
      return OPERATOR_CANCELLED;
    }
  }

  if (seq_key_owner == nullptr && !changed) {
    return OPERATOR_CANCELLED;
  }

  if (seq_key_owner != nullptr) {
    if (toggle && SEQ_retiming_selection_contains(ed, seq_key_owner, key)) {
      SEQ_retiming_selection_remove(ed, seq_key_owner, key);
    }
    else {
      SEQ_retiming_selection_append(ed, seq_key_owner, key);
    }
  }

  WM_toolsystem_ref_set_by_id(C, "builtin.retime"); /* Switch to retiming tool. */
  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
  return changed ? OPERATOR_FINISHED : OPERATOR_CANCELLED;
}

static int sequencer_retiming_key_select_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  return WM_generic_select_invoke(C, op, event);
}

void SEQUENCER_OT_retiming_key_select(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Select Retiming Key";
  ot->description = "Select retiming key";
  ot->idname = "SEQUENCER_OT_retiming_key_select";

  /* api callbacks */
  ot->exec = sequencer_retiming_key_select_exec;
  ot->invoke = sequencer_retiming_key_select_invoke;
  ot->poll = retiming_poll;
  ot->modal = WM_generic_select_modal;
  ot->get_name = ED_select_pick_get_name;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  WM_operator_properties_generic_select(ot);
  WM_operator_properties_mouse_select(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Retiming Key Box Select
 * \{ */

static int sequencer_retiming_box_select_exec(bContext *C, wmOperator *op)
{

  Scene *scene = CTX_data_scene(C);
  View2D *v2d = UI_view2d_fromcontext(C);
  Editing *ed = SEQ_editing_get(scene);

  if (ed == nullptr) {
    return OPERATOR_CANCELLED;
  }

  const eSelectOp sel_op = eSelectOp(RNA_enum_get(op->ptr, "mode"));
  bool changed = false;

  if (SEL_OP_USE_PRE_DESELECT(sel_op)) {
    changed |= SEQ_retiming_selection_clear(ed);
  }

  rctf rectf;
  WM_operator_properties_border_to_rctf(op, &rectf);
  UI_view2d_region_to_view_rctf(v2d, &rectf, &rectf);

  for (const Sequence *seq : sequencer_visible_strips_get(C)) {
    for (const SeqRetimingKey &key : SEQ_retiming_keys_get(seq)) {
      if (seq->machine < rectf.ymin || seq->machine > rectf.ymax) {
        continue;
      }
      const int key_frame = SEQ_retiming_key_timeline_frame_get(scene, seq, &key);
      const int strip_start = SEQ_time_left_handle_frame_get(scene, seq);
      const int strip_end = SEQ_time_right_handle_frame_get(scene, seq);
      if (key_frame <= strip_start || key_frame > strip_end) {
        continue;
      }
      if (key_frame > rectf.xmax || key_frame < rectf.xmin) {
        continue;
      }

      switch (sel_op) {
        case SEL_OP_ADD:
        case SEL_OP_SET: {
          SEQ_retiming_selection_append(ed, seq, &key);
          break;
        }
        case SEL_OP_SUB: {
          SEQ_retiming_selection_remove(ed, seq, &key);
          break;
        }
        case SEL_OP_XOR: { /* Toggle */
          if (SEQ_retiming_selection_contains(ed, seq, &key)) {
            SEQ_retiming_selection_remove(ed, seq, &key);
          }
          else {
            SEQ_retiming_selection_append(ed, seq, &key);
          }
          break;
        }
      }

      changed = true;
    }
  }

  return changed ? OPERATOR_FINISHED : OPERATOR_CANCELLED;
}

static int sequencer_retiming_box_select_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  if (RNA_boolean_get(op->ptr, "tweak") &&
      retiming_mousover_key_get(C, event->mval, nullptr) != nullptr)
  {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  return WM_gesture_box_invoke(C, op, event);
}

void SEQUENCER_OT_retiming_select_box(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Box Select";
  ot->idname = "SEQUENCER_OT_retiming_select_box";
  ot->description = "Select strips using box selection";

  /* Api callbacks. */
  ot->invoke = sequencer_retiming_box_select_invoke;
  ot->exec = sequencer_retiming_box_select_exec;
  ot->modal = WM_gesture_box_modal;
  ot->cancel = WM_gesture_box_cancel;

  ot->poll = retiming_poll;

  /* Flags. */
  ot->flag = OPTYPE_UNDO;

  /* Properties. */
  WM_operator_properties_gesture_box(ot);
  WM_operator_properties_select_operation_simple(ot);
  RNA_def_boolean(
      ot->srna, "tweak", 0, "Tweak", "Operator has been activated using a click-drag event");
}

/** \} */
