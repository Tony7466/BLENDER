/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 * SPDX-FileCopyrightText: 2003-2009 Blender Authors
 * SPDX-FileCopyrightText: 2005-2006 Peter Schlaile <peter [at] schlaile [dot] de>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <cstring>

#include "MEM_guardedalloc.h"

#include "ED_keyframing.hh"
#include "ED_outliner.hh"
#include "ED_sequencer.hh"

#include "DNA_anim_types.h"
#include "DNA_scene_types.h"
#include "DNA_sequence_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "BLI_ghash.h"
#include "BLI_listbase.h"
#include "BLI_path_util.h"

#include "BLO_writefile.hh"

#include "BKE_appdir.h"
#include "BKE_blender_copybuffer.h"
#include "BKE_blendfile.h"
#include "BKE_context.hh"
#include "BKE_fcurve.h"
#include "BKE_lib_id.h"
#include "BKE_main.hh"
#include "BKE_report.h"
#include "BKE_scene.h"

#include "RNA_access.hh"

#include "SEQ_animation.hh"
#include "SEQ_clipboard.hh"
#include "SEQ_select.hh"
#include "SEQ_sequencer.hh"
#include "SEQ_time.hh"
#include "SEQ_transform.hh"
#include "SEQ_utils.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#ifdef WITH_AUDASPACE
#  include <AUD_Special.h>
#endif

/* -------------------------------------------------------------------- */
/* Copy Operator Helper functions
 */
static void sequencer_copy_animation_listbase(Scene *scene,
                                              Sequence *seq,
                                              ListBase *clipboard,
                                              ListBase *fcurve_base)
{
  /* Add curves for strips inside meta strip. */
  if (seq->type == SEQ_TYPE_META) {
    LISTBASE_FOREACH (Sequence *, meta_child, &seq->seqbase) {
      sequencer_copy_animation_listbase(scene, meta_child, clipboard, fcurve_base);
    }
  }

  GSet *fcurves = SEQ_fcurves_by_strip_get(seq, fcurve_base);
  if (fcurves == nullptr) {
    return;
  }

  GSET_FOREACH_BEGIN (FCurve *, fcu, fcurves) {
    BLI_addtail(clipboard, BKE_fcurve_copy(fcu));
  }
  GSET_FOREACH_END();

  BLI_gset_free(fcurves, nullptr);
}

static void sequencer_copy_animation(Scene *scene_src,
                                     ListBase *copied_fcurves,
                                     ListBase *copied_drivers,
                                     Sequence *seq)
{
  if (SEQ_animation_curves_exist(scene_src)) {
    sequencer_copy_animation_listbase(
        scene_src, seq, copied_fcurves, &scene_src->adt->action->curves);
  }
  if (SEQ_animation_drivers_exist(scene_src)) {
    sequencer_copy_animation_listbase(scene_src, seq, copied_drivers, &scene_src->adt->drivers);
  }
}

static void sequencer_copybuffer_filepath_get(char filepath[FILE_MAX], size_t filepath_maxncpy)
{
  BLI_path_join(filepath, filepath_maxncpy, BKE_tempdir_base(), "copybuffer_vse.blend");
}

static bool sequencer_write_copy_paste_file(Main *bmain_src,
                                            Scene *scene_src,
                                            const char *filepath,
                                            ReportList *reports)

{
  Scene *scene_dst = BKE_scene_add(bmain_src, "copybuffer_vse_scene");

  // TODO this is taken from scene.cc. Perhaps create a generic "Copy scene->ed" function?
  scene_dst->ed = MEM_cnew<Editing>(__func__);
  scene_dst->ed->seqbasep = &scene_dst->ed->seqbase;
  const int flag_subdata = LIB_ID_CREATE_NO_USER_REFCOUNT;
  SEQ_sequence_base_dupli_recursive(
      scene_src, scene_dst, &scene_dst->ed->seqbase, &scene_src->ed->seqbase, 0, flag_subdata);

  BLI_duplicatelist(&scene_dst->ed->channels, &scene_src->ed->channels);
  scene_dst->ed->displayed_channels = &scene_dst->ed->channels;

  /* Save current frame and active strip. */
  scene_dst->r.cfra = scene_src->r.cfra;
  Sequence *prev_active_seq = SEQ_select_active_get(scene_src);
  if (prev_active_seq) {
    LISTBASE_FOREACH (Sequence *, seq, &scene_dst->ed->seqbase) {
      if (STREQ(seq->name, prev_active_seq->name)) {
        SEQ_select_active_set(scene_dst, seq);
      }
    }
  }

  ListBase fcurves_dst = {nullptr, nullptr};
  ListBase drivers_dst = {nullptr, nullptr};
  LISTBASE_FOREACH (Sequence *, seq, &scene_dst->ed->seqbase) {
    /* Copy animation curves from seq (if any). */
    sequencer_copy_animation(scene_src, &fcurves_dst, &drivers_dst, seq);
  }
  if (BLI_listbase_count(&fcurves_dst) != 0 || BLI_listbase_count(&drivers_dst) != 0) {
    bAction *act = ED_id_action_ensure(bmain_src, &scene_dst->id);
    BLI_movelisttolist(&act->curves, &fcurves_dst);
    BLI_movelisttolist(&scene_dst->adt->drivers, &drivers_dst);
  }

  /* Ensure that there are no old tags around */
  BKE_blendfile_write_partial_begin(bmain_src);
  /* Tag the scene copy so we can pull in all scrip deps */
  BKE_copybuffer_copy_tag_ID(&scene_dst->id);
  /* Create the copy/paste temp file */
  const int write_flags = 0;
  const eBLO_WritePathRemap remap_mode = BLO_WRITE_PATH_REMAP_RELATIVE;

  bool retval = BKE_blendfile_write_partial(bmain_src, filepath, write_flags, remap_mode, reports);

  BKE_blendfile_write_partial_end(bmain_src);
  BKE_id_delete(bmain_src, scene_dst);

  return retval;
}

int SEQ_clipboard_copy_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  Editing *ed = SEQ_editing_get(scene);

  if (SEQ_transform_seqbase_isolated_sel_check(ed->seqbasep) == false) {
    BKE_report(op->reports, RPT_ERROR, "Please select all related strips");
    return OPERATOR_CANCELLED;
  }

  char filepath[FILE_MAX];
  sequencer_copybuffer_filepath_get(filepath, sizeof(filepath));
  bool success = sequencer_write_copy_paste_file(bmain, scene, filepath, op->reports);
  if (!success) {
    BKE_report(op->reports, RPT_ERROR, "Could not create the copy paste file!");
    return OPERATOR_CANCELLED;
  }

  /* We are all done! */
  BKE_report(op->reports, RPT_INFO, "Copied the selected VSE strips to internal clipboard");
  return OPERATOR_FINISHED;
}

/* -------------------------------------------------------------------- */
/* Paste Operator Helper functions
 */

static void sequencer_paste_animation(bContext *C, Scene *paste_scene)
{
  if (!SEQ_animation_curves_exist(paste_scene) && !SEQ_animation_drivers_exist(paste_scene)) {
    return;
  }

  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  bAction *act;

  if (scene->adt != nullptr && scene->adt->action != nullptr) {
    act = scene->adt->action;
  }
  else {
    /* get action to add F-Curve+keyframe to */
    act = ED_id_action_ensure(bmain, &scene->id);
  }

  LISTBASE_FOREACH (FCurve *, fcu, &paste_scene->adt->action->curves) {
    BLI_addtail(&act->curves, BKE_fcurve_copy(fcu));
  }
  LISTBASE_FOREACH (FCurve *, fcu, &paste_scene->adt->drivers) {
    BLI_addtail(&scene->adt->drivers, BKE_fcurve_copy(fcu));
  }
}
int SEQ_clipboard_paste_exec(bContext *C, wmOperator *op)

{
  char filepath[FILE_MAX];
  sequencer_copybuffer_filepath_get(filepath, sizeof(filepath));
  Main *bmain = CTX_data_main(C);

  // TODO we probably want to use the "temp paste" logic that the asset system is
  // using that Julian talked about

  /* Make sure that we don't have any lingering copy tags from previous pastes. */
  BKE_blendfile_write_partial_begin(bmain);

  const int datablocks_pasted = BKE_copybuffer_paste(C, filepath, 0, op->reports, 0);
  if (datablocks_pasted == 0) {
    BKE_report(op->reports, RPT_INFO, "No data to paste");
    return OPERATOR_CANCELLED;
  }

  Scene *paste_scene = nullptr;
  /* Find the scene we pasted that contains the strips. It should be tagged. */
  LISTBASE_FOREACH (Scene *, scene_iter, &bmain->scenes) {
    if (scene_iter->id.flag & LIB_CLIPBOARD_MARK) {
      paste_scene = scene_iter;
      break;
    }
  }

  const int stripts_pasted = BLI_listbase_count(&paste_scene->ed->seqbase);
  if (!paste_scene || !paste_scene->ed || stripts_pasted == 0) {
    BKE_report(op->reports, RPT_INFO, "No strips to paste");
    return OPERATOR_CANCELLED;
  }

  Scene *scene = CTX_data_scene(C);
  Editing *ed = SEQ_editing_ensure(scene); /* Create if needed. */
  int ofs;

  ED_sequencer_deselect_all(scene);
  if (RNA_boolean_get(op->ptr, "keep_offset")) {
    ofs = scene->r.cfra - paste_scene->r.cfra;
  }
  else {
    int min_seq_startdisp = INT_MAX;
    LISTBASE_FOREACH (Sequence *, seq, &paste_scene->ed->seqbase) {
      if (SEQ_time_left_handle_frame_get(paste_scene, seq) < min_seq_startdisp) {
        min_seq_startdisp = SEQ_time_left_handle_frame_get(paste_scene, seq);
      }
    }
    /* Paste strips relative to the current-frame. */
    ofs = scene->r.cfra - min_seq_startdisp;
  }

  /* Paste animation.
   * NOTE: Only fcurves are copied. Drivers and NLA action strips are not copied.
   * First backup original curves from scene and move curves from clipboard into scene. This way,
   * when pasted strips are renamed, pasted fcurves are renamed with them. Finally restore original
   * curves from backup.
   */

  SeqAnimationBackup animation_backup = {{nullptr}};
  SEQ_animation_backup_original(scene, &animation_backup);
  sequencer_paste_animation(C, paste_scene);

  ListBase nseqbase = {nullptr, nullptr};
  SEQ_sequence_base_dupli_recursive(
      paste_scene, scene, &nseqbase, &paste_scene->ed->seqbase, 0, 0);

  Sequence *prev_active_seq = SEQ_select_active_get(paste_scene);

  /* NOTE: SEQ_sequence_base_dupli_recursive() takes care of generating
   * new UUIDs for sequences in the new list. */
  Sequence *iseq_first = static_cast<Sequence *>(nseqbase.first);
  BLI_movelisttolist(ed->seqbasep, &nseqbase);
  /* Restore "first" pointer as BLI_movelisttolist sets it to nullptr */
  nseqbase.first = iseq_first;

  LISTBASE_FOREACH (Sequence *, iseq, &nseqbase) {
    if (prev_active_seq && STREQ(iseq->name, prev_active_seq->name)) {
      SEQ_select_active_set(scene, iseq);
    }
    /* Make sure, that pasted strips have unique names. This has to be done after
     * adding strips to seqbase, for lookup cache to work correctly. */
    SEQ_ensure_unique_name(iseq, scene);
  }

  LISTBASE_FOREACH (Sequence *, iseq, &nseqbase) {
    /* Translate after name has been changed, otherwise this will affect animdata of original
     * strip. */
    SEQ_transform_translate_sequence(scene, iseq, ofs);
    /* Ensure, that pasted strips don't overlap. */
    if (SEQ_transform_test_overlap(scene, ed->seqbasep, iseq)) {
      SEQ_transform_seqbase_shuffle(ed->seqbasep, iseq, scene);
    }
  }

  BKE_id_delete(bmain, paste_scene);

  SEQ_animation_restore_original(scene, &animation_backup);

  DEG_id_tag_update(&scene->id, ID_RECALC_SEQUENCER_STRIPS);
  DEG_relations_tag_update(bmain);
  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
  ED_outliner_select_sync_from_sequence_tag(C);

  BKE_reportf(op->reports, RPT_INFO, "%d strips pasted", stripts_pasted);

  return OPERATOR_FINISHED;
}
