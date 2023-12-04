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
#include "BLI_string.h"

#include "BKE_appdir.h"
#include "BKE_blender_copybuffer.h"
#include "BKE_blendfile.h"
#include "BKE_context.hh"
#include "BKE_fcurve.h"
#include "BKE_lib_id.h"
#include "BKE_lib_query.h"
#include "BKE_lib_remap.hh"
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

static int gather_strip_data_ids_to_null(LibraryIDLinkCallbackData *cb_data)
{
  IDRemapper *id_remapper = static_cast<IDRemapper *>(cb_data->user_data);
  ID *id = *cb_data->id_pointer;

  /* We don't care about embedded, loopback, or internal IDs. */
  if (cb_data->cb_flag & (IDWALK_CB_EMBEDDED | IDWALK_CB_LOOPBACK | IDWALK_CB_INTERNAL)) {
    return IDWALK_RET_NOP;
  }

  if (id) {
    ID_Type id_type = GS((id)->name);
    /* Nullify everything that is not:
     * Sound, Movieclip, Image, Text, Vfont, Action, or Collection IDs.
     */
    if (!ELEM(id_type, ID_SO, ID_MC, ID_IM, ID_TXT, ID_VF, ID_AC, ID_GR)) {
      BKE_id_remapper_add(id_remapper, id, nullptr);
      return IDWALK_RET_NOP;
    }
  }
  return IDWALK_RET_NOP;
}

static void sequencer_copy_animation_listbase(Scene *scene_src,
                                              Sequence *seq_dst,
                                              ListBase *clipboard_dst,
                                              ListBase *fcurve_base_src)
{
  /* Add curves for strips inside meta strip. */
  if (seq_dst->type == SEQ_TYPE_META) {
    LISTBASE_FOREACH (Sequence *, meta_child, &seq_dst->seqbase) {
      sequencer_copy_animation_listbase(scene_src, meta_child, clipboard_dst, fcurve_base_src);
    }
  }

  GSet *fcurves_src = SEQ_fcurves_by_strip_get(seq_dst, fcurve_base_src);
  if (fcurves_src == nullptr) {
    return;
  }

  GSET_FOREACH_BEGIN (FCurve *, fcu_src, fcurves_src) {
    BLI_addtail(clipboard_dst, BKE_fcurve_copy(fcu_src));
  }
  GSET_FOREACH_END();

  BLI_gset_free(fcurves_src, nullptr);
}

static void sequencer_copy_animation(Scene *scene_src,
                                     ListBase *fcurves_dst,
                                     ListBase *drivers_dst,
                                     Sequence *seq_dst)
{
  if (SEQ_animation_curves_exist(scene_src)) {
    sequencer_copy_animation_listbase(
        scene_src, seq_dst, fcurves_dst, &scene_src->adt->action->curves);
  }
  if (SEQ_animation_drivers_exist(scene_src)) {
    sequencer_copy_animation_listbase(scene_src, seq_dst, drivers_dst, &scene_src->adt->drivers);
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

  /* Create a temporary scene that we will copy from.
   * This is needed as it is the scene that contains all the VSE strip data.
   */
  scene_dst->ed = MEM_cnew<Editing>(__func__);
  scene_dst->ed->seqbasep = &scene_dst->ed->seqbase;
  SEQ_sequence_base_dupli_recursive(
      scene_src, scene_dst, &scene_dst->ed->seqbase, &scene_src->ed->seqbase, 0, 0);

  BLI_duplicatelist(&scene_dst->ed->channels, &scene_src->ed->channels);
  scene_dst->ed->displayed_channels = &scene_dst->ed->channels;

  /* Save current frame and active strip. */
  scene_dst->r.cfra = scene_src->r.cfra;
  Sequence *active_seq_src = SEQ_select_active_get(scene_src);
  if (active_seq_src) {
    Sequence *seq_dst = static_cast<Sequence *>(
        BLI_findstring(&scene_dst->ed->seqbase, active_seq_src->name, offsetof(Sequence, name)));
    if (seq_dst) {
      SEQ_select_active_set(scene_dst, seq_dst);
    }
  }

  ListBase fcurves_dst = {nullptr, nullptr};
  ListBase drivers_dst = {nullptr, nullptr};
  LISTBASE_FOREACH (Sequence *, seq_dst, &scene_dst->ed->seqbase) {
    /* Copy animation curves from seq_dst (if any). */
    sequencer_copy_animation(scene_src, &fcurves_dst, &drivers_dst, seq_dst);
  }

  if (!BLI_listbase_is_empty(&fcurves_dst) || !BLI_listbase_is_empty(&drivers_dst)) {
    BLI_assert(scene_dst->adt == nullptr);
    bAction *act_dst = ED_id_action_ensure(bmain_src, &scene_dst->id);
    BLI_movelisttolist(&act_dst->curves, &fcurves_dst);
    BLI_movelisttolist(&scene_dst->adt->drivers, &drivers_dst);
  }

  /* Nullify all ID pointers that we don't want to copy. For example, we don't want
   * to copy whole scenes. We have to come up with a proper idea of how to copy and
   * paste scene strips.
   */
  IDRemapper *id_remapper = BKE_id_remapper_create();
  BKE_library_foreach_ID_link(
      bmain_src, &scene_dst->id, gather_strip_data_ids_to_null, id_remapper, IDWALK_RECURSE);

  BKE_libblock_remap_multiple(bmain_src, id_remapper, 0);
  BKE_id_remapper_free(id_remapper);

  /* Ensure that there are no old copy tags around */
  BKE_blendfile_write_partial_begin(bmain_src);
  /* Tag the scene copy so we can pull in all scrip deps */
  BKE_copybuffer_copy_tag_ID(&scene_dst->id);
  /* Create the copy/paste temp file */
  bool retval = BKE_copybuffer_copy_end(bmain_src, filepath, reports);

  /* Clean up the action ID if we created any. */
  if (scene_dst->adt != nullptr && scene_dst->adt->action != nullptr) {
    BKE_id_delete(bmain_src, scene_dst->adt->action);
  }

  /* Cleanup the dummy scene file */
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

struct SEQPasteData {
  Main *bmain_dst;
  IDRemapper *id_remapper;
  std::vector<ID *> ids_to_copy;
};

enum {
  ID_SIMPLE_COPY = 0,
  ID_LIB_COPY,
  ID_REMAP,
};

static int should_copy_id(ID *id_dst, ID *id_src, Main *bmain_dst)
{
  if (!id_dst) {
    /* No local id found, we need to copy over this id. */
    if (id_src->lib == nullptr) {
      return ID_SIMPLE_COPY;
    }
    /* We will remap the pointer to null in this case. */
    return ID_LIB_COPY;
  }

  if (id_dst->lib == nullptr && id_src->lib == nullptr) {
    /* The local ID is the one we want to remap to, don't copy in a new one. */
    return ID_REMAP;
  }

  if (id_dst->lib == nullptr && id_src->lib != nullptr) {
    /* Check that the lib id doesn't point to the current file.
     * If this is the case, then it means that we want to simply remap as it points to the ID in
     * this file already.
     */
    if (STREQ(id_src->lib->filepath_abs, bmain_dst->filepath)) {
      return ID_REMAP;
    }
    return ID_LIB_COPY;
  }

  /* Check if they are using the same filepath. */
  if (STREQ(id_dst->lib->filepath_abs, id_src->lib->filepath_abs)) {
    return ID_REMAP;
  }

  return ID_LIB_COPY;
}

/**
 * Re-map ID's from the clipboard to ID's in `bmain`, by name.
 * If the ID name doesn't already exist, it is pasted in.
 */
static int paste_strips_data_ids_reuse_or_add(LibraryIDLinkCallbackData *cb_data)
{
  SEQPasteData *paste_data = static_cast<SEQPasteData *>(cb_data->user_data);
  Main *bmain_dst = paste_data->bmain_dst;
  ID *id_src = *cb_data->id_pointer;

  /* We don't care about embedded, loopback, or internal IDs. */
  if (cb_data->cb_flag & (IDWALK_CB_EMBEDDED | IDWALK_CB_LOOPBACK | IDWALK_CB_INTERNAL)) {
    return IDWALK_RET_NOP;
  }

  if (id_src) {
    ID_Type id_src_type = GS((id_src)->name);
    if (id_src_type == ID_AC || id_src_type == ID_GR) {
      /* Don't copy in actions here as we already handle these in "sequencer_paste_animation".
       * We don't copy Collections (ID_GR) here either as we don't care about scene collections.
       */
      return IDWALK_RET_NOP;
    }

    ID *id_dst = nullptr;
    int copy_method = -1;

    ListBase *lb = which_libbase(bmain_dst, id_src_type);
    LISTBASE_FOREACH (ID *, id_iter, lb) {
      if (STREQ(id_src->name, id_iter->name)) {
        int copy_method_iter = should_copy_id(id_iter, id_src, bmain_dst);
        if (copy_method_iter > copy_method) {
          copy_method = copy_method_iter;
          id_dst = id_iter;
        }
      }
    }

    if (id_dst == nullptr) {
      copy_method = should_copy_id(id_dst, id_src, bmain_dst);
    }

    switch (copy_method) {
      case ID_SIMPLE_COPY:
        paste_data->ids_to_copy.push_back(id_src);
        break;
      case ID_REMAP:
        BKE_id_remapper_add(paste_data->id_remapper, id_src, id_dst);
        break;
      case ID_LIB_COPY:
        paste_data->ids_to_copy.push_back(&id_src->lib->id);
        paste_data->ids_to_copy.push_back(id_src);
        break;
      default:
        BLI_assert_unreachable();
        break;
    }
  }
  return IDWALK_RET_NOP;
}

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
  Main *temp_bmain = BKE_main_new();
  STRNCPY(temp_bmain->filepath, BKE_main_blendfile_path_from_global());

  if (!BKE_copybuffer_read(temp_bmain, filepath, op->reports, 0)) {
    BKE_report(op->reports, RPT_INFO, "No data to paste");
    BKE_main_free(temp_bmain);
    return OPERATOR_CANCELLED;
  }

  Scene *paste_scene = nullptr;
  /* Find the scene we pasted that contains the strips. It should be tagged. */
  LISTBASE_FOREACH (Scene *, scene_iter, &temp_bmain->scenes) {
    if (scene_iter->id.flag & LIB_CLIPBOARD_MARK) {
      paste_scene = scene_iter;
      break;
    }
  }

  if (!paste_scene || !paste_scene->ed) {
    BKE_report(op->reports, RPT_ERROR, "No clipboard scene to paste VSE data from");
    BKE_main_free(temp_bmain);
    return OPERATOR_CANCELLED;
  }

  const int num_strips_to_paste = BLI_listbase_count(&paste_scene->ed->seqbase);
  if (num_strips_to_paste == 0) {
    BKE_report(op->reports, RPT_INFO, "No strips to paste");
    BKE_main_free(temp_bmain);
    return OPERATOR_CANCELLED;
  }

  Main *bmain_dst = CTX_data_main(C);
  SEQPasteData paste_data;
  paste_data.bmain_dst = bmain_dst;
  paste_data.id_remapper = BKE_id_remapper_create();
  BKE_library_foreach_ID_link(temp_bmain,
                              &paste_scene->id,
                              paste_strips_data_ids_reuse_or_add,
                              &paste_data,
                              IDWALK_RECURSE);

  /* Copy over all new ID data, save remapping for after we have moved over all the strips into
   * bmain.
   */
  for (ID *id_to_copy : paste_data.ids_to_copy) {
    BKE_libblock_management_main_remove(temp_bmain, id_to_copy);
    BKE_libblock_management_main_add(bmain_dst, id_to_copy);
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
      paste_scene, scene, &nseqbase, &paste_scene->ed->seqbase, 0, LIB_ID_CREATE_NO_USER_REFCOUNT);

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

  BKE_libblock_remap_multiple(bmain_dst, paste_data.id_remapper, 0);
  BKE_id_remapper_free(paste_data.id_remapper);

  BKE_main_free(temp_bmain);

  SEQ_animation_restore_original(scene, &animation_backup);

  DEG_id_tag_update(&scene->id, ID_RECALC_SEQUENCER_STRIPS);
  DEG_relations_tag_update(bmain_dst);
  WM_event_add_notifier(C, NC_SCENE | ND_SEQUENCER, scene);
  ED_outliner_select_sync_from_sequence_tag(C);

  BKE_reportf(op->reports, RPT_INFO, "%d strips pasted", num_strips_to_paste);

  return OPERATOR_FINISHED;
}
