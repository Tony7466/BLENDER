/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

#include "BKE_main.hh"
#include "BLI_fileops.h"
#include "BLI_map.hh"
#include "BLI_path_util.h"
#include "DNA_scene_types.h"
#include "DNA_sequence_types.h"

namespace blender::ed::seq {

static bool check_media_missing(const Sequence *seq)
{
  if (seq == nullptr || seq->strip == nullptr) {
    return false;
  }

  /* For strips that have a path, check their existence. */
  if (SEQ_HAS_PATH(seq)) {
    const StripElem *elem = seq->strip->stripdata;
    if (elem != nullptr) {
      int paths_count = 1;
      if (seq->type == SEQ_TYPE_IMAGE) {
        /* Image strip has array of file names. */
        paths_count = int(MEM_allocN_len(elem) / sizeof(*elem));
      }
      char filepath[FILE_MAX];
      const char *basepath = seq->scene ? ID_BLEND_PATH_FROM_GLOBAL(&seq->scene->id) :
                                          BKE_main_blendfile_path_from_global();
      for (int i = 0; i < paths_count; i++, elem++) {
        BLI_path_join(filepath, sizeof(filepath), seq->strip->dirpath, elem->filename);
        BLI_path_abs(filepath, basepath);
        if (!BLI_exists(filepath)) {
          return true;
        }
      }
    }
  }

  /* Recurse into meta strips. */
  if (seq->type == SEQ_TYPE_META) {
    LISTBASE_FOREACH (Sequence *, seqn, &seq->seqbase) {
      if (check_media_missing(seqn)) {
        return true;
      }
    }
  }

  /* Nothing is missing. */
  return false;
}

struct MediaPresence {
  Map<const Sequence *, bool> map;
};

bool media_presence_is_missing(MediaPresence **presence, const Sequence *seq)
{
  if (*presence == nullptr) {
    *presence = MEM_new<MediaPresence>(__func__);
  }
  const bool *val = (*presence)->map.lookup_ptr(seq);
  if (val != nullptr) {
    return *val;
  }

  bool missing = check_media_missing(seq);
  (*presence)->map.add_new(seq, missing);
  return missing;
}

void media_presence_invalidate_strip(MediaPresence *presence, const Sequence *seq)
{
  if (presence != nullptr) {
    presence->map.remove(seq);
  }
}

void media_presence_free(MediaPresence **presence)
{
  if (*presence != nullptr) {
    MEM_delete(*presence);
    *presence = nullptr;
  }
}

void media_presence_free_all(Main *bmain)
{
  LISTBASE_FOREACH (Scene *, scene, &bmain->scenes) {
    if (scene->ed != nullptr) {
      media_presence_free(&scene->ed->runtime.media_presence);
    }
  }
}

}  // namespace blender::ed::seq
