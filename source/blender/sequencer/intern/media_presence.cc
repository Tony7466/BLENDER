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
#include "BLI_string.h"
#include "DNA_scene_types.h"
#include "DNA_sequence_types.h"
#include "DNA_sound_types.h"

namespace blender::seq {

static const char *get_seq_base_path(const Sequence *seq)
{
  return seq->scene ? ID_BLEND_PATH_FROM_GLOBAL(&seq->scene->id) :
                      BKE_main_blendfile_path_from_global();
}

static bool check_sound_media_missing(const bSound *sound, const Sequence *seq)
{
  if (sound == nullptr) {
    return false;
  }

  char filepath[FILE_MAX];
  STRNCPY(filepath, sound->filepath);
  const char *basepath = get_seq_base_path(seq);
  BLI_path_abs(filepath, basepath);
  return !BLI_exists(filepath);
}

static bool check_media_missing(const Sequence *seq)
{
  if (seq == nullptr || seq->strip == nullptr) {
    return false;
  }

  /* Images or movies. */
  if (ELEM((seq)->type, SEQ_TYPE_MOVIE, SEQ_TYPE_IMAGE)) {
    const StripElem *elem = seq->strip->stripdata;
    if (elem != nullptr) {
      int paths_count = 1;
      if (seq->type == SEQ_TYPE_IMAGE) {
        /* Image strip has array of file names. */
        paths_count = int(MEM_allocN_len(elem) / sizeof(*elem));
      }
      char filepath[FILE_MAX];
      const char *basepath = get_seq_base_path(seq);
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
  Map<const void *, bool> map_seq;
  Map<const bSound *, bool> map_sound;
};

bool media_presence_is_missing(Scene *scene, const Sequence *seq)
{
  if (seq == nullptr || scene == nullptr || scene->ed == nullptr) {
    return false;
  }

  MediaPresence **presence = &scene->ed->runtime.media_presence;
  if (*presence == nullptr) {
    *presence = MEM_new<MediaPresence>(__func__);
  }

  /* Strips that reference another data block that has path to media
   * (e.g. sound strips) need to key the presence cache on that data
   * block. Since it can be used by multiple strips. */
  if (seq->type == SEQ_TYPE_SOUND_RAM) {
    const bSound *sound = seq->sound;
    const bool *val = (*presence)->map_sound.lookup_ptr(sound);
    if (val != nullptr) {
      return *val;
    }

    bool missing = check_sound_media_missing(sound, seq);
    (*presence)->map_sound.add_new(sound, missing);
    return missing;
  }

  /* Regular strips that point to media directly. */
  const bool *val = (*presence)->map_seq.lookup_ptr(seq);
  if (val != nullptr) {
    return *val;
  }

  bool missing = check_media_missing(seq);
  (*presence)->map_seq.add_new(seq, missing);
  return missing;
}

void media_presence_invalidate_strip(Scene *scene, const Sequence *seq)
{
  if (scene != nullptr && scene->ed != nullptr && scene->ed->runtime.media_presence != nullptr) {
    scene->ed->runtime.media_presence->map_seq.remove(seq);
  }
}

void media_presence_invalidate_sound(Scene *scene, const bSound *sound)
{
  if (scene != nullptr && scene->ed != nullptr && scene->ed->runtime.media_presence != nullptr) {
    scene->ed->runtime.media_presence->map_sound.remove(sound);
  }
}

void media_presence_free(Scene *scene)
{
  if (scene != nullptr && scene->ed != nullptr && scene->ed->runtime.media_presence != nullptr) {
    MEM_delete(scene->ed->runtime.media_presence);
    scene->ed->runtime.media_presence = nullptr;
  }
}

}  // namespace blender::seq
