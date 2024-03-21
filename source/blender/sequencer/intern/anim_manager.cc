/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

#include "BLI_blenlib.h"
#include "BLI_hash_mm3.hh"
#include "BLI_index_range.hh"
#include "BLI_math_base.h"
#include "BLI_task.hh"
#include "BLI_timeit.hh"

#include "BKE_image.h"
#include "BKE_main.hh"
#include "BKE_scene.hh"

#include "DNA_scene_types.h"
#include "DNA_sequence_types.h"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "SEQ_iterator.hh"
#include "SEQ_relations.hh"
#include "SEQ_sequencer.hh"
#include "SEQ_utils.hh"

#include "anim_manager.hh"
#include "multiview.hh"
#include "proxy.hh"
#include "render.hh"
#include "strip_time.hh"

#define PREFETCH_DIST 512

static void anim_filepath_get(const Scene *scene,
                              Sequence *seq,
                              size_t filepath_size,
                              char *r_filepath)
{
  BLI_path_join(r_filepath, filepath_size, seq->strip->dirpath, seq->strip->stripdata->filename);
  BLI_path_abs(r_filepath, ID_BLEND_PATH_FROM_GLOBAL(&scene->id));
}

static bool use_proxy(Editing *ed, Sequence *seq)
{
  StripProxy *proxy = seq->strip->proxy;
  return proxy && ((proxy->storage & SEQ_STORAGE_PROXY_CUSTOM_DIR) != 0 ||
                   (ed->proxy_storage == SEQ_EDIT_PROXY_DIR_STORAGE));
}

static void proxy_dir_get(Editing *ed, Sequence *seq, size_t str_len, char *r_proxy_dirpath)
{
  if (use_proxy(ed, seq)) {
    if (ed->proxy_storage == SEQ_EDIT_PROXY_DIR_STORAGE) {
      if (ed->proxy_dir[0] == 0) {
        BLI_strncpy(r_proxy_dirpath, "//BL_proxy", str_len);
      }
      else {
        BLI_strncpy(r_proxy_dirpath, ed->proxy_dir, str_len);
      }
    }
    else {
      BLI_strncpy(r_proxy_dirpath, seq->strip->proxy->dirpath, str_len);
    }
    BLI_path_abs(r_proxy_dirpath, BKE_main_blendfile_path_from_global());
  }
}

static void index_dir_set(Editing *ed, Sequence *seq, StripAnim *sanim)
{
  if (sanim->anim == nullptr || !use_proxy(ed, seq)) {
    return;
  }

  char proxy_dirpath[FILE_MAX];
  proxy_dir_get(ed, seq, sizeof(proxy_dirpath), proxy_dirpath);
  seq_proxy_index_dir_set(sanim->anim, proxy_dirpath);
}

static ImBufAnim *anim_get(Sequence *seq, const char *filepath, bool openfile)
{
  ImBufAnim *anim = nullptr;

  if (openfile) {
    anim = openanim(filepath,
                    IB_rect | ((seq->flag & SEQ_FILTERY) ? IB_animdeinterlace : 0),
                    seq->streamindex,
                    seq->strip->colorspace_settings.name);
  }
  else {
    anim = openanim_noload(filepath,
                           IB_rect | ((seq->flag & SEQ_FILTERY) ? IB_animdeinterlace : 0),
                           seq->streamindex,
                           seq->strip->colorspace_settings.name);
  }

  return anim;
}

static bool is_multiview(const Scene *scene, Sequence *seq, const char *filepath)
{
  bool use_multiview = (seq->flag & SEQ_USE_VIEWS) != 0 && (scene->r.scemode & R_MULTIVIEW) != 0;
  char prefix[FILE_MAX];
  const char *ext = nullptr;
  BKE_scene_multiview_view_prefix_get(const_cast<Scene *>(scene), filepath, prefix, &ext);

  return use_multiview && seq->views_format == R_IMF_VIEWS_INDIVIDUAL && prefix[0] != '\0';
}

static blender::Vector<ImBufAnim *> multiview_anims_get(const Scene *scene,
                                                        Sequence *seq,
                                                        const char *filepath)
{
  int totfiles = seq_num_files(scene, seq->views_format, true);
  char prefix[FILE_MAX];
  const char *ext = nullptr;
  BKE_scene_multiview_view_prefix_get(const_cast<Scene *>(scene), filepath, prefix, &ext);
  blender::Vector<ImBufAnim *> anims;

  for (int i = 0; i < totfiles; i++) {
    const char *suffix = BKE_scene_multiview_view_id_suffix_get(&scene->r, i);
    char filepath_view[FILE_MAX];
    SNPRINTF(filepath_view, "%s%s%s", prefix, suffix, ext);

    /* Multiview files must be loaded, otherwise it is not possible to detect failure. */
    ImBufAnim *anim = anim_get(seq, filepath_view, true);
    if (anim != nullptr) {
      anims.append(anim);
    }
  }

  return anims;
}

void ShareableAnim::release_from_strip(Sequence *seq)
{
  if (anims.size() == 0 || BLI_listbase_is_empty(&seq->anims)) {
    return;
  }

  BLI_mutex_lock(mutex);

  LISTBASE_FOREACH (StripAnim *, sanim, &seq->anims) {
    MEM_freeN(sanim);
  }
  BLI_listbase_clear(&seq->anims);

  users.remove_if([seq](Sequence *seq_user) { return seq == seq_user; });

  if (users.size() == 0) {
    for (ImBufAnim *anim : anims) {
      IMB_free_anim(anim);
    }
    anims.clear();
  }
  BLI_mutex_unlock(mutex);
};

void ShareableAnim::release_from_all_strips(void)
{
  for (Sequence *user : users) {
    release_from_strip(user);
  }
}

void ShareableAnim::assign_to_strip(const Scene *scene, Sequence *seq)
{
  // XXX check this out
  // BLI_assert(BLI_listbase_is_empty(&seq->anims));
  Editing *ed = SEQ_editing_get(scene);

  for (int i = 0; i < anims.size(); i++) {
    ImBufAnim *anim = anims[i];
    StripAnim *sanim = static_cast<StripAnim *>(MEM_mallocN(sizeof(StripAnim), "Strip Anim"));
    sanim->anim = anim;
    BLI_addtail(&seq->anims, sanim);
    index_dir_set(ed, seq, sanim);
    if (is_multiview(scene, seq, filepath)) {
      const char *suffix = BKE_scene_multiview_view_id_suffix_get(&scene->r, i);
      IMB_suffix_anim(sanim->anim, suffix);
    }
  }

  users.append(seq);
};

// check if needed.
ThreadMutex acq_anim_mutex = BLI_MUTEX_INITIALIZER;

void ShareableAnim::acquire_anims(const Scene *scene, Sequence *seq, bool openfile)
{
  if (is_multiview(scene, seq, filepath)) {
    anims = multiview_anims_get(scene, seq, filepath);
    multiview_loaded = true;
    return;
  }

  BLI_mutex_lock(&acq_anim_mutex);
  ImBufAnim *anim = anim_get(seq, filepath, openfile);
  BLI_mutex_unlock(&acq_anim_mutex);
  if (anim != nullptr) {
    anims.append(anim);
  }
}

bool ShareableAnim::has_anim(const Scene *scene, Sequence *seq)
{
  if (is_multiview(scene, seq, filepath) && !multiview_loaded) {
    return false;
  }

  return !anims.is_empty();
}

ShareableAnim::ShareableAnim()
{
  mutex = BLI_mutex_alloc();
}

static ShareableAnim &anim_lookup_by_filepath(Editing *ed, const char *filepath)
{
  blender::Map<std::string, ShareableAnim> &anims = ed->runtime.anim_lookup->anims;
  BLI_mutex_lock(ed->runtime.anim_lookup->mutex);
  // XXX How does this free sh_anim.anims???
  ShareableAnim &sh_anim = anims.lookup_or_add_default(std::string(filepath));
  memcpy(sh_anim.filepath, filepath, sizeof(sh_anim.filepath));
  BLI_mutex_unlock(ed->runtime.anim_lookup->mutex);
  return sh_anim;
}

static ShareableAnim &anim_lookup_by_seq(const Scene *scene, Sequence *seq)
{
  Editing *ed = SEQ_editing_get(scene);
  seq_anim_lookup_ensure(ed);
  char filepath[FILE_MAX];
  anim_filepath_get(scene, seq, sizeof(filepath), filepath);
  ShareableAnim &sh_anim = anim_lookup_by_filepath(ed, filepath);
  return sh_anim;
}

void seq_open_anim_file(const Scene *scene, Sequence *seq, bool openfile)
{
  if ((seq->anims.first != nullptr) && (((StripAnim *)seq->anims.first)->anim != nullptr)) {
    return;
  }

  Editing *ed = SEQ_editing_get(scene);
  seq_anim_lookup_ensure(ed);
  char filepath[FILE_MAX];
  anim_filepath_get(scene, seq, sizeof(filepath), filepath);
  ShareableAnim &sh_anim = anim_lookup_by_filepath(ed, filepath);

  BLI_mutex_lock(sh_anim.mutex);
  if (!sh_anim.has_anim(scene, seq)) {
    sh_anim.acquire_anims(scene, seq, openfile);
  }
  sh_anim.assign_to_strip(scene, seq);
  BLI_mutex_unlock(sh_anim.mutex);
}

void SEQ_relations_sequence_free_anim(const Scene *scene, Sequence *seq)
{
  ShareableAnim &sh_anim = anim_lookup_by_seq(scene, seq);
  sh_anim.release_from_strip(seq);
}

static blender::Vector<Sequence *> strips_to_prefetch_get(const Scene *scene)
{
  Editing *ed = SEQ_editing_get(scene);
  ListBase *seqbase = SEQ_active_seqbase_get(ed);
  blender::VectorSet<Sequence *> strips = SEQ_query_all_strips_recursive(seqbase);
  const int timeline_frame = BKE_scene_frame_get(scene);
  strips.remove_if([scene, timeline_frame](Sequence *seq) {
    if (seq->type != SEQ_TYPE_MOVIE) {
      return true;
    }

    if (seq_time_distance_from_frame(scene, seq, timeline_frame) > PREFETCH_DIST) {
      return true;
    }
    return false;
  });

  blender::Vector<Sequence *> strips_2;

  for (Sequence *seq : strips) {
    strips_2.append(seq);
  }
  // XXX is VectorSet not sortable?
  std::sort(strips_2.begin(), strips_2.end(), [&](const Sequence *a, const Sequence *b) {
    return seq_time_distance_from_frame(scene, a, timeline_frame) <
           seq_time_distance_from_frame(scene, b, timeline_frame);
  });

  return strips_2;
}

static void free_unused_anims(const Editing *ed, blender::Vector<Sequence *> &strips)
{
  seq_render_mutex_lock();
  blender::Map<std::string, ShareableAnim> &anims = ed->runtime.anim_lookup->anims;
  for (ShareableAnim &sh_anim : anims.values()) {
    bool strips_use_anim = false;
    for (Sequence *user : sh_anim.users) {
      if (strips.contains(user)) {
        strips_use_anim = true;
        break;
      }
    }

    // TODO has users func? could be nicer anyway...
    if (!strips_use_anim && sh_anim.users.size() > 0) {
      sh_anim.release_from_all_strips();
    }
  }
  seq_render_mutex_unlock();
}

/* This function single-handedly manages all* anims VSE could ever need.
 * Well, not all, but all for playback purposes, not counting proxies, which is TODO...
 */
static void *manage_anims_thread(void *data)
{
  const Scene *scene = static_cast<const Scene *>(data);
  Editing *ed = SEQ_editing_get(scene);
  blender::Vector<Sequence *> strips = strips_to_prefetch_get(scene);

  free_unused_anims(ed, strips);

  // TODO why is this not working?
  /*using namespace blender;
  threading::parallel_for(strips.index_range(), 1, [&](const IndexRange range) {
    for (int i : range) {
      Sequence *seq = strips[i];
      seq_open_anim_file(scene, seq, true);
    }
  });*/

  for (Sequence *seq : strips) {
    seq_open_anim_file(scene, seq, true);
  }

  ed->runtime.anim_lookup->working = false;
  return 0;
}

void seq_anim_lookup_ensure(Editing *ed)
{
  if (ed->runtime.anim_lookup == nullptr) {
    ed->runtime.anim_lookup = MEM_new<AnimManager>(__func__);
  }
}

AnimManager::AnimManager()
{
  BLI_threadpool_init(&threads, manage_anims_thread, 1);
  mutex = BLI_mutex_alloc();
}

void AnimManager::manage_anims(Scene *scene)
{
  if (working) {
    return;
  }

  working = true;
  BLI_threadpool_clear(&threads);
  BLI_threadpool_insert(&threads, static_cast<void *>(scene));
}

void AnimManager::load_set(const Scene *scene, blender::Vector<Sequence *> &strips)
{

  blender::Map<std::string, ShareableAnim> &anims = scene->ed->runtime.anim_lookup->anims;
  anims.reserve(strips.size() * 50);

  using namespace blender;
  threading::parallel_for(strips.index_range(), 1, [&](const IndexRange range) {
    for (int i : range) {
      Sequence *seq = strips[i];
      seq_open_anim_file(scene, seq, true);
    }
  });
}
