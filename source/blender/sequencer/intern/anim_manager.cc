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

/* This is arbitrary, it is possible to prefetch n strips ahead, but if strips are too short, but
 * it may be better to prefetch frame range. */
#define PREFETCH_DIST 512

static void anim_filepath_get(const Scene *scene,
                              const Sequence *seq,
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

static void index_dir_set(Editing *ed, Sequence *seq, ImBufAnim *anim)
{
  if (!use_proxy(ed, seq)) {
    return;
  }

  char proxy_dirpath[FILE_MAX];
  proxy_dir_get(ed, seq, sizeof(proxy_dirpath), proxy_dirpath);
  seq_proxy_index_dir_set(anim, proxy_dirpath);
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
  if (anims.size() == 0) {
    return;
  }

  mutex->lock();

  users.remove_if([seq](Sequence *seq_user) { return seq == seq_user; });

  if (users.size() == 0) {
    for (ImBufAnim *anim : anims) {
      IMB_free_anim(anim);
    }
    anims.clear();
  }

  mutex->unlock();
};

void ShareableAnim::release_from_all_strips(void)
{
  for (Sequence *user : users) {
    release_from_strip(user);
  }
}

void ShareableAnim::acquire_anims(const Scene *scene, Sequence *seq, bool openfile)
{
  char filepath[FILE_MAX];
  anim_filepath_get(scene, seq, sizeof(filepath), filepath);

  if (is_multiview(scene, seq, filepath)) {
    anims = multiview_anims_get(scene, seq, filepath);
    multiview_loaded = true;
    return;
  }

  ImBufAnim *anim = anim_get(seq, filepath, openfile);
  if (anim != nullptr) {
    anims.append(anim);
  }

  for (int i = 0; i < anims.size(); i++) {
    index_dir_set(SEQ_editing_get(scene), seq, anims[i]);
    char filepath[FILE_MAX];
    anim_filepath_get(scene, seq, sizeof(filepath), filepath);
    if (is_multiview(scene, seq, filepath)) {
      const char *suffix = BKE_scene_multiview_view_id_suffix_get(&scene->r, i);
      IMB_suffix_anim(anims[i], suffix);
    }
  }

  users.append(seq);
}

bool ShareableAnim::has_anim(const Scene *scene, Sequence *seq)
{
  char filepath[FILE_MAX];
  anim_filepath_get(scene, seq, sizeof(filepath), filepath);

  if (is_multiview(scene, seq, filepath) && !multiview_loaded) {
    return false;
  }

  return !anims.is_empty();
}

bool ShareableAnim::try_lock()
{
  return mutex->try_lock();
}

void ShareableAnim::unlock()
{
  mutex->unlock();
}

void seq_open_anim_file(const Scene *scene, Sequence *seq, bool openfile)
{
  /// xxx
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

  blender::Vector<Sequence *> strips_sorted = strips.as_span();

  std::sort(strips_sorted.begin(), strips_sorted.end(), [&](const Sequence *a, const Sequence *b) {
    return seq_time_distance_from_frame(scene, a, timeline_frame) <
           seq_time_distance_from_frame(scene, b, timeline_frame);
  });

  return strips_sorted;
}

AnimManager *seq_anim_lookup_ensure(Editing *ed)
{
  if (ed->runtime.anim_lookup == nullptr) {
    ed->runtime.anim_lookup = MEM_new<AnimManager>(__func__);
  }
  return ed->runtime.anim_lookup;
}

void AnimManager::free_unused_anims(const Editing *ed, blender::Vector<Sequence *> &strips)
{
  mutex.lock();
  for (ShareableAnim &sh_anim : anims.values()) {
    bool strips_use_anim = false;
    for (Sequence *user : sh_anim.users) {
      if (strips.contains(user)) {
        strips_use_anim = true;
        break;
      }
    }

    if (!strips_use_anim && sh_anim.users.size() > 0) {
      sh_anim.release_from_all_strips();
    }
  }
  mutex.unlock();
}

void AnimManager::parallel_load_anims(const Scene *scene,
                                      blender::Vector<Sequence *> &strips,
                                      bool unlock)
{
  /* Ensure cache items. */
  // XXX why is this needed, when cache_entry_get is locking vector? if this is not done before
  // parallel for loop, it causes use after free in ShareableAnim::anims...
  for (Sequence *seq : strips) {
    if (seq->type != SEQ_TYPE_MOVIE) {
      continue;
    }
    cache_entry_get(scene, seq);
  }

  using namespace blender;
  threading::parallel_for(strips.index_range(), 1, [&](const IndexRange range) {
    for (int i : range) {
      Sequence *seq = strips[i];
      if (seq->type != SEQ_TYPE_MOVIE) {
        continue;
      }

      ShareableAnim &sh_anim = cache_entry_get(scene, seq);
      if (!sh_anim.mutex->try_lock()) {
        continue;
      }

      if (sh_anim.has_anim(scene, seq)) {
        if (unlock) {
          sh_anim.unlock();
        }
        continue;
      }

      sh_anim.acquire_anims(scene, seq, true);
      if (unlock) {
        sh_anim.unlock();
      }
    }
  });
}

void AnimManager::free_unused_and_prefetch_anims(const Scene *scene)
{
  Editing *ed = SEQ_editing_get(scene);
  blender::Vector<Sequence *> strips = strips_to_prefetch_get(scene);

  free_unused_anims(ed, strips);
  parallel_load_anims(scene, strips, true);
}

void AnimManager::manage_anims(const Scene *scene)
{
  if (prefetch_thread.joinable()) {
    prefetch_thread.join();
  }
  else {
    prefetch_thread = std::thread(&AnimManager::free_unused_and_prefetch_anims, this, scene);
  }
}

ShareableAnim &AnimManager::cache_entry_get(const Scene *scene, const Sequence *seq)
{
  char filepath[FILE_MAX];
  anim_filepath_get(scene, seq, sizeof(filepath), filepath);

  mutex.lock();
  ShareableAnim &sh_anim = anims.lookup_or_add_default(std::string(filepath));
  mutex.unlock();
  return sh_anim;
}

void AnimManager::strip_anims_laod_and_lock(const Scene *scene,
                                            blender::Vector<Sequence *> &strips)
{
  parallel_load_anims(scene, strips, false);
}

void AnimManager::strip_anims_unlock(const Scene *scene, blender::Vector<Sequence *> &strips)
{
  for (Sequence *seq : strips) {
    if (seq->type != SEQ_TYPE_MOVIE) {  // XXX this should not be needed?
      continue;
    }
    ShareableAnim &sh_anim = cache_entry_get(scene, seq);
    sh_anim.unlock();
  }
}

blender::Vector<ImBufAnim *> &AnimManager::strip_anims_get(const Scene *scene, const Sequence *seq)
{
  ShareableAnim &sh_anim = cache_entry_get(scene, seq);
  return sh_anim.anims;
}

void AnimManager::free_anims_by_seq(const Scene *scene, const Sequence *seq)
{
  ShareableAnim &sh_anim = cache_entry_get(scene, seq);
  sh_anim.release_from_all_strips();
}
