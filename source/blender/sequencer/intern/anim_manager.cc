/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

#include "BLI_blenlib.h"
#include "BLI_index_range.hh"
#include "BLI_task.hh"

#include "BKE_image.h"
#include "BKE_main.hh"
#include "BKE_scene.hh"

#include "DNA_scene_types.h"
#include "DNA_sequence_types.h"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "SEQ_anim_manager.hh"
#include "SEQ_iterator.hh"
#include "SEQ_relations.hh"
#include "SEQ_sequencer.hh"
#include "SEQ_utils.hh"

#include "multiview.hh"
#include "proxy.hh"
#include "render.hh"
#include "strip_time.hh"

namespace blender::seq {

/* This is arbitrary, it is possible to prefetch n strips ahead, but if strips are too short, but
 * it may be better to prefetch frame range. */
#define PREFETCH_DIST 512

static void anim_filepath_get(const Scene *scene,
                              const Sequence *seq,
                              size_t filepath_size,
                              char *r_filepath)
{
  if (seq->strip == nullptr || seq->strip->stripdata == nullptr) {
    return;
  }
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

static ImBufAnim *anim_get(Sequence *seq, const char *filepath)
{
  return openanim(filepath,
                  IB_rect | ((seq->flag & SEQ_FILTERY) ? IB_animdeinterlace : 0),
                  seq->streamindex,
                  seq->strip->colorspace_settings.name);
}

static bool is_multiview(const Scene *scene, Sequence *seq)
{
  char filepath[FILE_MAX];
  anim_filepath_get(scene, seq, sizeof(filepath), filepath);
  bool use_multiview = (seq->flag & SEQ_USE_VIEWS) != 0 && (scene->r.scemode & R_MULTIVIEW) != 0;
  char prefix[FILE_MAX];
  const char *ext = nullptr;
  BKE_scene_multiview_view_prefix_get(const_cast<Scene *>(scene), filepath, prefix, &ext);

  return use_multiview && seq->views_format == R_IMF_VIEWS_INDIVIDUAL && prefix[0] != '\0';
}

static blender::Vector<ImBufAnim *> multiview_anims_get(const Scene *scene,
                                                        Sequence *seq,
                                                        const char *filepath,
                                                        int start_file)
{
  int totfiles = seq_num_files(scene, seq->views_format, true);
  char prefix[FILE_MAX];
  const char *ext = nullptr;
  BKE_scene_multiview_view_prefix_get(const_cast<Scene *>(scene), filepath, prefix, &ext);
  blender::Vector<ImBufAnim *> anims;

  for (int i = start_file; i < totfiles; i++) {
    const char *suffix = BKE_scene_multiview_view_id_suffix_get(&scene->r, i);
    char filepath_view[FILE_MAX];
    SNPRINTF(filepath_view, "%s%s%s", prefix, suffix, ext);

    ImBufAnim *anim = anim_get(seq, filepath_view);
    if (anim != nullptr) {
      anims.append(anim);
    }
  }

  return anims;
}

void ShareableAnim::release_from_strip(Sequence *seq)
{
  if (this->anims.is_empty()) {
    return;
  }

  this->mutex.lock();

  this->users.remove_if([seq](Sequence *seq_user) { return seq == seq_user; });

  if (this->users.is_empty()) {
    for (ImBufAnim *anim : this->anims) {
      IMB_free_anim(anim);
    }
    this->anims.clear();
  }

  this->mutex.unlock();
};

void ShareableAnim::release_from_all_strips(void)
{
  for (Sequence *user : users) {
    this->release_from_strip(user);
  }
}

/* This function must check if all anims are loaded and possibly skip actual loading.
 * However it must always append users. */
void ShareableAnim::acquire_anims(const Scene *scene, Sequence *seq)
{
  char filepath[FILE_MAX];
  anim_filepath_get(scene, seq, sizeof(filepath), filepath);

  const int anims_needed = seq_num_files(scene, seq->views_format, is_multiview(scene, seq));
  const int anims_loaded = this->anims.size();

  if (anims_loaded >= anims_needed) {
    this->users.add(seq);
    return;
  }

  if (is_multiview(scene, seq)) {
    const blender::Vector<ImBufAnim *> new_anims = multiview_anims_get(
        scene, seq, filepath, anims_loaded);

    for (ImBufAnim *anim : new_anims) {
      this->anims.append(anim);
    }
  }
  else {
    ImBufAnim *anim = anim_get(seq, filepath);
    if (anim != nullptr) {
      this->anims.append(anim);
    }
  }

  for (const int i : this->anims.index_range()) {
    index_dir_set(SEQ_editing_get(scene), seq, this->anims[i]);
    if (is_multiview(scene, seq)) {
      const char *suffix = BKE_scene_multiview_view_id_suffix_get(&scene->r, i);
      IMB_suffix_anim(this->anims[i], suffix);
    }
  }

  this->users.add(seq);
}

void ShareableAnim::unlock()
{
  this->mutex.unlock();
}

/* TODO: It would be simpler, and perhaps better for user to load n strips, instead of
 * relying on distance from CFRA. */
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

void AnimManager::free_unused_anims(blender::Vector<Sequence *> &strips)
{
  this->mutex.lock();
  for (std::unique_ptr<ShareableAnim> &sh_anim : this->anims_map.values()) {
    bool strips_use_anim = false;
    for (Sequence *user : sh_anim->users) {
      if (strips.contains(user)) {
        strips_use_anim = true;
        break;
      }
    }

    if (!strips_use_anim && sh_anim->users.size() > 0) {
      sh_anim->release_from_all_strips();
    }
  }
  this->mutex.unlock();
}

/* The main purpose of this function is to create set of strips, which ensures, that all
 * multiview anims will be loaded.
 * The set does need to contain only one user(strip) of any particular anim(filepath), because
 * during parallel loading `ShareableAnim` is locked.
 */
static blender::Vector<Sequence *> remove_duplicates_for_parallel_load(
    const Scene *scene, blender::Vector<Sequence *> &strips)
{
  blender::Map<std::string, Sequence *> unique_strips_map;
  blender::Vector<Sequence *> unique_strips;

  /* Add strips with multiview configuration first. */
  for (Sequence *seq : strips) {
    char filepath[FILE_MAX];
    anim_filepath_get(scene, seq, sizeof(filepath), filepath);

    if (unique_strips_map.contains(std::string(filepath))) {
      continue;
    }

    if (is_multiview(scene, seq)) {
      unique_strips_map.add(std::string(filepath), seq);
    }
  }

  /* Add all other strips. */
  for (Sequence *seq : strips) {
    char filepath[FILE_MAX];
    anim_filepath_get(scene, seq, sizeof(filepath), filepath);

    if (unique_strips_map.contains(std::string(filepath))) {
      continue;
    }

    unique_strips_map.add(std::string(filepath), seq);
  }

  for (Sequence *seq : unique_strips_map.values()) {
    unique_strips.append(seq);
  }

  return unique_strips;
}

void AnimManager::parallel_load_anims(const Scene *scene,
                                      blender::Vector<Sequence *> &strips,
                                      bool keep_locked)
{
  using namespace blender;

  strips.remove_if([](Sequence *seq) { return seq->type != SEQ_TYPE_MOVIE; });
  strips = remove_duplicates_for_parallel_load(scene, strips);

  threading::parallel_for(strips.index_range(), 1, [&](const IndexRange range) {
    for (const int i : range) {
      Sequence *seq = strips[i];
      ShareableAnim &sh_anim = this->cache_entry_get(scene, seq);
      sh_anim.mutex.lock();

      sh_anim.acquire_anims(scene, seq);

      if (!keep_locked) {
        sh_anim.unlock();
      }
    }
  });
}

void AnimManager::free_unused_and_prefetch_anims(const Scene *scene)
{
  blender::Vector<Sequence *> strips = strips_to_prefetch_get(scene);

  /* Prefetch first, to avoid freeing anim and loading it again. */
  this->parallel_load_anims(scene, strips, false);
  this->free_unused_anims(strips);
}

void AnimManager::manage_anims(const Scene *scene)
{
  if (this->prefetch_thread.joinable()) {
    this->prefetch_thread.join();
  }
  else {
    this->prefetch_thread = std::thread(&AnimManager::free_unused_and_prefetch_anims, this, scene);
  }
}

ShareableAnim &AnimManager::cache_entry_get(const Scene *scene, const Sequence *seq)
{
  char filepath[FILE_MAX];
  anim_filepath_get(scene, seq, sizeof(filepath), filepath);

  this->mutex.lock();
  ShareableAnim &sh_anim = *this->anims_map.lookup_or_add_cb(std::string(filepath), [&]() {
    std::unique_ptr<ShareableAnim> new_sh_anim = std::make_unique<ShareableAnim>();
    return new_sh_anim;
  });
  this->mutex.unlock();
  return sh_anim;
}

void AnimManager::strip_anims_acquire(const Scene *scene, blender::Vector<Sequence *> strips)
{
  this->parallel_load_anims(scene, strips, true);
}

void AnimManager::strip_anims_acquire(const Scene *scene, Sequence *seq)
{
  ShareableAnim &sh_anim = this->cache_entry_get(scene, seq);
  sh_anim.mutex.lock();
  sh_anim.acquire_anims(scene, seq);
}

void AnimManager::strip_anims_release(const Scene *scene, blender::Vector<Sequence *> &strips)
{
  for (Sequence *seq : strips) {
    if (seq->type == SEQ_TYPE_MOVIE) {
      ShareableAnim &sh_anim = this->cache_entry_get(scene, seq);
      sh_anim.unlock();
    }
  }
}

void AnimManager::strip_anims_release(const Scene *scene, Sequence *seq)
{
  ShareableAnim &sh_anim = this->cache_entry_get(scene, seq);
  sh_anim.unlock();
}

blender::Vector<ImBufAnim *> AnimManager::strip_anims_get(const Scene *scene, const Sequence *seq)
{
  ShareableAnim &sh_anim = this->cache_entry_get(scene, seq);
  return sh_anim.anims;
}

void AnimManager::free_anims_by_seq(const Scene *scene, const Sequence *seq)
{
  ShareableAnim &sh_anim = this->cache_entry_get(scene, seq);
  sh_anim.release_from_all_strips();
}

AnimManager::~AnimManager()
{
  if (this->prefetch_thread.joinable()) {
    this->prefetch_thread.join();
  }

  for (std::unique_ptr<ShareableAnim> &sh_anim : this->anims_map.values()) {
    sh_anim->release_from_all_strips();
  }
}

AnimManager *seq_anim_manager_ensure(Editing *ed)
{
  if (ed->runtime.anim_lookup == nullptr) {
    ed->runtime.anim_lookup = MEM_new<AnimManager>(__func__);
  }
  return ed->runtime.anim_lookup;
}

void seq_anim_manager_free(const Editing *ed)
{
  if (ed->runtime.anim_lookup != nullptr) {
    MEM_delete(ed->runtime.anim_lookup);
  }
}

}  // namespace blender::seq
