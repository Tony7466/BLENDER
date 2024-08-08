/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

#include "BLI_map.hh"
#include "BLI_math_base.h"
#include "BLI_set.hh"
#include "BLI_path_util.h"
#include "BLI_threads.h"
#include "BLI_vector.hh"

#include "BKE_main.hh"

#include "DNA_scene_types.h"
#include "DNA_sequence_types.h"

#include "IMB_imbuf.hh"

#include "SEQ_render.hh"
#include "SEQ_thumbnail_cache.hh"
#include "SEQ_time.hh"

#include <fmt/format.h>

namespace blender::seq {

static ThreadMutex thumb_cache_lock = BLI_MUTEX_INITIALIZER;

struct ThumbnailCache {
  struct FrameEntry {
    int frame_index = 0;
    //@TODO: do we need timecode?
    ImBuf *thumb = nullptr;
  };

  typedef Vector<FrameEntry> Value;

  struct Request {
    std::string file_path;
    int frame_index = 0;
    SequenceType seq_type = SEQ_TYPE_IMAGE;

    int full_width = 0;
    int full_height = 0;

    uint64_t hash() const {
      return get_default_hash(file_path, frame_index, seq_type);
    }
    bool operator==(const Request& o) const {
      return frame_index == o.frame_index && seq_type == o.seq_type && file_path == o.file_path;
    }
  };

  //@TODO: do we need something for multi-view/stereo?
  Map<std::string, Value> map_;

  Set<Request> requests_;

  ~ThumbnailCache()
  {
    clear();
  }

  void clear()
  {
    for (const auto &kvp : map_.items()) {
      for (const auto &thumb : kvp.value) {
        IMB_freeImBuf(thumb.thumb);
      }
    }
    map_.clear_and_shrink();
    requests_.clear_and_shrink();
  }
};

static ThumbnailCache *ensure_thumbnail_cache(Scene *scene)
{
  ThumbnailCache **cache = &scene->ed->runtime.thumbnail_cache;
  if (*cache == nullptr) {
    *cache = MEM_new<ThumbnailCache>(__func__);
  }
  return *cache;
}

static ThumbnailCache *query_thumbnail_cache(Scene *scene)
{
  if (scene == nullptr || scene->ed == nullptr) {
    return nullptr;
  }
  return scene->ed->runtime.thumbnail_cache;
}

static bool can_have_thumbnail(Scene *scene, const Sequence *seq)
{
  if (scene == nullptr || scene->ed == nullptr || seq == nullptr) {
    return false;
  }
  if (!ELEM(seq->type, SEQ_TYPE_MOVIE, SEQ_TYPE_IMAGE)) {
    return false;
  }
  if ((seq->flag & SEQ_FLAG_SKIP_THUMBNAILS) != 0) {
    return false;
  }
  const StripElem *se = seq->strip->stripdata;
  if (se->orig_height == 0 || se->orig_width == 0) {
    return false;
  }
  return true;
}

static std::string get_path_from_seq(Scene *scene, const Sequence *seq, float timeline_frame)
{
  char filepath[FILE_MAX];
  filepath[0] = 0;
  switch (seq->type) {
    case SEQ_TYPE_IMAGE: {
      const StripElem *s_elem = SEQ_render_give_stripelem(scene, seq, timeline_frame);
      if (s_elem != nullptr) {
        BLI_path_join(filepath, sizeof(filepath), seq->strip->dirpath, s_elem->filename);
        BLI_path_abs(filepath, ID_BLEND_PATH_FROM_GLOBAL(&scene->id));
      }
      // ibuf = seq_render_image_strip_view(context, seq, filepath, prefix, ext, context->view_id);
    } break;
    case SEQ_TYPE_MOVIE:
      BLI_path_join(
          filepath, sizeof(filepath), seq->strip->dirpath, seq->strip->stripdata->filename);
      BLI_path_abs(filepath, ID_BLEND_PATH_FROM_GLOBAL(&scene->id));

      // seq_open_anim_file(context->scene, seq, false);
      // StripAnim *sanim = static_cast<StripAnim *>(seq->anims.first);
      // ibuf = seq_render_movie_strip_view(context, seq, timeline_frame, sanim, r_is_proxy_image);
      break;
  }
  return filepath;
}

static ImBuf *query_thumbnail(ThumbnailCache& cache, const std::string& key, int frame_index, const Sequence *seq)
{
  ThumbnailCache::Value *val = cache.map_.lookup_ptr(key);

  if (val == nullptr) {
    /* Nothing in cache for this path yet. */
    ThumbnailCache::Value value;
    cache.map_.add_new(key, value);
    val = cache.map_.lookup_ptr(key);
  }

  if (val == nullptr) {
    return nullptr;
  }

  /* Search thumbnail entries of this file for closest match to the frame we want. */
  int64_t best_index = -1;
  int best_score = INT_MAX;
  for (int64_t index = 0; index < val->size(); index++) {
    int score = math::abs(frame_index - (*val)[index].frame_index);
    if (score < best_score) {
      best_score = score;
      best_index = index;
      if (score == 0) {
        break;
      }
    }
  }

  if (best_score > 0) {
    /* We do not have an exact frame match, add a thumb generation request. */
    const StripElem *se = seq->strip->stripdata;
    int img_width = se->orig_width;
    int img_height = se->orig_height;
    //@TODO: actual thing
    //ThumbnailCache::Request request{
    //    key, frame_index, SequenceType(seq->type), img_width, img_height};
    //cache.requests_.add(request);
    //
    // for now just a test random colored image:
    float aspect = float(img_width) / float(img_height);
    constexpr int THUMB_SIZE = 256; // SEQ_RENDER_THUMB_SIZE
    if (img_width > img_height) {
      img_width = THUMB_SIZE;
      img_height = round_fl_to_int(THUMB_SIZE / aspect);
    }
    else {
      img_height = THUMB_SIZE;
      img_width = round_fl_to_int(THUMB_SIZE * aspect);
    }

    ImBuf *thumb = IMB_allocImBuf(img_width, img_height, 32, IB_rect);
    float col[4] = {
        (rand() % 100) / 100.0f, (rand() % 100) / 100.0f, (rand() % 100) / 100.0f, 1.0f};
    IMB_rectfill(thumb, col);
    val->append({frame_index, thumb});
  }

  /* Return the closest thumbnail fit we have so far. */
  return best_index >= 0 ? (*val)[best_index].thumb : nullptr;
}

ImBuf* thumbnail_cache_get(Scene* scene, const Sequence* seq, float timeline_frame)
{
  if (!can_have_thumbnail(scene, seq)) {
    return nullptr;
  }

  const std::string key = get_path_from_seq(scene, seq, timeline_frame);
  const int frame_index = SEQ_give_frame_index(scene, seq, timeline_frame);

  BLI_mutex_lock(&thumb_cache_lock);
  ThumbnailCache *cache = ensure_thumbnail_cache(scene);
  ImBuf *res = query_thumbnail(*cache, key, frame_index, seq);
  BLI_mutex_unlock(&thumb_cache_lock);

  if (res)
    IMB_refImBuf(res);
  return res;
}

void thumbnail_cache_invalidate_strip(Scene* scene, const Sequence* seq)
{
  if (!can_have_thumbnail(scene, seq)) {
    return;
  }
  //@TODO implement, and call this when reloading strips
}

void thumbnail_cache_clear(Scene* scene)
{
  BLI_mutex_lock(&thumb_cache_lock);
  ThumbnailCache *cache = query_thumbnail_cache(scene);
  if (cache != nullptr) {
    scene->ed->runtime.thumbnail_cache->clear();
  }
  BLI_mutex_unlock(&thumb_cache_lock);
}

void thumbnail_cache_destroy(Scene *scene)
{
  BLI_mutex_lock(&thumb_cache_lock);
  ThumbnailCache *cache = query_thumbnail_cache(scene);
  if (cache != nullptr) {
    BLI_assert(cache == scene->ed->runtime.thumbnail_cache);
    MEM_delete(scene->ed->runtime.thumbnail_cache);
    scene->ed->runtime.thumbnail_cache = nullptr;
  }
  BLI_mutex_unlock(&thumb_cache_lock);
}

std::string thumbnail_cache_get_stats(Scene* scene)
{
  std::string stats = "<no cache>";
  BLI_mutex_lock(&thumb_cache_lock);
  ThumbnailCache *cache = query_thumbnail_cache(scene);
  if (cache != nullptr) {
    int64_t entries = 0, bytes = 0;
    for (const auto &kvp : cache->map_.items()) {
      entries += kvp.value.size();
      for (const auto &thumb : kvp.value) {
        bytes += thumb.thumb->x * thumb.thumb->y * 4;
      }
    }
    stats = fmt::format("Thumb cache (new): {} file paths, {} thumbs, {:.1f} MB, {} requests",
                        cache->map_.size(),
                        entries,
                        bytes / double(1024 * 1024),
                        cache->requests_.size());
  }
  BLI_mutex_unlock(&thumb_cache_lock);
  return stats;
}

}  // namespace blender::seq

