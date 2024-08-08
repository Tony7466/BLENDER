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

#include "BKE_context.hh"
#include "BKE_main.hh"

#include "DNA_scene_types.h"
#include "DNA_sequence_types.h"

#include "IMB_imbuf.hh"

#include "SEQ_render.hh"
#include "SEQ_thumbnail_cache.hh"
#include "SEQ_time.hh"

#include "WM_api.hh"

#include <fmt/format.h>

#include "render.hh"

void sequencer_thumbnail_transform(ImBuf *in, ImBuf *out);  // render.cc @TODO cleanup

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

static void image_size_to_thumb_size(int& r_width, int& r_height)
{
  float aspect = float(r_width) / float(r_height);
  constexpr int THUMB_SIZE = 256; //@TODO: use SEQ_RENDER_THUMB_SIZE
  if (r_width > r_height) {
    r_width = THUMB_SIZE;
    r_height = round_fl_to_int(THUMB_SIZE / aspect);
  }
  else {
    r_height = THUMB_SIZE;
    r_width = round_fl_to_int(THUMB_SIZE * aspect);
  }
}

static ImBuf* make_thumb_for_image(Scene *scene, const ThumbnailCache::Request& request)
{
  int flag = IB_rect | IB_metadata;
  //if (seq->alpha_mode == SEQ_ALPHA_PREMUL) { //@TODO
  //  flag |= IB_alphamode_premul;
  //}
  ImBuf *ibuf = IMB_loadiffname(request.file_path.c_str(), flag, nullptr); //@TODO: seq->strip->colorspace_settings.name
  if (ibuf == nullptr) {
    return nullptr;
  }
  /* Keep only float buffer if we have both byte & float. */
  if (ibuf->float_buffer.data != nullptr && ibuf->byte_buffer.data != nullptr) {
    imb_freerectImBuf(ibuf);
  }

  /* All sequencer color is done in SRGB space, linear gives odd cross-fades. */
  seq_imbuf_to_sequencer_space(scene, ibuf, false);
  seq_imbuf_assign_spaces(scene, ibuf);

  /* Scale ibuf to thumbnail size. */
  int width = request.full_width;
  int height = request.full_height;
  image_size_to_thumb_size(width, height);

  ImBuf *scaled_ibuf = IMB_allocImBuf(
      width, height, 32, ibuf->float_buffer.data ? IB_rectfloat : IB_rect);
  sequencer_thumbnail_transform(ibuf, scaled_ibuf);
  seq_imbuf_assign_spaces(scene, scaled_ibuf);
  IMB_freeImBuf(ibuf);

  return scaled_ibuf;
}


class ThumbGenerationJob {
  Scene *scene_ = nullptr;
  ThumbnailCache *cache_ = nullptr;

public:
  ThumbGenerationJob(Scene *scene, ThumbnailCache *cache) : scene_(scene), cache_(cache)
  {
  }

  static void ensure_job(const bContext *C, ThumbnailCache *cache);

private:
  static void run_fn(void *customdata, wmJobWorkerStatus *worker_status);
  static void update_fn(void *customdata);
  static void end_fn(void *customdata);
  static void free_fn(void *customdata);
};

void ThumbGenerationJob::ensure_job(const bContext *C, ThumbnailCache *cache)
{
  wmWindowManager *wm = CTX_wm_manager(C);
  wmWindow *win = CTX_wm_window(C);
  Scene *scene = CTX_data_scene(C);
  wmJob *wm_job = WM_jobs_get(
      wm, win, scene, "Strip Thumbnails", eWM_JobFlag(0), WM_JOB_TYPE_SEQ_DRAW_THUMBNAIL_NEW);
  if (!WM_jobs_is_running(wm_job)) {
    ThumbGenerationJob *tj = MEM_new<ThumbGenerationJob>("ThumbGenerationJob", scene, cache);
    WM_jobs_customdata_set(wm_job, tj, free_fn);
    WM_jobs_timer(wm_job, 0.1, NC_SCENE | ND_SEQUENCER, NC_SCENE | ND_SEQUENCER);
    WM_jobs_callbacks(wm_job, run_fn, nullptr, update_fn, end_fn);

    WM_jobs_start(wm, wm_job);
  }
}

void ThumbGenerationJob::free_fn(void *customdata)
{
  ThumbGenerationJob *job = static_cast<ThumbGenerationJob *>(customdata);
  MEM_delete(job);
}

void ThumbGenerationJob::run_fn(void *customdata, wmJobWorkerStatus *worker_status)
{
  ThumbGenerationJob *job = static_cast<ThumbGenerationJob *>(customdata);
  Set<ThumbnailCache::Request> requests;
  while (!worker_status->stop) {
    /* Move all the current requests (under cache mutex lock). */
    BLI_mutex_lock(&thumb_cache_lock);
    requests = job->cache_->requests_; //@TODO: more efficient with some sort of move thingy?
    job->cache_->requests_.clear();
    BLI_mutex_unlock(&thumb_cache_lock);

    if (requests.is_empty()) {
      break;
    }

    /* Process the requests. */
    for (auto &request : requests) {
      if (worker_status->stop) {
        break;
      }

      ImBuf *thumb = nullptr;
      if (request.seq_type == SEQ_TYPE_IMAGE) {
        thumb = make_thumb_for_image(job->scene_, request);
      }
      else {
        //@TODO: for now just a test random colored image
        int img_width = request.full_width;
        int img_height = request.full_height;
        image_size_to_thumb_size(img_width, img_height);
        thumb = IMB_allocImBuf(img_width, img_height, 32, IB_rect);
        float col[4] = {
            (rand() % 100) / 100.0f, (rand() % 100) / 100.0f, (rand() % 100) / 100.0f, 1.0f};
        IMB_rectfill(thumb, col);
      }

      /* Add result into the cache (under cache mutex lock). */
      BLI_mutex_lock(&thumb_cache_lock);
      ThumbnailCache::Value *val = job->cache_->map_.lookup_ptr(request.file_path);
      if (val != nullptr) {
        val->append({request.frame_index, thumb});
      }
      else {
        IMB_freeImBuf(thumb);
      }
      BLI_mutex_unlock(&thumb_cache_lock);
    }
  }
}

void ThumbGenerationJob::update_fn(void *customdata)
{
  ThumbGenerationJob *job = static_cast<ThumbGenerationJob *>(customdata);
}

void ThumbGenerationJob::end_fn(void *customdata)
{
  ThumbGenerationJob *job = static_cast<ThumbGenerationJob *>(customdata);
  WM_main_add_notifier(NC_SCENE | ND_SEQUENCER, job->scene_);
}



static ImBuf *query_thumbnail(ThumbnailCache &cache,
                              const std::string &key,
                              int frame_index,
                              const bContext *C,
                              const Sequence *seq)
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
    ThumbnailCache::Request request{
        key, frame_index, SequenceType(seq->type), img_width, img_height};
    cache.requests_.add(request);
    ThumbGenerationJob::ensure_job(C, &cache);
  }

  /* Return the closest thumbnail fit we have so far. */
  return best_index >= 0 ? (*val)[best_index].thumb : nullptr;
}

ImBuf *thumbnail_cache_get(const bContext *C,
                           Scene *scene,
                           const Sequence *seq,
                           float timeline_frame)
{
  if (!can_have_thumbnail(scene, seq)) {
    return nullptr;
  }

  const std::string key = get_path_from_seq(scene, seq, timeline_frame);
  const int frame_index = SEQ_give_frame_index(scene, seq, timeline_frame);

  BLI_mutex_lock(&thumb_cache_lock);
  ThumbnailCache *cache = ensure_thumbnail_cache(scene);
  ImBuf *res = query_thumbnail(*cache, key, frame_index, C, seq);
  BLI_mutex_unlock(&thumb_cache_lock);

  if (res) {
    IMB_refImBuf(res);
  }
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
        if (thumb.thumb) {
          bytes += thumb.thumb->x * thumb.thumb->y * 4;
        }
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

