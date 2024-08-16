/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

#include "BLI_map.hh"
#include "BLI_math_base.h"
#include "BLI_path_util.h"
#include "BLI_set.hh"
#include "BLI_task.hh"
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

constexpr int THUMB_SIZE = 256;  //@TODO: use SEQ_RENDER_THUMB_SIZE and/or remove that one

static ThreadMutex thumb_cache_lock = BLI_MUTEX_INITIALIZER;

//@TODO: cache cleanup when full
struct ThumbnailCache {
  struct FrameEntry {
    int frame_index = 0;
    ImBuf *thumb = nullptr;
  };

  typedef Vector<FrameEntry> Value;

  //@TODO: add timestamp, when processing order by timestamps (starting from most recent)
  struct Request {
    explicit Request(const std::string &path,
                     int frame,
                     SequenceType type,
                     float time_frame,
                     int ch,
                     int width,
                     int height)
        : file_path(path),
          frame_index(frame),
          seq_type(type),
          timeline_frame(time_frame),
          channel(ch),
          full_width(width),
          full_height(height)
    {
    }
    std::string file_path;
    int frame_index = 0;
    SequenceType seq_type = SEQ_TYPE_IMAGE;

    float timeline_frame = 0;
    int channel = 0;
    int full_width = 0;
    int full_height = 0;

    uint64_t hash() const
    {
      return get_default_hash(file_path, frame_index, seq_type);
    }
    bool operator==(const Request &o) const
    {
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
      break;
  }
  return filepath;
}

static void image_size_to_thumb_size(int &r_width, int &r_height)
{
  float aspect = float(r_width) / float(r_height);
  if (r_width > r_height) {
    r_width = THUMB_SIZE;
    r_height = round_fl_to_int(THUMB_SIZE / aspect);
  }
  else {
    r_height = THUMB_SIZE;
    r_width = round_fl_to_int(THUMB_SIZE * aspect);
  }
}

static ImBuf *make_thumb_for_image(Scene *scene, const ThumbnailCache::Request &request)
{
  //@TODO: IMB_thumb_load_image skips files larger than 100MB if they don't have a dedicated
  // thumbnail code path. Need to add flags to stop checking that.
  ImBuf *ibuf = IMB_thumb_load_image(request.file_path.c_str(), THUMB_SIZE, nullptr);
  if (ibuf == nullptr) {
    return nullptr;
  }
  /* Keep only float buffer if we have both byte & float. */
  if (ibuf->float_buffer.data != nullptr && ibuf->byte_buffer.data != nullptr) {
    imb_freerectImBuf(ibuf);
  }

  seq_imbuf_to_sequencer_space(scene, ibuf, false);
  seq_imbuf_assign_spaces(scene, ibuf);
  return ibuf;
}

static ImBuf *scale_to_thumbnail_size(Scene *scene, ImBuf *ibuf)
{
  if (ibuf == nullptr) {
    return nullptr;
  }
  int width = ibuf->x;
  int height = ibuf->y;
  image_size_to_thumb_size(width, height);
  if (width == ibuf->x && height == ibuf->y) {
    return ibuf;
  }

  /* Scale ibuf to thumbnail size. */
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
  ThumbGenerationJob(Scene *scene, ThumbnailCache *cache) : scene_(scene), cache_(cache) {}

  static void ensure_job(const bContext *C, ThumbnailCache *cache);

 private:
  static void run_fn(void *customdata, wmJobWorkerStatus *worker_status);
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
    WM_jobs_callbacks(wm_job, run_fn, nullptr, nullptr, end_fn);

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
  clock_t t0 = clock();
  std::atomic<int> total_thumbs = 0, total_images = 0, total_movies = 0;

  ThumbGenerationJob *job = static_cast<ThumbGenerationJob *>(customdata);
  Vector<ThumbnailCache::Request> requests;
  while (!worker_status->stop) {
    /* Copy all current requests (under cache mutex lock). */
    BLI_mutex_lock(&thumb_cache_lock);
    requests.clear();
    requests.reserve(job->cache_->requests_.size());
    for (const auto &request : job->cache_->requests_) {
      requests.append(request);
    }
    BLI_mutex_unlock(&thumb_cache_lock);

    if (requests.is_empty()) {
      break;
    }

    /* Sort requests by file and increasing frame number. */
    std::sort(requests.begin(),
              requests.end(),
              [](const ThumbnailCache::Request &a, const ThumbnailCache::Request &b) {
                if (a.file_path != b.file_path) {
                  return a.file_path < b.file_path;
                }
                return a.frame_index < b.frame_index;
              });

    /* Process the requests in parallel. Split requests into approximately 4 groups:
     * we don't want to go too wide since that would potentially mean that a single input
     * movie gets assigned to more than one thread, and the thumbnail loading itself
     * is somewhat-threaded already. */
    int64_t grain_size = math::max(8ll, requests.size() / 4);
    threading::parallel_for(requests.index_range(), grain_size, [&](IndexRange range) {
      ImBufAnim *cur_anim = nullptr;
      std::string cur_anim_path;
      for (const int i : range) {
        const ThumbnailCache::Request &request = requests[i];
        if (worker_status->stop) {
          break;
        }

        ++total_thumbs;
        ImBuf *thumb = nullptr;
        if (request.seq_type == SEQ_TYPE_IMAGE) {
          ++total_images;
          thumb = make_thumb_for_image(job->scene_, request);
        }
        else if (request.seq_type == SEQ_TYPE_MOVIE) {
          ++total_movies;

          /* Are we swiching to a different movie file? */
          if (request.file_path != cur_anim_path) {
            if (cur_anim != nullptr) {
              IMB_free_anim(cur_anim);
              cur_anim = nullptr;
            }

            cur_anim_path = request.file_path;
            int flag = IB_rect;
            // if (seq->flag & SEQ_FILTERY) {
            //   flag |= IB_animdeinterlace; //@TODO
            // }
            int stream_index = 0;  //@TODO: seq->streamindex
            cur_anim = IMB_open_anim(cur_anim_path.c_str(), flag, stream_index, nullptr);
          }

          /* Decode the movie frame. */
          if (cur_anim != nullptr) {
            int frame_index = request.frame_index;
            thumb = IMB_anim_absolute(cur_anim, frame_index, IMB_TC_NONE, IMB_PROXY_NONE);
            if (thumb != nullptr) {
              seq_imbuf_assign_spaces(job->scene_, thumb);
            }
          }
        }
        else {
          BLI_assert_unreachable();
        }
        thumb = scale_to_thumbnail_size(job->scene_, thumb);

        /* Add result into the cache (under cache mutex lock). */
        BLI_mutex_lock(&thumb_cache_lock);
        ThumbnailCache::Value *val = job->cache_->map_.lookup_ptr(request.file_path);
        if (val != nullptr) {
          val->append({request.frame_index, thumb});
        }
        else {
          IMB_freeImBuf(thumb);
        }
        /* Remove the request from original set. */
        job->cache_->requests_.remove(request);
        BLI_mutex_unlock(&thumb_cache_lock);

        if (thumb) {
          worker_status->do_update = true;
        }
      }
      if (cur_anim != nullptr) {
        IMB_free_anim(cur_anim);
        cur_anim = nullptr;
      }
    });
  }

  clock_t t1 = clock();  //@TODO: debug log
  printf("Thumb job new: %i thumbs (%i img, %i movie) in %.3f sec\n",
         total_thumbs.load(),
         total_images.load(),
         total_movies.load(),
         double(t1 - t0) / CLOCKS_PER_SEC);
}

void ThumbGenerationJob::end_fn(void *customdata)
{
  ThumbGenerationJob *job = static_cast<ThumbGenerationJob *>(customdata);
  WM_main_add_notifier(NC_SCENE | ND_SEQUENCER, job->scene_);
}

static ImBuf *query_thumbnail(ThumbnailCache &cache,
                              const std::string &key,
                              int frame_index,
                              float timeline_frame,
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
    ThumbnailCache::Request request(key,
                                    frame_index,
                                    SequenceType(seq->type),
                                    timeline_frame,
                                    seq->machine,
                                    img_width,
                                    img_height);
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

  timeline_frame = math::round(timeline_frame);

  const std::string key = get_path_from_seq(scene, seq, timeline_frame);
  int frame_index = SEQ_give_frame_index(scene, seq, timeline_frame);
  if (seq->type == SEQ_TYPE_MOVIE) {
    frame_index += seq->anim_startofs;
  }

  BLI_mutex_lock(&thumb_cache_lock);
  ThumbnailCache *cache = ensure_thumbnail_cache(scene);
  ImBuf *res = query_thumbnail(*cache, key, frame_index, timeline_frame, C, seq);
  BLI_mutex_unlock(&thumb_cache_lock);

  if (res) {
    IMB_refImBuf(res);
  }
  return res;
}

void thumbnail_cache_invalidate_strip(Scene *scene, const Sequence *seq)
{
  if (!can_have_thumbnail(scene, seq)) {
    return;
  }
  //@TODO implement, and call this when reloading strips
}

void thumbnail_cache_discard_requests_outside(Scene *scene, const rctf &rect)
{
  BLI_mutex_lock(&thumb_cache_lock);
  ThumbnailCache *cache = query_thumbnail_cache(scene);
  if (cache != nullptr) {
    cache->requests_.remove_if([&](const ThumbnailCache::Request &request) {
      return request.timeline_frame < rect.xmin || request.timeline_frame > rect.xmax ||
             request.channel < rect.ymin || request.channel > rect.ymax;
    });
  }
  BLI_mutex_unlock(&thumb_cache_lock);
}

void thumbnail_cache_clear(Scene *scene)
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

std::string thumbnail_cache_get_stats(Scene *scene)
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

void thumbnail_cache_for_each_request(
    Scene *scene,
    FunctionRef<void(int index, float timeline_frame, int channel, int frame_index)> callback)
{
  BLI_mutex_lock(&thumb_cache_lock);
  ThumbnailCache *cache = query_thumbnail_cache(scene);
  if (cache != nullptr) {
    int index = 0;
    for (const ThumbnailCache::Request &request : cache->requests_) {
      callback(index, request.timeline_frame, request.channel, request.frame_index);
      index++;
    }
  }
  BLI_mutex_unlock(&thumb_cache_lock);
}

}  // namespace blender::seq
