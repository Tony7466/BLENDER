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

namespace blender::seq {

static constexpr int MAX_THUMBNAILS = 5000;

// #define DEBUG_PRINT_THUMB_JOB_TIMES

static ThreadMutex thumb_cache_lock = BLI_MUTEX_INITIALIZER;

/* Thumbnail cache is a map keyed by media file path, with values being
 * the various thumbnails that are loaded for it (mostly images would contain just
 * one thumbnail frame, but movies can contain multiple).
 *
 * File entries and individual frame entries also record the timestamp when they were
 * last accessed, so that when the cache is full, some of the old entries can be removed.
 *
 * Thumbnails that are requested but do not have an exact match in the cache, are added
 * to the "requests" set. The requests are processed in the background by a WM job. */
struct ThumbnailCache {
  struct FrameEntry {
    int frame_index = 0;
    int stream_index = 0;
    ImBuf *thumb = nullptr;
    double used_at = 0;
  };

  struct FileEntry {
    Vector<FrameEntry> frames;
    double used_at = 0;
  };

  //@TODO: when processing order by timestamps (starting from most recent) somehow?
  struct Request {
    explicit Request(const std::string &path,
                     int frame,
                     int stream,
                     SequenceType type,
                     double time,
                     float time_frame,
                     int ch,
                     int width,
                     int height)
        : file_path(path),
          frame_index(frame),
          stream_index(stream),
          seq_type(type),
          requested_at(time),
          timeline_frame(time_frame),
          channel(ch),
          full_width(width),
          full_height(height)
    {
    }
    /* These determine request uniqueness (for equality/hash in a Set). */
    std::string file_path;
    int frame_index = 0;
    int stream_index = 0;
    SequenceType seq_type = SEQ_TYPE_IMAGE;

    /* The following members are payload and do not contribute to uniqueness. */
    double requested_at = 0;
    float timeline_frame = 0;
    int channel = 0;
    int full_width = 0;
    int full_height = 0;

    uint64_t hash() const
    {
      return get_default_hash(file_path, frame_index, stream_index, seq_type);
    }
    bool operator==(const Request &o) const
    {
      return frame_index == o.frame_index && stream_index == o.stream_index &&
             seq_type == o.seq_type && file_path == o.file_path;
    }
  };

  //@TODO: do we need something for multi-view/stereo?
  Map<std::string, FileEntry> map_;
  Set<Request> requests_;

  ~ThumbnailCache()
  {
    clear();
  }

  void clear()
  {
    for (const auto &kvp : map_.items()) {
      for (const auto &thumb : kvp.value.frames) {
        IMB_freeImBuf(thumb.thumb);
      }
    }
    map_.clear_and_shrink();
    requests_.clear_and_shrink();
  }

  void remove_entry(const std::string &path)
  {
    FileEntry *entry = map_.lookup_ptr(path);
    if (entry == nullptr) {
      return;
    }
    for (const auto &thumb : entry->frames) {
      IMB_freeImBuf(thumb.thumb);
    }
    map_.remove_contained(path);
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
    r_width = SEQ_THUMB_SIZE;
    r_height = round_fl_to_int(SEQ_THUMB_SIZE / aspect);
  }
  else {
    r_height = SEQ_THUMB_SIZE;
    r_width = round_fl_to_int(SEQ_THUMB_SIZE * aspect);
  }
}

static ImBuf *make_thumb_for_image(Scene *scene, const ThumbnailCache::Request &request)
{
  ImBuf *ibuf = IMB_thumb_load_image(
      request.file_path.c_str(), SEQ_THUMB_SIZE, nullptr, IMBThumbLoadFlags::LoadLargeFiles);
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

static void scale_to_thumbnail_size(ImBuf *ibuf)
{
  if (ibuf == nullptr) {
    return;
  }
  int width = ibuf->x;
  int height = ibuf->y;
  image_size_to_thumb_size(width, height);
  IMB_scale(ibuf, width, height, IMBScaleFilter::Nearest, false);
}

/* Background job that processes in-flight thumbnail requests. */
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
      wm, win, scene, "Strip Thumbnails", eWM_JobFlag(0), WM_JOB_TYPE_SEQ_DRAW_THUMBNAIL);
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
#ifdef DEBUG_PRINT_THUMB_JOB_TIMES
  clock_t t0 = clock();
  std::atomic<int> total_thumbs = 0, total_images = 0, total_movies = 0;
#endif

  ThumbGenerationJob *job = static_cast<ThumbGenerationJob *>(customdata);
  Vector<ThumbnailCache::Request> requests;
  while (!worker_status->stop) {
    /* Under cache mutex lock: copy all current requests into a vector for processing.
     * Note: keep the requests set intact! We don't want to add new requests for same
     * items while we are processing them. They will be removed from the set once
     * they are finished, one by one. */
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

    /* Sort requests by file, stream and increasing frame index. */
    std::sort(requests.begin(),
              requests.end(),
              [](const ThumbnailCache::Request &a, const ThumbnailCache::Request &b) {
                if (a.file_path != b.file_path) {
                  return a.file_path < b.file_path;
                }
                if (a.stream_index != b.stream_index) {
                  return a.stream_index < b.stream_index;
                }
                return a.frame_index < b.frame_index;
              });

    /* Process the requests in parallel. Split requests into approximately 4 groups:
     * we don't want to go too wide since that would potentially mean that a single input
     * movie gets assigned to more than one thread, and the thumbnail loading itself
     * is somewhat-threaded already. */
    int64_t grain_size = math::max(8ll, requests.size() / 4);
    threading::parallel_for(requests.index_range(), grain_size, [&](IndexRange range) {
      /* Often the same movie file is chopped into multiple strips next to each other.
       * Since the requests are sorted by file path and frame index, we can reuse ImBufAnim
       * objects between them for performance. */
      ImBufAnim *cur_anim = nullptr;
      std::string cur_anim_path;
      int cur_stream = 0;
      for (const int i : range) {
        const ThumbnailCache::Request &request = requests[i];
        if (worker_status->stop) {
          break;
        }

#ifdef DEBUG_PRINT_THUMB_JOB_TIMES
        ++total_thumbs;
#endif
        ImBuf *thumb = nullptr;
        if (request.seq_type == SEQ_TYPE_IMAGE) {
          /* Load thumbnail for an image. */
#ifdef DEBUG_PRINT_THUMB_JOB_TIMES
          ++total_images;
#endif
          thumb = make_thumb_for_image(job->scene_, request);
        }
        else if (request.seq_type == SEQ_TYPE_MOVIE) {
          /* Load thumbnail for an movie. */
#ifdef DEBUG_PRINT_THUMB_JOB_TIMES
          ++total_movies;
#endif

          /* Are we swiching to a different movie file / stream? */
          if (request.file_path != cur_anim_path || request.stream_index != cur_stream) {
            if (cur_anim != nullptr) {
              IMB_free_anim(cur_anim);
              cur_anim = nullptr;
            }

            cur_anim_path = request.file_path;
            cur_stream = request.stream_index;
            int flag = IB_rect;
            // if (seq->flag & SEQ_FILTERY) {
            //   flag |= IB_animdeinterlace; //@TODO
            // }
            cur_anim = IMB_open_anim(cur_anim_path.c_str(), flag, cur_stream, nullptr);
          }

          /* Decode the movie frame. */
          if (cur_anim != nullptr) {
            thumb = IMB_anim_absolute(cur_anim, request.frame_index, IMB_TC_NONE, IMB_PROXY_NONE);
            if (thumb != nullptr) {
              seq_imbuf_assign_spaces(job->scene_, thumb);
            }
          }
        }
        else {
          BLI_assert_unreachable();
        }

        scale_to_thumbnail_size(thumb);

        /* Add result into the cache (under cache mutex lock). */
        BLI_mutex_lock(&thumb_cache_lock);
        ThumbnailCache::FileEntry *val = job->cache_->map_.lookup_ptr(request.file_path);
        if (val != nullptr) {
          val->used_at = math::max(val->used_at, request.requested_at);
          val->frames.append(
              {request.frame_index, request.stream_index, thumb, request.requested_at});
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

#ifdef DEBUG_PRINT_THUMB_JOB_TIMES
  clock_t t1 = clock();
  printf("VSE thumb job: %i thumbs (%i img, %i movie) in %.3f sec\n",
         total_thumbs.load(),
         total_images.load(),
         total_movies.load(),
         double(t1 - t0) / CLOCKS_PER_SEC);
#endif
}

void ThumbGenerationJob::end_fn(void *customdata)
{
  ThumbGenerationJob *job = static_cast<ThumbGenerationJob *>(customdata);
  WM_main_add_notifier(NC_SCENE | ND_SEQUENCER, job->scene_);
}

static ImBuf *query_thumbnail(ThumbnailCache &cache,
                              const std::string &key,
                              int frame_index,
                              double cur_time,
                              float timeline_frame,
                              const bContext *C,
                              const Sequence *seq)
{
  ThumbnailCache::FileEntry *val = cache.map_.lookup_ptr(key);

  if (val == nullptr) {
    /* Nothing in cache for this path yet. */
    ThumbnailCache::FileEntry value;
    value.used_at = cur_time;
    cache.map_.add_new(key, value);
    val = cache.map_.lookup_ptr(key);
  }

  if (val == nullptr) {
    return nullptr;
  }

  /* Search thumbnail entries of this file for closest match to the frame we want. */
  int64_t best_index = -1;
  int best_score = INT_MAX;
  for (int64_t index = 0; index < val->frames.size(); index++) {
    /* Make video stream mismatch count way more than a frame mismatch. */
    int score = math::abs(frame_index - val->frames[index].frame_index) +
                math::abs(seq->streamindex - val->frames[index].stream_index) * 1024;
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
                                    seq->streamindex,
                                    SequenceType(seq->type),
                                    cur_time,
                                    timeline_frame,
                                    seq->machine,
                                    img_width,
                                    img_height);
    cache.requests_.add(request);
    ThumbGenerationJob::ensure_job(C, &cache);
  }

  if (best_index < 0) {
    return nullptr;
  }

  /* Return the closest thumbnail fit we have so far. */
  val->used_at = math::max(val->used_at, cur_time);
  val->frames[best_index].used_at = math::max(val->frames[best_index].used_at, cur_time);
  return val->frames[best_index].thumb;
}

ImBuf *thumbnail_cache_get(
    const bContext *C, Scene *scene, const Sequence *seq, float timeline_frame, double cur_time)
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
  ImBuf *res = query_thumbnail(*cache, key, frame_index, cur_time, timeline_frame, C, seq);
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

void thumbnail_cache_maintain_capacity(Scene *scene, double cur_time)
{
  BLI_mutex_lock(&thumb_cache_lock);
  ThumbnailCache *cache = query_thumbnail_cache(scene);
  if (cache != nullptr) {

    /* Count total number of thumbnails, and track which one is the least recently used file. */
    int64_t entries = 0;
    std::string oldest_file;
    /* Do not remove thumbnails for files used within last 1 sec. */
    double oldest_time = cur_time - 1.0;
    int64_t oldest_entries = 0;
    for (const auto &kvp : cache->map_.items()) {
      entries += kvp.value.frames.size();
      if (kvp.value.used_at < oldest_time) {
        oldest_file = kvp.key;
        oldest_time = kvp.value.used_at;
        oldest_entries = kvp.value.frames.size();
      }
    }

    /* If we're beyond capacity and have a long-unused file, remove that. */
    if (entries > MAX_THUMBNAILS && !oldest_file.empty()) {
      cache->remove_entry(oldest_file);
      entries -= oldest_entries;
    }

    /* If we're still beyond capacity, remove individual long-unused (but not within last 10 sec)
     * individual frames. */
    if (entries > MAX_THUMBNAILS) {
      for (const auto &kvp : cache->map_.items()) {
        for (int64_t i = 0; i < kvp.value.frames.size(); i++) {
          if (kvp.value.frames[i].used_at < cur_time - 10.0) {
            IMB_freeImBuf(kvp.value.frames[i].thumb);
            kvp.value.frames.remove_and_reorder(i);
          }
        }
      }
    }
  }
  BLI_mutex_unlock(&thumb_cache_lock);
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
      entries += kvp.value.frames.size();
      for (const auto &thumb : kvp.value.frames) {
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
