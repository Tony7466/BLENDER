/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

struct Scene;
struct Sequence;

#include <mutex>
#include <thread>

#include "BLI_map.hh"

class ShareableAnim {
 public:
  blender::Vector<ImBufAnim *> anims; /* Ordered by view_id. */
  blender::Vector<Sequence *> users;
  bool multiview_loaded = false;
  std::unique_ptr<std::mutex> mutex = std::make_unique<std::mutex>();

  void release_from_strip(Sequence *seq);
  void release_from_all_strips(void);
  void acquire_anims(const Scene *scene, Sequence *seq, bool openfile);
  bool has_anim(const Scene *scene, Sequence *seq);
  bool try_lock();
  void unlock();
};

class AnimManager {
 public:
  blender::Map<std::string, ShareableAnim> anims;
  std::mutex mutex;
  std::thread prefetch_thread;

  /**
   * Free anims of strips behind current frame and prefetch anims that are to be played.
   */
  void manage_anims(const Scene *scene);
  /**
   * Load anims used by strips and lock them so they won't be freed.
   */
  void strip_anims_laod_and_lock(const Scene *scene,
                                 blender::Vector<Sequence *> &strips);  // acquire?
  /**
   * Unlock anims used by strips.
   */
  void strip_anims_unlock(const Scene *scene, blender::Vector<Sequence *> &strips);  // release?
  /**
   * Get anims used by `seq`.
   */
  blender::Vector<ImBufAnim *> &strip_anims_get(const Scene *scene, const Sequence *seq);
  /**
   * Free anims used by `seq`.
   */
  void free_anims_by_seq(const Scene *scene, const Sequence *seq);

 private:
  ShareableAnim &cache_entry_get(const Scene *scene, const Sequence *seq);
  void free_unused_anims(const Editing *ed, blender::Vector<Sequence *> &strips);
  void free_unused_and_prefetch_anims(const Scene *scene);
  void parallel_load_anims(const Scene *scene, blender::Vector<Sequence *> &strips, bool unlock);
};

void seq_open_anim_file(const Scene *scene, Sequence *seq, bool openfile);
AnimManager *seq_anim_lookup_ensure(Editing *ed);
