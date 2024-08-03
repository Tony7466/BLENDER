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
#include "BLI_set.hh"

namespace blender::seq {

class ShareableAnim {
 public:
  blender::Vector<ImBufAnim *> anims; /* Ordered by view_id. */
  blender::Set<Sequence *> users;
  std::mutex mutex;

  void release_from_strip(Sequence *seq);
  void release_from_all_strips();
  void acquire_anims(const Scene *scene, Sequence *seq);
  void unlock();
};

class AnimManager {
 public:
  blender::Map<std::string, std::unique_ptr<ShareableAnim>> anims_map;
  std::mutex mutex;
  std::thread prefetch_thread;

  /**
   * Free anims of strips behind current frame and prefetch anims that are to be played.
   */
  void manage_anims(const Scene *scene);
  /**
   * Load anims used by strips and lock them so they won't be freed.
   */
  void strip_anims_acquire(const Scene *scene, blender::Vector<Sequence *> strips);
  void strip_anims_acquire(const Scene *scene, Sequence *seq);
  /**
   * Unlock anims used by strips.
   */
  void strip_anims_release(const Scene *scene, blender::Vector<Sequence *> &strips);
  void strip_anims_release(const Scene *scene, Sequence *seq);
  /**
   * Get anims used by `seq`.
   */
  blender::Vector<ImBufAnim *> strip_anims_get(const Scene *scene, const Sequence *seq);
  /**
   * Free anims used by `seq`.
   */
  void free_anims_by_seq(const Scene *scene, const Sequence *seq);
  ~AnimManager();

 private:
  ShareableAnim &cache_entry_get(const Scene *scene, const Sequence *seq);
  void free_unused_anims(blender::Vector<Sequence *> &strips);
  void free_unused_and_prefetch_anims(const Scene *scene);
  void parallel_load_anims(const Scene *scene,
                           blender::Vector<Sequence *> &strips,
                           bool keep_locked);
};

AnimManager *seq_anim_manager_ensure(Editing *ed);
void seq_anim_manager_free(const Editing *ed);

}  // namespace blender::seq
