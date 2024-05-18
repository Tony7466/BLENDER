/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

struct Scene;
struct Sequence;

#include <thread>

#include "BLI_map.hh"
#include "BLI_struct_equality_utils.hh"
#include "BLI_threads.h"

class ShareableAnim {
 public:
  blender::Vector<ImBufAnim *> anims; /* Ordered by view_id. */
  blender::Vector<Sequence *> users;
  bool multiview_loaded = false;
  ThreadMutex mutex = BLI_MUTEX_INITIALIZER;

  void release_from_strip(Sequence *seq);
  void release_from_all_strips(void);
  void acquire_anims(const Scene *scene, Sequence *seq, bool openfile);
  bool has_anim(const Scene *scene, Sequence *seq);
  void lock();
  void unlock();
};

class AnimManager {
 public:
  blender::Map<std::string, ShareableAnim> anims;
  ThreadMutex mutex = BLI_MUTEX_INITIALIZER;
  std::thread prefetch_thread;

  void prefetch(const Scene *scene);
  void manage_anims(const Scene *scene);

  void strip_anims_laod_and_lock(const Scene *scene, blender::Vector<Sequence *> &strips);
  void strip_anims_unlock(const Scene *scene, blender::Vector<Sequence *> &strips);

  blender::Vector<ImBufAnim *> &strip_anims_get(const Scene *scene, const Sequence *seq);
  ShareableAnim &cache_entry_get(const Scene *scene, const Sequence *seq);
};

void seq_open_anim_file(const Scene *scene, Sequence *seq, bool openfile);
AnimManager *seq_anim_lookup_ensure(Editing *ed);
