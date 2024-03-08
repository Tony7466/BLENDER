/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

struct Scene;
struct Sequence;

#include "BLI_map.hh"
#include "BLI_struct_equality_utils.hh"
#include "BLI_threads.h"

class ShareableAnim {
 public:
  blender::Vector<ImBufAnim *> anims; /* In same order as strip views (`StripAnim` order). */
  blender::Vector<Sequence *> users;
  bool multiview_loaded = false;
  ThreadMutex *mutex;
  std::string filepath;

  void release_from_strip(Sequence *seq);
  void release_from_all_strips(void);
  void assign_to_strip(const Scene *scene, Sequence *seq);
  void acquire_anims(const Scene *scene, Sequence *seq, bool openfile);
  bool has_anim(const Scene *scene, Sequence *seq);
  ShareableAnim();
};

// This needs to be lockable class and has "working" property
class AnimManager {
 public:
  blender::Map<std::string, ShareableAnim> anims;
  ListBase threads{nullptr, nullptr};
  bool working{false};
  ThreadMutex *mutex;

  /* TODO: this will be tricky function - it must know where anims were prefetched last time and if
   current playhead position is within this range. It very well may not be...
   */
  // bool anims_are_prefetched()

  AnimManager();
  void manage_anims(Scene *scene);
  void load_set(const Scene *scene, blender::Vector<Sequence *> &strips);
};

void seq_open_anim_file(const Scene *scene, Sequence *seq, bool openfile);
void seq_anim_lookup_ensure(Editing *ed);
