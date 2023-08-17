/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief Ghosting system.
 */

#include "BLI_function_ref.hh"
#include "BLI_map.hh"
#include "BLI_set.hh"
#include "BLI_utility_mixins.hh"

struct Main;
struct Depsgraph;
struct Scene;
struct ViewLayer;
struct Object;

namespace blender::bke::ghosts {

struct GhostFrame {
  Depsgraph *depsgraph;
  Set<Object *> objects;
};

class GhostingSystem : NonCopyable, NonMovable {
 private:
  Map<int, GhostFrame> ghost_frames_;

  /* Main, scene, and view layer this ghosting system is built for. */
  Main *bmain_;
  Scene *scene_;
  ViewLayer *view_layer_;

 public:
  GhostingSystem();
  ~GhostingSystem();

  bool is_empty();

  void request_ghost(Main *bmain, Scene *scene, ViewLayer *view_layer, Object *object, int frame);

  void evaluate_all_frames();
  void evaluate_frames(Span<int> frames);
  void evaluate_on_framechange();

  void foreach_ghost_frame(FunctionRef<void(int, GhostFrame&)> function);
};

}  // namespace blender::bke::ghosts