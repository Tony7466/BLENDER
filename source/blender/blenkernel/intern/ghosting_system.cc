/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_ghosting_system.hh"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_build.h"

#include "DNA_object_types.h"

namespace blender::bke::ghosts {

GhostingSystem::GhostingSystem() {}

GhostingSystem::~GhostingSystem()
{
  for (auto [key, ghost_frame] : this->ghost_frames_.items()) {
    DEG_graph_free(ghost_frame.depsgraph);
  }
}

bool GhostingSystem::is_empty()
{
  return ghost_frames_.is_empty();
}

void GhostingSystem::request_ghost(
    Main *bmain, Scene *scene, ViewLayer *view_layer, Object *object, const int frame)
{
  if (!ghost_frames_.contains(frame)) {
    GhostFrame new_ghost_frame;
    new_ghost_frame.depsgraph = DEG_graph_new(bmain, scene, view_layer, DAG_EVAL_VIEWPORT);
    new_ghost_frame.objects.add_new(object);
    ID *id = &object->id;
    DEG_graph_build_from_ids(new_ghost_frame.depsgraph, &id, 1);
    ghost_frames_.add_new(frame, new_ghost_frame);
    return;
  }
  GhostFrame &ghost_frame = ghost_frames_.lookup(frame);
  if (ghost_frame.objects.contains(object)) {
    /* Object already in the depsgraph. */
    return;
  }
  /* Ghost frame exists, but doesn't contain the object. Recreate the depsgraph with the new
   * object. */
  DEG_graph_free(ghost_frame.depsgraph);
  ghost_frame.depsgraph = DEG_graph_new(bmain, scene, view_layer, DAG_EVAL_VIEWPORT);

  ghost_frame.objects.add_new(object);
  /* Collect all the IDs. */
  Array<ID *> ids(ghost_frame.objects.size());
  int i = 0;
  for (Object *ob : ghost_frame.objects) {
    ids[i] = &ob->id;
    i++;
  }
  /* Build depsgraph. */
  DEG_graph_build_from_ids(ghost_frame.depsgraph, ids.data(), ids.size());
}

void GhostingSystem::evaluate_all_frames()
{
  if (ghost_frames_.is_empty()) {
    /* No ghost frames are built. */
    return;
  }
  for (auto [key, ghost_frame] : ghost_frames_.items()) {
    DEG_evaluate_on_refresh(ghost_frame.depsgraph, false);
  }
}

void GhostingSystem::evaluate_frames(Span<int> frames)
{
  if (ghost_frames_.is_empty()) {
    /* No ghost frames are built. */
    return;
  }
  for (const int frame : frames) {
    if (!ghost_frames_.contains(frame)) {
      continue;
    }
    GhostFrame &ghost_frame = ghost_frames_.lookup(frame);
    DEG_evaluate_on_refresh(ghost_frame.depsgraph, false);
  }
}

void GhostingSystem::evaluate_on_framechange()
{
  if (ghost_frames_.is_empty()) {
    /* No ghost frames are built. */
    return;
  }
  for (auto [key, ghost_frame] : this->ghost_frames_.items()) {
    DEG_evaluate_on_framechange(ghost_frame.depsgraph, float(key));
  }
}

void GhostingSystem::foreach_ghost_frame(FunctionRef<void(int, GhostFrame &)> function)
{
  if (ghost_frames_.is_empty()) {
    /* No ghost frames are built. */
    return;
  }
  for (auto [key, ghost_frame] : ghost_frames_.items()) {
    function(key, ghost_frame);
  }
}

}  // namespace blender::bke::ghosts