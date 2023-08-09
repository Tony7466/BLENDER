/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_node_runtime.hh"
#include "RE_pipeline.h"

#include "IMB_imbuf.h"

struct bContext;
struct bNodeTree;
struct ImBuf;
struct Render;

namespace blender::ed::space_node {
using bke::DirtyState;
struct ShaderNodesPreviewJob;

/**
 * All properties of the previews present in this structure should always be corresponding to all
 * the previews cached in the `previews_map`. The size/dirtystate properties should be modified
 * only when all previews are corresponding to those properties. It may result in some more
 * refreshes, but it is the only way to make sure that the system always detect when a preview is
 * outdated.
 */
struct NestedTreePreviews {
  Render *previews_render = nullptr;
  /** Use this map to keep track of the latest #ImBuf used (after freeing the renderresult). */
  blender::Map<const int32_t, std::pair<ImBuf *, DirtyState>> previews_map;
  int preview_size;
  ShaderNodesPreviewJob *running_job = nullptr;

  /**
   * Dirty state of the bNodeTreePath vector. It is the sum of the tree_dirty_state of all the
   * nodetrees plus the sum of all the dirty_state of the group nodes.
   * If this state is dirty, it means that at least some nodes are dirty.
   */
  DirtyState treepath_dirtystate;
  /**
   * Dirty state of the current nodetree. If this flag is dirty, it means that all nodes are
   * dirty.
   */
  DirtyState whole_tree_dirtystate;
  /**
   * Dirty state of the nodetree viewed, it is used to know if at least one node needs to be
   * re-rendered.
   */
  DirtyState any_node_dirtystate;

  NestedTreePreviews(const int size) : preview_size(size) {}
  ~NestedTreePreviews()
  {
    if (this->previews_render) {
      RE_FreeRender(this->previews_render);
    }
    for (std::pair<ImBuf *, DirtyState> &cache : this->previews_map.values()) {
      IMB_freeImBuf(cache.first);
    }
  }
};

void free_previews(wmWindowManager &wm, SpaceNode &snode);
ImBuf *node_preview_acquire_ibuf(NestedTreePreviews &tree_previews, const bNode &node);
void node_release_preview_ibuf(NestedTreePreviews &tree_previews);
NestedTreePreviews *get_nested_previews(const bContext &C, SpaceNode &snode);
void stop_preview_job(wmWindowManager &wm);

}  // namespace blender::ed::space_node
