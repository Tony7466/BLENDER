/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "RE_pipeline.h"

#include "IMB_imbuf.h"

struct bContext;
struct bNodeTree;
struct ImBuf;
struct Render;

struct NestedTreePreviews {
  Render *previews_render;
  /* Use this map to keep track of the latest ImBuf used (after freeing the renderresult). */
  blender::Map<const int32_t, std::pair<ImBuf *, uint32_t>> previews_map;
  int preview_size;
  bool rendering;
  bool restart_needed;
  bool partial_tree_refresh;
  /* Dirty state of the bNodeTreePath vector. It is the sum of the tree_dirty_state of all the
   * nodetrees plus the sum of all the dirty_state of the group nodes.*/
  uint32_t treepath_dirty_state;
  /* Dirty state of the nodetree viewed, it is used to know if any node needs to be re-rendered. */
  uint32_t nodes_dirty_state;
  NestedTreePreviews(const int size)
      : previews_render(nullptr),
        preview_size(size),
        rendering(false),
        restart_needed(false),
        partial_tree_refresh(false),
        treepath_dirty_state(-1),
        nodes_dirty_state(-1)
  {
  }
  ~NestedTreePreviews()
  {
    if (this->previews_render) {
      RE_FreeRender(this->previews_render);
    }
    this->previews_map.foreach_item([&](const int32_t, const std::pair<ImBuf *, uint32_t> &cache) {
      IMB_freeImBuf(cache.first);
    });
  }
};

void ED_spacenode_free_previews(wmWindowManager *wm, SpaceNode *snode);
ImBuf *ED_node_preview_acquire_ibuf(bNodeTree *ntree,
                                    NestedTreePreviews *tree_previews,
                                    const bNode *node);
void ED_node_release_preview_ibuf(NestedTreePreviews *tree_previews);
NestedTreePreviews *ED_spacenode_get_nested_previews(const bContext *ctx, SpaceNode *sn);
void ED_spacenode_stop_preview_job(wmWindowManager *wm);
