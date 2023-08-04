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
  Render *previews_render = nullptr;
  /* Use this map to keep track of the latest ImBuf used (after freeing the renderresult). */
  blender::Map<const bNode *, ImBuf *> previews_map;
  int preview_size;
  bool rendering = false;
  bool restart_needed = false;
  uint32_t previews_refresh_state = 0;
  NestedTreePreviews(const int size) : preview_size(size) {}
  ~NestedTreePreviews()
  {
    if (this->previews_render) {
      RE_FreeRender(this->previews_render);
    }
    for (ImBuf *ibuf : this->previews_map.values()) {
      IMB_freeImBuf(ibuf);
    }
  }
};

void ED_spacenode_free_previews(wmWindowManager &wm, SpaceNode &snode);
ImBuf *ED_node_preview_acquire_ibuf(bNodeTree &ntree,
                                    NestedTreePreviews &tree_previews,
                                    const bNode &node);
void ED_node_release_preview_ibuf(NestedTreePreviews &tree_previews);
NestedTreePreviews *ED_spacenode_get_nested_previews(const bContext &C, SpaceNode &sn);
void ED_spacenode_stop_preview_job(wmWindowManager &wm);
