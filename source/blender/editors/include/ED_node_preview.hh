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
  blender::Map<const bNode *, ImBuf *> previews_map;
  int preview_size;
  bool rendering;
  bool restart_needed;
  uint32_t previews_refresh_state;
  NestedTreePreviews(const int size)
      : previews_render(nullptr),
        preview_size(size),
        rendering(false),
        restart_needed(false),
        previews_refresh_state(0)
  {
  }
  ~NestedTreePreviews()
  {
    if (this->previews_render) {
      RE_FreeRender(this->previews_render);
    }
    this->previews_map.foreach_item([&](const bNode *, ImBuf *ibuf) { IMB_freeImBuf(ibuf); });
  }
};

void ED_spacenode_free_previews(wmWindowManager *wm, SpaceNode *snode);
ImBuf *ED_node_preview_acquire_ibuf(bNodeTree *ntree,
                                    NestedTreePreviews *tree_previews,
                                    const bNode *node);
void ED_node_release_preview_ibuf(NestedTreePreviews *tree_previews);
NestedTreePreviews *ED_spacenode_get_nested_previews(const bContext *ctx, SpaceNode *sn);
