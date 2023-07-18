/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2005 Blender Foundation */

/** \file
 * \ingroup editors
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

struct bContext;
struct bNodeTree;
struct ImBuf;

struct NestedNodePreviewMap {
  blender::Map<int, NodePreviewImage> node_previews;
  int pr_size;
  bool rendering;
  bool restart_needed;
  uint16_t preview_refresh_state;

  ~NestedNodePreviewMap()
  {
    for (NodePreviewImage image : node_previews.values()) {
      MEM_freeN(image.rect);
    }
  }
};

void ED_spacenode_free_previews(SpaceNode *sn);
ImBuf *ED_node_get_preview_ibuf(bNodeTree *ntree, NestedNodePreviewMap *data, const bNode *node);
NestedNodePreviewMap *ED_spacenode_get_nested_previews(const bContext *ctx, SpaceNode *sn);

#ifdef __cplusplus
}
#endif
