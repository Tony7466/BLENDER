/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2005 Blender Foundation */

#pragma once

/** \file
 * \ingroup bke
 */

#include "BLI_compiler_compat.h"
#include "BLI_ghash.h"

#include "DNA_listBase.h"

#include "BKE_node.h"

/* for FOREACH_NODETREE_BEGIN */
#include "DNA_node_types.h"

#include "RNA_types.h"

#include "BLI_map.hh"
#include "BLI_string_ref.hh"

namespace blender::bke {

/**
 * Set the mute status of a single link.
 */
void nodeLinkSetMute( bNodeTree *ntree,  bNodeLink *link, const bool muted);
bool nodeLinkIsSelected(const  bNodeLink *link);
void nodeInternalRelink( bNodeTree *ntree,  bNode *node);

void nodeToView(const  bNode *node, float x, float y, float *rx, float *ry);
void nodeFromView(const  bNode *node, float x, float y, float *rx, float *ry);

void nodePositionRelative( bNode *from_node,
                          const  bNode *to_node,
                          const  bNodeSocket *from_sock,
                          const  bNodeSocket *to_sock);


void nodePositionPropagate( bNode *node);

/**
 * \note Recursive.
 */
 bNode *nodeFindRootParent(bNode *node);

/**
 * Iterate over a chain of nodes, starting with \a node_start, executing
 * \a callback for each node (which can return false to end iterator).
 *
 * \param reversed: for backwards iteration
 * \note Recursive
 */
void nodeChainIter(const bNodeTree *ntree,
                   const bNode *node_start,
                   bool (*callback)(bNode *, bNode *, void *, const bool),
                   void *userdata,
                   bool reversed);

/**
 * Iterate over a chain of nodes, starting with \a node_start, executing
 * \a callback for each node (which can return false to end iterator).
 *
 * Faster than nodeChainIter. Iter only once per node.
 * Can be called recursively (using another nodeChainIterBackwards) by
 * setting the recursion_lvl accordingly.
 *
 * \note Needs updated socket links (ntreeUpdateTree).
 * \note Recursive
 */
void nodeChainIterBackwards(const bNodeTree *ntree,
                            const bNode *node_start,
                            bool (*callback)(bNode *, bNode *, void *),
                            void *userdata,
                            int recursion_lvl);
/**
 * Iterate over all parents of \a node, executing \a callback for each parent
 * (which can return false to end iterator)
 *
 * \note Recursive
 */
void nodeParentsIter(bNode *node, bool (*callback)(bNode *, void *), void *userdata);

/**
 * A dangling reroute node is a reroute node that does *not* have a "data source", i.e. no
 * non-reroute node is connected to its input.
 */
bool nodeIsDanglingReroute(const struct bNodeTree *ntree, const struct bNode *node);

struct bNodeLink *nodeFindLink(struct bNodeTree *ntree,
                               const struct bNodeSocket *from,
                               const struct bNodeSocket *to);

struct bNode *nodeGetActivePaintCanvas(struct bNodeTree *ntree);

}

namespace blender::bke {

/**
 * \brief Does the given node supports the sub active flag.
 *
 * \param sub_active: The active flag to check. #NODE_ACTIVE_TEXTURE / #NODE_ACTIVE_PAINT_CANVAS.
 */
bool nodeSupportsActiveFlag(const struct bNode *node, int sub_active);

void nodeSetSocketAvailability(struct bNodeTree *ntree,
                               struct bNodeSocket *sock,
                               bool is_available);

/**
 * If the node implements a `declare` function, this function makes sure that `node->declaration`
 * is up to date. It is expected that the sockets of the node are up to date already.
 */
bool nodeDeclarationEnsure(struct bNodeTree *ntree, struct bNode *node);
/**
 * Just update `node->declaration` if necessary. This can also be called on nodes that may not be
 * up to date (e.g. because the need versioning or are dynamic).
 */
bool nodeDeclarationEnsureOnOutdatedNode(struct bNodeTree *ntree, struct bNode *node);
/**
 * Update `socket->declaration` for all sockets in the node. This assumes that the node declaration
 * and sockets are up to date already.
 */
void nodeSocketDeclarationsUpdate(struct bNode *node);

}

namespace blender::bke {

typedef GHashIterator bNodeInstanceHashIterator;

BLI_INLINE bNodeInstanceHashIterator *node_instance_hash_iterator_new(bNodeInstanceHash *hash)
{
  return BLI_ghashIterator_new(hash->ghash);
}
BLI_INLINE void node_instance_hash_iterator_init(bNodeInstanceHashIterator *iter,
                                                     bNodeInstanceHash *hash)
{
  BLI_ghashIterator_init(iter, hash->ghash);
}
BLI_INLINE void node_instance_hash_iterator_free(bNodeInstanceHashIterator *iter)
{
  BLI_ghashIterator_free(iter);
}
BLI_INLINE bNodeInstanceKey node_instance_hash_iterator_get_key(bNodeInstanceHashIterator *iter)
{
  return *(bNodeInstanceKey *)BLI_ghashIterator_getKey(iter);
}
BLI_INLINE void *node_instance_hash_iterator_get_value(bNodeInstanceHashIterator *iter)
{
  return BLI_ghashIterator_getValue(iter);
}
BLI_INLINE void node_instance_hash_iterator_step(bNodeInstanceHashIterator *iter)
{
  BLI_ghashIterator_step(iter);
}
BLI_INLINE bool node_instance_hash_iterator_done(bNodeInstanceHashIterator *iter)
{
  return BLI_ghashIterator_done(iter);
}

#define NODE_INSTANCE_HASH_ITER(iter_, hash_) \
  for (blender::bke::node_instance_hash_iterator_init(&iter_, hash_); \
       blender::bke::node_instance_hash_iterator_done(&iter_) == false; \
       blender::bke::node_instance_hash_iterator_step(&iter_))

}

namespace blender::bke {

/* Node Previews */
bool node_preview_used(const  bNode *node);
bNodePreview *node_preview_verify(bNodeInstanceHash *previews, bNodeInstanceKey key, int xsize, int ysize, bool create);
bNodePreview *node_preview_copy( bNodePreview *preview);
void node_preview_free( bNodePreview *preview);

void node_preview_init_tree(bNodeTree *ntree, int xsize, int ysize);
void node_preview_remove_unused(bNodeTree *ntree);
void node_preview_clear(bNodePreview *preview);
void node_preview_merge_tree(bNodeTree *to_ntree,
                             bNodeTree *from_ntree,
                             bool remove_old);

/* -------------------------------------------------------------------- */
/** \name Node Type Access
 * \{ */
void nodeLabel(const bNodeTree *ntree, const bNode *node, char *label, int maxlen);

/**
 * Get node socket label if it is set.
 */
const char *nodeSocketLabel(const bNodeSocket *sock);

/**
 * Initialize a new node type struct with default values and callbacks.
 */
void node_type_base(bNodeType *ntype, int type, const char *name, short nclass);
void node_type_socket_templates(bNodeType *ntype,
                                bNodeSocketTemplate *inputs,
                                bNodeSocketTemplate *outputs);
void node_type_size(bNodeType *ntype, int width, int minwidth, int maxwidth);
void node_type_size_preset(bNodeType *ntype, eNodeSizePreset size);

/* -------------------------------------------------------------------- */
/** \name Node Generic Functions
 * \{ */

bool node_is_connected_to_output(const struct bNodeTree *ntree, const struct bNode *node);

}  // namespace blender::bke

namespace blender::bke {

bNodeSocket *node_find_enabled_socket(bNode &node, eNodeSocketInOut in_out, StringRef name);
bNodeSocket *node_find_enabled_input_socket(bNode &node, StringRef name);
bNodeSocket *node_find_enabled_output_socket(bNode &node, StringRef name);

}  // namespace blender::bke

#define NODE_STORAGE_FUNCS(StorageT) \
  [[maybe_unused]] static StorageT &node_storage(bNode &node) \
  { \
    return *static_cast<StorageT *>(node.storage); \
  } \
  [[maybe_unused]] static const StorageT &node_storage(const bNode &node) \
  { \
    return *static_cast<const StorageT *>(node.storage); \
  }
