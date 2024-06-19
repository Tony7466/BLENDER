/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <string>

#include "BLI_set.hh"

#include "BKE_node.hh"

#include "DNA_node_types.h"

namespace blender::bke::compositor {

/* Adds the pass names of the passes used by the given Render Layer node to the given used passes.
 * This essentially adds the identifiers of the outputs that are logically linked, their the
 * identifiers are the names of the passes. Note however that the Image output is actually the
 * Combined pass, but named as Image for compatibility reasons. */
static void add_passes_used_by_render_layer_node(const bNode *node, Set<std::string> &used_passes)
{
  for (const bNodeSocket *output : node->output_sockets()) {
    if (output->is_logically_linked()) {
      if (std::string(output->identifier) == "Image") {
        used_passes.add(RE_PASSNAME_COMBINED);
      }
      else {
        used_passes.add(output->identifier);
      }
    }
  }
}

/* Adds the pass names of the passes used by the given compositor node tree to the given used
 * passes. This is called recursively for node groups. */
static void add_used_passes_recursive(const bNodeTree *node_tree, Set<std::string> &used_passes)
{
  if (node_tree == nullptr) {
    return;
  }

  node_tree->ensure_topology_cache();
  for (const bNode *node : node_tree->all_nodes()) {
    if (node->is_muted()) {
      continue;
    }

    switch (node->type) {
      case NODE_GROUP:
      case NODE_CUSTOM_GROUP:
        add_used_passes_recursive(reinterpret_cast<const bNodeTree *>(node->id), used_passes);
        break;
      case CMP_NODE_R_LAYERS:
        add_passes_used_by_render_layer_node(node, used_passes);
        break;
      default:
        break;
    }
  }
}

Set<std::string> get_used_passes(const Scene &scene)
{
  Set<std::string> used_passes;
  add_used_passes_recursive(scene.nodetree, used_passes);
  return used_passes;
}

}  // namespace blender::bke::compositor
