/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include "DNA_node_types.h"

#include "BLI_vector.hh"

namespace blender::bke::node_tree_zones {

struct TreeZone {
  TreeZones *owner = nullptr;
  /** Index of the zone in the array of all zones in a node tree. */
  int index = -1;
  /** Zero for top level zones, one for a nested zone, and so on. */
  int depth = -1;
  /** Input node of the zone. */
  const bNode *input_node = nullptr;
  /** Output node of the zone. */
  const bNode *output_node = nullptr;
  /** Direct parent of the zone. If this is null, this is a top level zone. */
  TreeZone *parent_zone = nullptr;
  /** Direct children zones. Does not contain recursively nested zones. */
  Vector<TreeZone *> child_zones;
  /** Direct children nodes. Does not contain recursively nested nodes. */
  Vector<const bNode *> child_nodes;

  bool contains_node_recursively(const bNode &node) const;
  bool contains_zone_recursively(const int zone_i) const;
};

class TreeZones {
 public:
  Vector<std::unique_ptr<TreeZone>> zones;
  Map<int, int> parent_zone_by_node_id;

  const TreeZone *get_zone_by_node(const bNode &node) const
  {
    const int zone_i = this->parent_zone_by_node_id.lookup_default(node.identifier, -1);
    if (zone_i == -1) {
      return nullptr;
    }
    return this->zones[zone_i].get();
  }

  const TreeZone *get_zone_by_socket(const bNodeSocket &socket) const
  {
    const bNode &node = socket.owner_node();
    for (const std::unique_ptr<TreeZone> &zone : zones) {
      if (socket.in_out == SOCK_IN) {
        if (zone->output_node == &node) {
          return zone.get();
        }
      }
      else {
        if (zone->input_node == &node) {
          return zone.get();
        }
      }
    }
    return this->get_zone_by_node(node);
  }
};

const TreeZones *get_tree_zones(const bNodeTree &tree);

}  // namespace blender::bke::node_tree_zones
