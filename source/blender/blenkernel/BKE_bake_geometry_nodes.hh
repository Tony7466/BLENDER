/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_compute_context.hh"
#include "BLI_map.hh"

#include "BKE_geometry_set.hh"

namespace blender::bke {

class BakeNodeStorage {
 public:
  std::optional<GeometrySet> geometry;
  /**
   * Store data during baking here to avoid race conditions with other nodes that access the
   * geometry above.
   */
  std::optional<GeometrySet> newly_baked_geometry;
};

class GeometryNodesModifierBakes {
 public:
  Map<int32_t, std::unique_ptr<BakeNodeStorage>> storage_by_id;
  Set<int32_t> requested_bake_ids;

  BakeNodeStorage *get_storage(const int32_t nested_node_id)
  {
    std::unique_ptr<BakeNodeStorage> *storage = this->storage_by_id.lookup_ptr(nested_node_id);
    if (storage) {
      return storage->get();
    }
    return nullptr;
  }
};

}  // namespace blender::bke
