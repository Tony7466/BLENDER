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
};

class GeometryNodesBakes {
 public:
  Map<std::pair<ComputeContextHash, int32_t>, BakeNodeStorage *> storage_by_compute_context_;

  BakeNodeStorage *get_storage(const ComputeContextHash &compute_context,
                               const int32_t bake_node_id)
  {
    return storage_by_compute_context_.lookup_default({compute_context, bake_node_id}, nullptr);
  }
};

class GeometryNodesModifierBakes {
 public:
  Map<int32_t, std::unique_ptr<BakeNodeStorage>> storage_by_id;
  Vector<int32_t> requested_bake_ids;
};

}  // namespace blender::bke
