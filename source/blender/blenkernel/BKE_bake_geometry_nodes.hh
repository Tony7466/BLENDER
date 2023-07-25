/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_compute_context.hh"
#include "BLI_map.hh"
#include "BLI_sub_frame.hh"

#include "BKE_bake_items.hh"

namespace blender::bke {

class BakeNodeState {
 public:
  Map<int, std::unique_ptr<BakeItem>> item_by_identifier;
};

class BakeNodeStateAtFrame {
 public:
  SubFrame frame;
  std::unique_ptr<BakeNodeState> state;
};

class BakeNodeStorage {
 public:
  Vector<BakeNodeStateAtFrame> states;
  std::unique_ptr<BakeNodeState> current_bake_state;
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
