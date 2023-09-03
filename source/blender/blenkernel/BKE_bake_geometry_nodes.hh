/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <mutex>

#include "BLI_compute_context.hh"
#include "BLI_map.hh"
#include "BLI_sub_frame.hh"

#include "BKE_bake_items.hh"
#include "BKE_bake_items_serialize.hh"

namespace blender::bke {

class BakeNodeState {
 public:
  Map<int, std::unique_ptr<BakeItem>> item_by_identifier;
  std::mutex mutex;
  std::optional<std::string> meta_path;
};

class BakeNodeStateAtFrame {
 public:
  SubFrame frame;
  std::unique_ptr<BakeNodeState> state;
};

class BakeNodeStorage {
 public:
  std::mutex mutex;
  Vector<BakeNodeStateAtFrame> states;
  std::unique_ptr<BakeNodeState> current_bake_state;

  std::unique_ptr<BlobSharing> blob_sharing;
  std::optional<std::string> blobs_dir;
};

class GeometryNodesModifierBakes {
 public:
  std::mutex mutex;
  Map<int32_t, std::unique_ptr<BakeNodeStorage>> storage_by_id;

  const BakeNodeStorage *get_storage(const int32_t bake_id) const
  {
    const std::unique_ptr<BakeNodeStorage> *storage = this->storage_by_id.lookup_ptr(bake_id);
    if (storage) {
      return storage->get();
    }
    return nullptr;
  }

  BakeNodeStorage &get_storage_for_write(const int32_t bake_id)
  {
    std::lock_guard lock{this->mutex};
    return *this->storage_by_id.lookup_or_add_cb(
        bake_id, []() { return std::make_unique<BakeNodeStorage>(); });
  }
};

}  // namespace blender::bke
