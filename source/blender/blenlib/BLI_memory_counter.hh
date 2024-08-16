/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_function_ref.hh"
#include "BLI_implicit_sharing_ptr.hh"
#include "BLI_map.hh"
#include "BLI_memory_counter_fwd.hh"
#include "BLI_set.hh"
#include "BLI_vector_set.hh"

namespace blender::memory_counter {

struct SharedDataInfo {
  WeakImplicitSharingPtr sharing_info;
  int64_t uniquely_owned_bytes = 0;
  VectorSet<const ImplicitSharingInfo *> parents;
  VectorSet<const ImplicitSharingInfo *> children;
};

struct MemoryBySharedData {
  Map<const ImplicitSharingInfo *, SharedDataInfo> map;

  bool ensure(const ImplicitSharingInfo *sharing_info)
  {
    bool newly_added = false;
    map.lookup_or_add_cb(sharing_info, [&]() {
      newly_added = true;
      if (sharing_info) {
        sharing_info->add_weak_user();
      }
      return SharedDataInfo{WeakImplicitSharingPtr{sharing_info}};
    });
    return newly_added;
  }
};

/**
 * #MemoryCounter helps counting the amount of memory used in cases where data is shared and should
 * not be double-counted. To achieve that, it counts bytes that are uniquely owned separately from
 * those that are shared.
 */
class MemoryCounter : NonCopyable, NonMovable {
 private:
  MemoryBySharedData &memory_by_shared_data_;
  const ImplicitSharingInfo *current_sharing_info_ = nullptr;
  int64_t newly_added_bytes_ = 0;

 public:
  MemoryCounter(MemoryBySharedData &memory_by_shared_data,
                const ImplicitSharingInfo *current_sharing_info = nullptr);

  /**
   * Add bytes that are uniquely owned, i.e. not shared.
   */
  void add(const int64_t bytes);

  /**
   * Add (potentially) shared data which should not be counted twice.
   *
   * \param sharing_info: The sharing info that owns the data. If null, the data is assumed to be
   *   uniquely owned.
   * \param count_fn: Function to count the amount of memory used by the shared data. This is only
   *   called if the shared memory was not counted before. It's a callback, because sometimes
   *   counting the amount of used memory can be a bit more involved.
   */
  void add_shared(const ImplicitSharingInfo *sharing_info,
                  const FunctionRef<void(MemoryCounter &shared_memory)> count_fn);

  /**
   * Same as above, but takes in the number of bytes directly instead of callback. This is easier
   * to use in cases where computing the number of bytes is very cheap.
   */
  void add_shared(const ImplicitSharingInfo *sharing_info, const int64_t bytes);

  /**
   * Get the total number of counted bytes.
   *
   * \note This is only a rough estimate of the actual used memory. Often, not every little bit of
   * memory is counted, so this is generally a lower bound. The actual memory usage should not be
   * significantly higher though.
   */
  int64_t counted_bytes() const;

  int64_t newly_added_bytes() const
  {
    return newly_added_bytes_;
  }
};

int64_t compute_total_bytes(const SharedDataInfo &data,
                            const MemoryBySharedData &memory_by_shared_data);

}  // namespace blender::memory_counter
