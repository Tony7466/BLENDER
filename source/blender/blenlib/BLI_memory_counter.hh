/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_function_ref.hh"
#include "BLI_implicit_sharing.hh"
#include "BLI_map.hh"
#include "BLI_memory_counter_fwd.hh"

namespace blender {

/**
 * #MemoryCounter helps counting the amount of memory used in cases where data is shared and should
 * not be double-counted. To achieve that, it counts bytes that are uniquely owned separately from
 * those that are shared.
 */
class MemoryCounter : NonCopyable, NonMovable {
 private:
  int64_t uniquely_owned_bytes_ = 0;
  Map<const ImplicitSharingInfo *, int64_t> shared_bytes_;

 public:
  MemoryCounter() = default;

  ~MemoryCounter()
  {
    for (const ImplicitSharingInfo *sharing_info : shared_bytes_.keys()) {
      sharing_info->remove_weak_user_and_delete_if_last();
    }
  }

  void add(const int64_t bytes)
  {
    uniquely_owned_bytes_ += bytes;
  }

  void add_shared(const ImplicitSharingInfo *sharing_info,
                  const void *data,
                  const FunctionRef<int64_t()> get_size_fn)
  {
    if (!data) {
      return;
    }
    if (!sharing_info) {
      uniquely_owned_bytes_ = get_size_fn();
      return;
    }
    sharing_info->add_weak_user();
    shared_bytes_.lookup_or_add_cb(sharing_info, get_size_fn);
  }

  int64_t total_bytes() const
  {
    int64_t bytes = uniquely_owned_bytes_;
    for (const int64_t shared_bytes : shared_bytes_.values()) {
      bytes += shared_bytes;
    }
    return bytes;
  }
};

}  // namespace blender
