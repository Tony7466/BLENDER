/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_function_ref.hh"
#include "BLI_implicit_sharing.hh"
#include "BLI_memory_counter_fwd.hh"
#include "BLI_set.hh"

namespace blender {

/**
 * #MemoryCounter helps counting the amount of memory used in cases where data is shared and should
 * not be double-counted. To achieve that, it counts bytes that are uniquely owned separately from
 * those that are shared.
 */
class MemoryCounter : NonCopyable, NonMovable {
 private:
  int64_t owned_bytes_ = 0;
  Set<const ImplicitSharingInfo *> counted_shared_data_;

 public:
  MemoryCounter() = default;

  ~MemoryCounter()
  {
    for (const ImplicitSharingInfo *sharing_info : counted_shared_data_) {
      sharing_info->remove_weak_user_and_delete_if_last();
    }
  }

  void add(const int64_t bytes)
  {
    owned_bytes_ += bytes;
  }

  void add_shared(const ImplicitSharingInfo *sharing_info,
                  const FunctionRef<void(MemoryCounter &memory)> count_fn)
  {
    if (!sharing_info) {
      /* Data is not actually shared. */
      count_fn(*this);
      return;
    }
    if (!counted_shared_data_.add(sharing_info)) {
      /* Data was counted before, avoid counting it again. */
      return;
    }
    sharing_info->add_weak_user();
    /* Count into the `this` for now. In the future we could pass in a separate #MemoryCounter here
     * if we needed to know the amount of memory used by each shared data. */
    count_fn(*this);
  }

  void add_shared(const ImplicitSharingInfo *sharing_info, const int64_t bytes)
  {
    this->add_shared(sharing_info,
                     [&](MemoryCounter &shared_memory) { shared_memory.add(bytes); });
  }

  int64_t total_bytes() const
  {
    return owned_bytes_;
  }
};

}  // namespace blender
