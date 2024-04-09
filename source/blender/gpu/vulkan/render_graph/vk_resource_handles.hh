/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_map.hh"
#include "BLI_vector.hh"

#include "vk_common.hh"

namespace blender::gpu::render_graph {

/**
 * List for working with handles and items.
 * Reference to the first empty slot is stored internally to stop iterating over all the elements.
 */
template<typename Handle, typename Item> class VKResourceHandles {
 private:
  const int64_t grow_size_ = 64;
  Handle first_empty_slot_ = 0;
  Vector<std::optional<Item>> items_;

 public:
  /**
   * Allocate a new handle.
   *
   * It can reuse a previous freed handle.
   */
  Handle allocate()
  {
    Handle handle = first_empty_slot_;
    if (handle >= items_.size()) {
      items_.append_n_times(std::nullopt, grow_size_);
    }
    Item new_item = {};
    items_[handle] = std::move(new_item);
    mark_filled(handle);
    return handle;
  }

  const Item &get(Handle handle) const
  {
    BLI_assert(items_[handle].has_value());
    return *items_[handle];
  }

  Span<const std::optional<Item>> as_span() const
  {
    return Span<const std::optional<Item>>(items_.data(), size());
  }

  Item &get(Handle handle)
  {
    BLI_assert(items_[handle].has_value());
    return *items_[handle];
  }

  void free(const Handle handle)
  {
    items_[handle] = std::nullopt;
    first_empty_slot_ = min_ii(first_empty_slot_, handle);
  }

  int64_t size() const
  {
    return items_.size();
  }

 private:
  void mark_filled(Handle handle)
  {
    BLI_assert(handle >= first_empty_slot_);

    /* Early exit when handle isn't set the first empty slot in this case the first empty slot
     * doesn't change. */
    if (handle != first_empty_slot_) {
      return;
    }
    for (Handle handle : IndexRange(first_empty_slot_, items_.size())) {
      if (items_[handle].has_value()) {
        continue;
      }
      first_empty_slot_ = handle;
      return;
    }
    /* No empty slot found, set first empty slot to the end of the list. */
    first_empty_slot_ = items_.size();
  }
};

}  // namespace blender::gpu::render_graph
