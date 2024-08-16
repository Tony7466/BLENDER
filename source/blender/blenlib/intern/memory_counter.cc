/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_memory_counter.hh"

namespace blender::memory_counter {

MemoryCounter::MemoryCounter(MemoryBySharedData &memory_by_shared_data,
                             const ImplicitSharingInfo *current_sharing_info)
    : memory_by_shared_data_(memory_by_shared_data), current_sharing_info_(current_sharing_info)
{
  memory_by_shared_data_.ensure(current_sharing_info);
}

void MemoryCounter::add(const int64_t bytes)
{
  memory_by_shared_data_.map.lookup_or_add_default_as(current_sharing_info_)
      .uniquely_owned_bytes += bytes;
  newly_added_bytes_ += bytes;
}

void MemoryCounter::add_shared(const ImplicitSharingInfo *sharing_info,
                               const FunctionRef<void(MemoryCounter &memory)> count_fn)
{
  if (!sharing_info) {
    /* Data is not actually shared. */
    count_fn(*this);
    return;
  }
  const bool newly_added = memory_by_shared_data_.ensure(sharing_info);

  memory_by_shared_data_.map.lookup(current_sharing_info_).children.add(sharing_info);
  memory_by_shared_data_.map.lookup(sharing_info).parents.add(current_sharing_info_);

  if (!newly_added) {
    /* Data was counted before, avoid counting it again. */
    return;
  }

  MemoryCounter shared_memory{memory_by_shared_data_, sharing_info};
  count_fn(shared_memory);
  newly_added_bytes_ += shared_memory.newly_added_bytes_;
}

void MemoryCounter::add_shared(const ImplicitSharingInfo *sharing_info, const int64_t bytes)
{
  this->add_shared(sharing_info, [&](MemoryCounter &shared_memory) { shared_memory.add(bytes); });
}

int64_t MemoryCounter::counted_bytes() const
{
  return compute_total_bytes(memory_by_shared_data_.map.lookup(current_sharing_info_),
                             memory_by_shared_data_);
}

static void gather_all_nested_sharing_info(const ImplicitSharingInfo &sharing_info,
                                           const MemoryBySharedData &memory_by_shared_data,
                                           Set<const ImplicitSharingInfo *> &r_all)
{
  if (r_all.add(&sharing_info)) {
    for (const ImplicitSharingInfo *child_sharing_info :
         memory_by_shared_data.map.lookup(&sharing_info).children)
    {
      gather_all_nested_sharing_info(*child_sharing_info, memory_by_shared_data, r_all);
    }
  }
}

int64_t compute_total_bytes(const SharedDataInfo &data,
                            const MemoryBySharedData &memory_by_shared_data)
{
  Set<const ImplicitSharingInfo *> all;
  for (const ImplicitSharingInfo *child_sharing_info : data.children) {
    gather_all_nested_sharing_info(*child_sharing_info, memory_by_shared_data, all);
  }

  int64_t count = data.uniquely_owned_bytes;
  for (const ImplicitSharingInfo *sharing_info : all) {
    count += memory_by_shared_data.map.lookup(sharing_info).uniquely_owned_bytes;
  }
  return count;
}

}  // namespace blender::memory_counter
