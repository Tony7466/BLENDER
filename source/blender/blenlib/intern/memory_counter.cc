/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_memory_counter.hh"

namespace blender::memory_counter {

MemoryCounter::MemoryCounter(OwnedMemory &top_level, MemoryBySharedData &memory_by_shared_data)
    : top_level_(top_level), memory_by_shared_data_(memory_by_shared_data)
{
}

void MemoryCounter::add_shared(const ImplicitSharingInfo *sharing_info,
                               const FunctionRef<void(MemoryCounter &memory)> count_fn)
{
  if (!sharing_info) {
    /* Data is not actually shared. */
    count_fn(*this);
    return;
  }
  /* Remember that this shared data is used. */
  if (top_level_.used_shared_data.add_as(sharing_info)) {
    sharing_info->add_weak_user();
  }
  if (memory_by_shared_data_.map.contains_as(sharing_info)) {
    /* Data was counted before, avoid counting it again. */
    return;
  }

  OwnedMemory shared_memory;
  MemoryCounter shared_memory_counter{shared_memory, memory_by_shared_data_};
  count_fn(shared_memory_counter);

  memory_by_shared_data_.map.add_as(sharing_info, std::move(shared_memory));
  sharing_info->add_weak_user();
}

void MemoryCounter::add_shared(const ImplicitSharingInfo *sharing_info, const int64_t bytes)
{
  this->add_shared(sharing_info, [&](MemoryCounter &shared_memory) { shared_memory.add(bytes); });
}

int64_t MemoryCounter::counted_bytes() const
{
  return compute_total_bytes(top_level_, memory_by_shared_data_);
}

int64_t compute_total_bytes(const OwnedMemory &memory,
                            const MemoryBySharedData &memory_by_shared_data)
{
  int64_t count = memory.uniquely_owned_bytes;
  for (const WeakImplicitSharingPtr &sharing_info : memory.used_shared_data) {
    count += memory_by_shared_data.map.lookup_as(&*sharing_info).uniquely_owned_bytes;
  }
  return count;
}

}  // namespace blender::memory_counter
