/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "atomic_ops.h"

#include "BLI_atomic_disjoint_set.hh"
#include "BLI_task.hh"

namespace blender {

AtomicDisjointSet::AtomicDisjointSet(const int size) : items_(size)
{
  threading::parallel_for(IndexRange(size), 4096, [&](const IndexRange range) {
    for (const int i : range) {
      items_[i].store(Item{i, 0}, relaxed);
    }
  });
}

void AtomicDisjointSet::calc_reduced_ids(MutableSpan<int> result) const
{
  BLI_assert(result.size() == items_.size());
  const int size = result.size();

  const int bad_index = size;
  Array<int> least_root_index(size, bad_index);

  /* Root index is the lowest index of item, joined with this root.
   * Parallel sort of root indices to make them deterministic. */
  threading::parallel_for(IndexRange(size), 1024, [&](const IndexRange range) {
    for (const int index : range) {
      const int root = this->find_root(index);
      while (true) {
        const int other_index = atomic_load_int32(&least_root_index[root]);
        if (index < other_index) {
          if (atomic_cas_int32(&least_root_index[root], other_index, index) == other_index) {
            break;
          }
        }
        break;
      }
    }
  });

  threading::parallel_for(IndexRange(size), 1024, [&](const IndexRange range) {
    for (const int i : range) {
      const int root = this->find_root(i);
      result[i] = least_root_index[root];
    }
  });
}

int AtomicDisjointSet::count_sets() const
{
  return threading::parallel_reduce<int>(
      items_.index_range(),
      1024,
      0,
      [&](const IndexRange range, int count) {
        for (const int i : range) {
          if (this->is_root(i)) {
            count++;
          }
        }
        return count;
      },
      [](const int a, const int b) { return a + b; });
}

}  // namespace blender
