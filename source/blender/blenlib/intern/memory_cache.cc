/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include <atomic>
#include <mutex>
#include <tbb/concurrent_hash_map.h>

#include "BLI_memory_cache.hh"
#include "BLI_memory_counter.hh"

namespace blender::memory_cache {

struct StoredValue {
  std::unique_ptr<const GenericKey> key;
  std::shared_ptr<CachedValue> value;
  int64_t last_use_time = 0;
};

struct Hasher {
  size_t hash(const GenericKey &key) const
  {
    return key.hash();
  }

  bool equal(const GenericKey &a, const GenericKey &b) const
  {
    return a == b;
  }
};

using ConcurrentMap =
    tbb::concurrent_hash_map<std::reference_wrapper<const GenericKey>, StoredValue, Hasher>;

struct Cache {
  std::atomic<int64_t> logical_time = 0;
  std::atomic<int64_t> size_in_bytes = 0;
  ConcurrentMap map;

  std::mutex global_mutex;
  MemoryCount memory;
  Vector<const GenericKey *> keys;
};

static Cache &get_cache()
{
  static Cache cache;
  return cache;
}

std::shared_ptr<CachedValue> get_base(const GenericKey &key,
                                      const FunctionRef<std::unique_ptr<CachedValue>()> compute_fn)
{
  Cache &cache = get_cache();
  std::shared_ptr<CachedValue> result;
  {
    ConcurrentMap::accessor accessor;
    const bool newly_inserted = cache.map.insert(accessor, std::ref(key));

    if (newly_inserted) {
      accessor->second.key = key.to_storable();
      accessor->second.value = compute_fn();
      /* Modifying the key should be fine because the new key is equal to the original key. */
      const_cast<std::reference_wrapper<const GenericKey> &>(accessor->first) = std::ref(
          *accessor->second.key);

      {
        std::lock_guard lock{cache.global_mutex};
        memory_counter::MemoryCounter memory_counter{cache.memory};
        accessor->second.value->count_memory(memory_counter);
        cache.keys.append(&accessor->first.get());
        cache.size_in_bytes = cache.memory.total_bytes;
      }
    }

    /* Update time this was last used, so that it is not freed. */
    accessor->second.last_use_time = cache.logical_time.fetch_add(1, std::memory_order_relaxed);
    result = accessor->second.value;
  }
  free_to_fit(int64_t(4) * 1024 * 1024 * 1024);
  return result;
}

void free_to_fit(const int64_t capacity)
{
  Cache &cache = get_cache();
  if (cache.size_in_bytes < capacity) {
    return;
  }
  std::lock_guard lock{cache.global_mutex};
  Vector<std::pair<int64_t, const GenericKey *>> keys_with_time;
  for (const GenericKey *key : cache.keys) {
    ConcurrentMap::const_accessor accessor;
    if (!cache.map.find(accessor, *key)) {
      continue;
    }
    keys_with_time.append({accessor->second.last_use_time, key});
  }
  std::sort(keys_with_time.begin(), keys_with_time.end());
  std::reverse(keys_with_time.begin(), keys_with_time.end());

  /* Count used memory starting at the most recently cached element. Stop at the element when the
   * amount became larger than the capacity. */
  cache.memory.reset();
  MemoryCounter memory_counter{cache.memory};
  std::optional<int> first_bad_index;
  for (const int i : keys_with_time.index_range()) {
    const GenericKey &key = *keys_with_time[i].second;
    ConcurrentMap::const_accessor accessor;
    if (!cache.map.find(accessor, key)) {
      continue;
    }
    accessor->second.value->count_memory(memory_counter);
    if (cache.memory.total_bytes <= capacity) {
      continue;
    }
    first_bad_index = i;
    break;
  }
  if (!first_bad_index) {
    return;
  }

  /* Avoid recounting memory if the last item is not way too large and the overshoot is still ok.
   * The alternative would be to subtract the last item from the counted memory again, but that is
   * not implemented yet. */
  bool need_memory_recount = false;
  if (cache.memory.total_bytes < capacity * 1.25) {
    *first_bad_index += 1;
    if (*first_bad_index == keys_with_time.size()) {
      return;
    }
  }
  else {
    need_memory_recount = true;
  }

  /* Remove elements that don't fit anymore. */
  for (const int i : keys_with_time.index_range().drop_front(*first_bad_index)) {
    const GenericKey &key = *keys_with_time[i].second;
    cache.map.erase(key);
  }
  cache.keys.clear();
  for (const int i : keys_with_time.index_range().take_front(*first_bad_index)) {
    cache.keys.append(keys_with_time[i].second);
  }

  if (need_memory_recount) {
    cache.memory.reset();
    MemoryCounter memory_counter{cache.memory};
    for (const int i : keys_with_time.index_range().take_front(*first_bad_index)) {
      const GenericKey &key = *keys_with_time[i].second;
      ConcurrentMap::const_accessor accessor;
      if (!cache.map.find(accessor, key)) {
        continue;
      }
      accessor->second.value->count_memory(memory_counter);
    }
  }
}

}  // namespace blender::memory_cache
