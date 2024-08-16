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

namespace blender::memory_cache2 {

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
  ConcurrentMap map;

  std::mutex global_mutex;
  memory_counter::MemoryBySharedData memory_by_shared_data;
  std::atomic<int64_t> cache_size = 0;
  Vector<const GenericKey *> keys;
};

static Cache &get_cache()
{
  static Cache cache;
  return cache;
}

std::shared_ptr<CachedValue> get(const GenericKey &key,
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
        memory_counter::MemoryCounter memory_counter{cache.memory_by_shared_data, nullptr};
        accessor->second.value->count_memory(memory_counter);
        cache.cache_size += memory_counter.newly_added_bytes();
        cache.memory_by_shared_data.map.remove(nullptr);
        cache.keys.append(&accessor->first.get());
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
  if (cache.cache_size < capacity) {
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

  memory_counter::MemoryBySharedData local_memory_by_shared_data;
  MemoryCounter memory{local_memory_by_shared_data, nullptr};
  std::optional<int> first_bad_index;
  int64_t last_valid_size = 0;

  Vector<const GenericKey *> new_keys;
  for (const int i : keys_with_time.index_range()) {
    const GenericKey &key = *keys_with_time[i].second;
    ConcurrentMap::const_accessor accessor;
    if (!cache.map.find(accessor, key)) {
      continue;
    }
    accessor->second.value->count_memory(memory);
    const int64_t size_up_to_now = memory.newly_added_bytes();
    if (size_up_to_now <= capacity) {
      last_valid_size = size_up_to_now;
      new_keys.append(&key);
      continue;
    }
    first_bad_index = i;
    break;
  }
  if (!first_bad_index) {
    return;
  }
  for (const int i : keys_with_time.index_range().drop_front(*first_bad_index)) {
    const GenericKey &key = *keys_with_time[i].second;
    cache.map.erase(key);
  }
  cache.cache_size = last_valid_size;
  cache.keys = std::move(new_keys);

  {
    cache.memory_by_shared_data.map.clear();
    MemoryCounter memory{cache.memory_by_shared_data, nullptr};
    for (const int i : keys_with_time.index_range().take_front(*first_bad_index)) {
      const GenericKey &key = *keys_with_time[i].second;
      ConcurrentMap::const_accessor accessor;
      if (!cache.map.find(accessor, key)) {
        continue;
      }
      accessor->second.value->count_memory(memory);
    }
  }
}

}  // namespace blender::memory_cache2
