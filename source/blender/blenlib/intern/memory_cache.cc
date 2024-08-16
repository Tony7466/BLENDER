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
#include "BLI_task.hh"

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
  std::atomic<int64_t> approximate_limit = 1024 * 1024 * 1024;
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

static void try_enforce_limit();

static void set_new_logical_time(const StoredValue &stored_value, const int64_t new_time)
{
  /* Don't want to use `std::atomic` directory in the struct, because that makes it
   * non-movable. Could also use a non-const accessor, but that may degrade performance more.
   * It's not necessary for correctness that the time is exactly the right value. */
  reinterpret_cast<std::atomic<int64_t> *>(const_cast<int64_t *>(&stored_value.last_use_time))
      ->store(new_time, std::memory_order_relaxed);
}

std::shared_ptr<CachedValue> get_base(const GenericKey &key,
                                      const FunctionRef<std::unique_ptr<CachedValue>()> compute_fn)
{
  Cache &cache = get_cache();
  /* "Touch" the cached value so that we know that it is still used. This makes it less likely that
   * it removed. */
  const int64_t new_time = cache.logical_time.fetch_add(1, std::memory_order_relaxed);
  {
    /* Fast path when the value is already cached. */
    ConcurrentMap::const_accessor accessor;
    if (cache.map.find(accessor, std::ref(key))) {
      set_new_logical_time(accessor->second, new_time);
      return accessor->second.value;
    }
  }

  /* Compute value while no locks are held to avoid potential for dead-locks. Not using a lock also
   * means that the value may be computed more than once, but that's still better than locking all
   * the time. It may be possible to implement something smarter in the future. */
  std::shared_ptr<CachedValue> result = compute_fn();
  /* Result should be valid. Use exception to propagate error if necessary. */
  BLI_assert(result);

  {
    ConcurrentMap::accessor accessor;
    const bool newly_inserted = cache.map.insert(accessor, std::ref(key));
    if (!newly_inserted) {
      /* The value is available already. The we unfortunately computed the value unnecessarily.
       * Use the value created by the other thread instead. */
      return accessor->second.value;
    }
    /* We want to store the key in the map, but the reference we got passed in may go out of scope.
     * So make a storable copy of it that we use in the map. */
    accessor->second.key = key.to_storable();
    /* Modifying the key should be fine because the new key is equal to the original key. */
    const_cast<std::reference_wrapper<const GenericKey> &>(accessor->first) = std::ref(
        *accessor->second.key);

    /* Store the value. Don't move, because we still want to return the value from the function. */
    accessor->second.value = result;
    /* Set initial logical time for the new cached entry. */
    set_new_logical_time(accessor->second, new_time);

    {
      /* Update global data of the cache. */
      std::lock_guard lock{cache.global_mutex};
      memory_counter::MemoryCounter memory_counter{cache.memory};
      accessor->second.value->count_memory(memory_counter);
      cache.keys.append(&accessor->first.get());
      cache.size_in_bytes = cache.memory.total_bytes;
    }
  }
  /* Potentially free elements from the cache. Note, even if this would free the value we just
   * added, it would still work correctly, because we already have a shared_ptr to it. */
  try_enforce_limit();
  return result;
}

void set_approximate_size_limit(const int64_t capacity)
{
  Cache &cache = get_cache();
  cache.approximate_limit = capacity;
  try_enforce_limit();
}

static void try_enforce_limit()
{
  Cache &cache = get_cache();
  const int64_t old_size = cache.size_in_bytes.load(std::memory_order_relaxed);
  const int64_t approximate_limit = cache.approximate_limit.load(std::memory_order_relaxed);
  if (old_size < approximate_limit) {
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
    /* Undershoot a little bit. This typically results in more things being freed that have not
     * been used in a while. The benefit is that we have to do the decision what to free less
     * often than if we were always just freeing the minimum amount necessary. */
    if (cache.memory.total_bytes <= approximate_limit * 0.75) {
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
  if (cache.memory.total_bytes < approximate_limit * 1.1) {
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
  cache.size_in_bytes = cache.memory.total_bytes;
}

}  // namespace blender::memory_cache
