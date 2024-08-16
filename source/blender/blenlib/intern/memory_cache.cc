/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include "BLI_memory_cache.hh"

namespace blender::memory_cache {

MemoryCache &global_cache()
{
  static MemoryCache cache;
  return cache;
}

MemoryCache::MemoryCache(const int64_t capacity_in_bytes) : capacity_in_bytes_(capacity_in_bytes)
{
}

void MemoryCache::add(std::unique_ptr<GenericKey> key,
                      FunctionRef<void(MemoryCounter &memory)> count_memory_fn,
                      std::function<void()> free_fn)
{
  std::lock_guard lock{mutex_};

  const GenericKey &key_ref = *key;
  BLI_assert(!cache_.contains(key_ref));

  OwnedMemory owned_memory;
  MemoryCounter memory_counter{owned_memory, memory_by_shared_data_};
  count_memory_fn(memory_counter);

  this->add_memory_of(owned_memory);

  Value value;
  value.key = std::move(key);
  value.memory = std::move(owned_memory);
  value.last_use = ++logical_time_;
  value.free_fn = std::move(free_fn);

  cache_.add_new(key_ref, std::move(value));

  this->free_if_necessary();
}

void MemoryCache::remove(const GenericKey &key)
{
  std::lock_guard lock{mutex_};
  const Value value = cache_.pop(key);
  this->subtract_memory_of(value.memory);
}

void MemoryCache::touch(const GenericKey &key)
{
  std::lock_guard lock{mutex_};
  cache_.lookup(key).last_use = ++logical_time_;
}

void MemoryCache::free_if_necessary()
{
  Vector<std::pair<int, const GenericKey *>> time_with_keys;
  for (auto &&item : cache_.items()) {
    time_with_keys.append({item.value.last_use, &item.key.get()});
  }
  std::sort(time_with_keys.begin(), time_with_keys.end());

  int index = 0;
  while (current_bytes_ > capacity_in_bytes_) {
    const GenericKey &key = *time_with_keys[index++].second;
    const Value value = cache_.pop(key);
    this->subtract_memory_of(value.memory);
    value.free_fn();
  }
}

void MemoryCache::add_memory_of(const OwnedMemory &memory)
{
  /* TODO: Handle shared memory. */
  current_bytes_ += memory.uniquely_owned_bytes;
}

void MemoryCache::subtract_memory_of(const OwnedMemory &memory)
{
  /* TODO: Handle shared memory. */
  current_bytes_ -= memory.uniquely_owned_bytes;
}

}  // namespace blender::memory_cache
