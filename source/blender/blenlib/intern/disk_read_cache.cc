/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <mutex>

#include "BLI_disk_read_cache.hh"
#include "BLI_map.hh"
#include "BLI_memory_cache.hh"

namespace blender::disk_read_cache {

struct Cache {
  std::mutex mutex;
  Map<std::reference_wrapper<const GenericKey>, std::shared_ptr<ReadValue>> map;
};

static Cache &get_cache()
{
  static Cache cache;
  return cache;
}

std::shared_ptr<const ReadValue> read(const GenericKey &key,
                                      const FunctionRef<std::unique_ptr<ReadValue>()> read_fn)
{
  Cache &cache = get_cache();
  std::shared_ptr<const ReadValue> result;
  bool newly_added = false;
  {
    std::lock_guard lock{cache.mutex};
    if (!cache.map.contains(key)) {
      /* Don't use #lookup_or_add_cb because the callback might add another value. */
      std::unique_ptr<ReadValue> value = read_fn();
      ReadValue &value_ref = *value;
      value_ref.key = key.to_storable();
      cache.map.add(*value_ref.key, std::move(value));
      newly_added = true;
    }
    result = cache.map.lookup(key);
  }

  if (newly_added) {
    memory_cache::MemoryCache &global_memory_cache = memory_cache::global_cache();
    global_memory_cache.add(
        key.to_storable(),
        [&](MemoryCounter &memory) { result->count_memory(memory); },
        [cache = &cache, key = result->key.get()]() {
          std::lock_guard lock{cache->mutex};
          cache->map.remove(*key);
        });
  }

  return result;
}

}  // namespace blender::disk_read_cache
