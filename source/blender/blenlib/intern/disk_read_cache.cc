/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <mutex>

#include "BLI_disk_read_cache.hh"
#include "BLI_map.hh"

namespace blender::disk_read_cache {

struct Cache {
  std::mutex mutex;
  Vector<std::unique_ptr<ReadKey>> keys;
  Map<std::reference_wrapper<const ReadKey>, std::unique_ptr<ReadValue>> map;
};

static Cache &get_cache()
{
  static Cache cache;
  return cache;
}

const ReadValue *read(std::unique_ptr<ReadKey> key,
                      std::unique_ptr<ReadValue> (*read_fn)(const ReadKey &key))
{
  Cache &cache = get_cache();
  std::lock_guard lock{cache.mutex};
  const ReadKey &key_ref = *key;
  if (!cache.map.contains(key_ref)) {
    if (cache.map.size() > 200) {
      cache.map.clear();
    }
    std::unique_ptr<ReadValue> value = read_fn(key_ref);
    cache.map.add(key_ref, std::move(value));
    cache.keys.append(std::move(key));
  }
  return cache.map.lookup(key_ref).get();
}

}  // namespace blender::disk_read_cache
