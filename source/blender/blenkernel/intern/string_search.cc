/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_string_search.hh"

#include "BLI_map.hh"

namespace blender::bke::string_search {

using blender::string_search::RecentCache;

struct RecentCacheStorage {
  int logical_clock = 0;
  Map<std::string, RecentCache> map;
};

static RecentCacheStorage &get_recent_cache_storage()
{
  static RecentCacheStorage storage;
  return storage;
}

void add_recent_search(const StringRef choosen_str)
{
  const StringRef search_id = "";
  RecentCacheStorage &storage = get_recent_cache_storage();
  RecentCache &cache = storage.map.lookup_or_add_default_as(search_id);
  cache.logical_time_by_str.add_overwrite(choosen_str, storage.logical_clock);
  storage.logical_clock++;
}

const RecentCache *get_recent_cache()
{
  const StringRef search_id = "";
  RecentCacheStorage &storage = get_recent_cache_storage();
  return storage.map.lookup_ptr_as(search_id);
}

}  // namespace blender::bke::string_search
