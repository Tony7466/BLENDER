/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_string_search.hh"

#include "BLI_map.hh"

namespace blender::bke::string_search {

struct RecentSearches {
  int logical_clock = 0;
  Map<std::string, int> map;
};

RecentSearches &get_recent_searches()
{
  static RecentSearches recent_searches;
  return recent_searches;
}

void add_recent_search(const StringRef search_id, const StringRef choosen_str)
{
  std::string key = search_id + choosen_str;
  RecentSearches &searches = get_recent_searches();
  searches.map.add_overwrite(std::move(key), searches.logical_clock);
  searches.logical_clock++;
}

}  // namespace blender::bke::string_search
