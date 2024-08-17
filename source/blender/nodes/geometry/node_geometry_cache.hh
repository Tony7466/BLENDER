/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <list>
#include <string>

#include "BKE_geometry_set.hh"

#include "BLI_map.hh"
#include "BLI_memory_counter.hh"

// namespace blender::bke {
// struct GeometrySet;
// }

namespace blender::nodes {

/**
 * LRU based geoemetry cache
 */
class GeometryCache {
 private:
  typedef std::pair<std::string, bke::GeometrySet> cache_entry;
  typedef std::list<cache_entry>::iterator cache_node;

  uint64_t capacity;
  uint64_t size;

  std::list<cache_entry> cache_list;
  blender::Map<std::string, cache_node> cache_map;

  MemoryCount memory_count;

 public:
  GeometryCache(uint64_t capacity);

  void add(std::string key, bke::GeometrySet geometry);
  bool contains(std::string key);
  void remove(std::string key);
  void clear();
  bke::GeometrySet get(std::string key);
  void resize(uint64_t capacity);

 private:
  void balance();
  uint64_t get_geometry_size_in_bytes(bke::GeometrySet &geometry);
};

void geo_cache_set(std::string key, bke::GeometrySet geometry);
bool geo_cache_contains(std::string key);
void geo_cache_clear(std::string key);
void geo_cache_clear_all();
bke::GeometrySet geo_cache_get(std::string key);
}  // namespace blender::nodes
