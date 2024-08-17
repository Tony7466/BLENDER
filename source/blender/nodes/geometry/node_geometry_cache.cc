/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_map.hh"

#include "node_geometry_cache.hh"

static blender::nodes::GeometryCache &get_cache()
{
  static blender::nodes::GeometryCache geometry_cache(256 * 1024 * 1024);  // 256MB
  return geometry_cache;
}

namespace blender::nodes {
GeometryCache::GeometryCache(uint64_t capacity)
{
  this->capacity = capacity;
}

void GeometryCache::add(std::string key, bke::GeometrySet geometry)
{
  cache_node node = cache_list.insert(cache_list.begin(), std::make_pair(key, geometry));
  cache_map.add(key, node);

  size += get_geometry_size_in_bytes(geometry);

  balance();
}

bool GeometryCache::contains(std::string key)
{
  return cache_map.contains(key);
}

void GeometryCache::remove(std::string key)
{
  cache_node node = cache_map.lookup(key);
  cache_list.erase(node);
  cache_map.remove(key);
}

void GeometryCache::clear()
{
  cache_list.clear();
  cache_map.clear();
}

bke::GeometrySet GeometryCache::get(std::string key)
{
  cache_node node = cache_map.lookup(key);
  cache_list.push_front(*node);
  cache_list.erase(node);
  return node->second;
}

void GeometryCache::resize(uint64_t capacity)
{
  this->capacity = capacity;
  balance();
}

void GeometryCache::balance()
{
  while (size > capacity) {
    cache_node last_node = --cache_list.end();

    size -= get_geometry_size_in_bytes(last_node->second);

    cache_map.pop(last_node->first);
    cache_list.pop_back();
  }

  fprintf(stderr, "Cache Size: %lld\n", size);
}

uint64_t GeometryCache::get_geometry_size_in_bytes(bke::GeometrySet &geometry)
{
  memory_count.reset();
  MemoryCounter memory{memory_count};
  geometry.count_memory(memory);
  fprintf(stderr, "Item Size: %lld\n", memory_count.total_bytes);
  return memory_count.total_bytes;
}

void geo_cache_set(std::string key, blender::bke::GeometrySet geometry)
{
  get_cache().add(key, geometry);
}

bool geo_cache_contains(std::string key)
{
  return get_cache().contains(key);
}

void geo_cache_clear(std::string key)
{
  get_cache().remove(key);
}

void geo_cache_clear_all()
{
  get_cache().clear();
}

blender::bke::GeometrySet geo_cache_get(std::string key)
{
  return get_cache().get(key);
}
}  // namespace blender::nodes
