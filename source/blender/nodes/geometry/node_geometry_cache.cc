/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_map.hh"

#include "node_geometry_cache.hh"

static blender::nodes::GeometryCache &get_cache()
{
  static blender::nodes::GeometryCache geometry_cache(50);
  return geometry_cache;
}

namespace blender::nodes {
GeometryCache::GeometryCache(uint64_t size)
{
  this->size = size;
}

void GeometryCache::add(std::string key, bke::GeometrySet geometry)
{
  GeometryCacheNode node{nullptr, nullptr, key};  // stack allocation ?
  list_push_first(node);
  cache_map.add(key, std::make_pair(node, geometry));
  balance();
}

bool GeometryCache::contains(std::string key)
{
  return cache_map.contains(key);
}

void GeometryCache::remove(std::string key)
{
  GeometryCacheNode node = cache_map.lookup(key).first;
  list_pop(node);
  cache_map.remove(node.key);
}

void GeometryCache::clear()
{
  cache_map.clear();  // this should work as long as map owns the list nodes
}

bke::GeometrySet GeometryCache::get(std::string key)
{
  list_move_to_first(key);

  return cache_map.lookup(key).second;
}

void GeometryCache::resize(uint64_t size)
{
  this->size = size;
  balance();
}

void GeometryCache::balance()
{
  while (cache_map.size() > size) {
    GeometryCacheNode node = list_pop_last();
    cache_map.remove(node.key);
  }
}

void GeometryCache::list_push_first(GeometryCacheNode &node)
{
  GeometryCacheNode *current_first = cache_list.first;

  if (current_first == nullptr) {  // list is empty
    cache_list.first = &node;
    cache_list.last = &node;
  }
  else {  // list is not empty
    node.next = current_first;
    current_first->prev = &node;
    cache_list.first = &node;
  }
}

void GeometryCache::list_move_to_first(std::string key)
{
  if (cache_map.size() <= 1)
    return;

  GeometryCacheNode node = cache_map.lookup(key).first;

  list_pop(node);

  list_push_first(node);
}

GeometryCacheNode GeometryCache::list_pop_last()
{
  GeometryCacheNode node = *cache_list.last;
  list_pop(node);
  return node;
}

void GeometryCache::list_pop(GeometryCacheNode &node)
{
  node.prev->next = node.next;
  if (node.next != nullptr) {
    node.next->prev = node.prev;
  }
  else {
    cache_list.last = node.prev;
  }
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
  return get_cache().get(key);  // get returns an optional type
}
}  // namespace blender::nodes
