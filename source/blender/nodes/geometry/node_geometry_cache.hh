/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <string>

#include "BKE_geometry_set.hh"
#include "BLI_map.hh"

// namespace blender::bke {
// struct GeometrySet;
// }

namespace blender::nodes {

/**
 * Based on DNA_listBase.h
 */
struct GeometryCacheNode {
  GeometryCacheNode *next;
  GeometryCacheNode *prev;
  std::string key;
};

/**
 * LRU based geoemetry cache
 */
class GeometryCache {
 private:
  uint64_t size;

  blender::Map<std::string, std::pair<GeometryCacheNode, bke::GeometrySet>> cache_map;
  struct {
    GeometryCacheNode *first;
    GeometryCacheNode *last;
  } cache_list;

 public:
  GeometryCache(uint64_t size);

  void add(std::string key, bke::GeometrySet geometry);
  bool contains(std::string key);
  void remove(std::string key);
  void clear();
  bke::GeometrySet get(std::string key);
  void resize(uint64_t size);

 private:
  void balance();

  void list_push_first(GeometryCacheNode &node);
  void list_move_to_first(std::string key);
  GeometryCacheNode list_pop_last();
  void list_pop(GeometryCacheNode &node);
};

void geo_cache_set(std::string key, bke::GeometrySet geometry);
bool geo_cache_contains(std::string key);
void geo_cache_clear(std::string key);
void geo_cache_clear_all();
bke::GeometrySet geo_cache_get(std::string key);
}  // namespace blender::nodes
