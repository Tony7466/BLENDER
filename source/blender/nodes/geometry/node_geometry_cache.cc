/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <boost/compute/detail/lru_cache.hpp>

#include "BLI_map.hh"

#include "BKE_geometry_set.hh"

#include "node_geometry_cache.hh"

static boost::compute::detail::lru_cache<std::string, blender::bke::GeometrySet> &get_cache()
{
  static boost::compute::detail::lru_cache<std::string, blender::bke::GeometrySet> geometry_cache(
      50);
  // static blender::Map<std::string, blender::bke::GeometrySet> geometry_cache;
  return geometry_cache;
}

void blender::geo_cache_set(std::string key, blender::bke::GeometrySet geometry)
{
  get_cache().insert(key, geometry);
}

bool blender::geo_cache_contains(std::string key)
{
  return get_cache().contains(key);
}

void blender::geo_cache_clear(std::string /*key*/)
{
  // TODO: Boost LRU cache has not remove, items will eventually be evicted is size > capacity
  // get_cache().remove(key);
}

void blender::geo_cache_clear_all()
{
  get_cache().clear();
}

blender::bke::GeometrySet blender::geo_cache_get(std::string key)
{
  return get_cache().get(key).get();  // get returns an optional type
}
