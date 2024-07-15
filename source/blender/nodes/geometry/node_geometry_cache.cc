/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_map.hh"

#include "BKE_geometry_set.hh"

#include "node_geometry_cache.hh"

static blender::Map<std::string, blender::bke::GeometrySet> &get_cache()
{
  static blender::Map<std::string, blender::bke::GeometrySet> geometry_cache;
  return geometry_cache;
}

void blender::geo_cache_set(std::string key, blender::bke::GeometrySet geometry)
{
  get_cache().add(key, geometry);
}

bool blender::geo_cache_contains(std::string key)
{
  return get_cache().contains(key);
}

void blender::geo_cache_clear(std::string key)
{
  get_cache().remove(key);
}

void blender::geo_cache_clear_all()
{
  get_cache().clear();
}

blender::bke::GeometrySet blender::geo_cache_get(std::string key)
{
  return get_cache().lookup(key);
}
