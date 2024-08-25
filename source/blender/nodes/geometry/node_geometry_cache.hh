/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <list>
#include <string>

#include "BKE_geometry_set.hh"

#include "BLI_memory_cache.hh"
#include "BLI_memory_counter.hh"

namespace blender::nodes::geometry_cache {

class GeometryReadValue : public memory_cache::CachedValue {
 public:
  bke::GeometrySet geometry;

  GeometryReadValue(bke::GeometrySet geometry) : geometry(geometry) {}

  void count_memory(MemoryCounter &memory) const override
  {
    geometry.count_memory(memory);
  }
};

void import_geometry_cache_clear_all();
bke::GeometrySet import_geometry_cached(
    std::string key, FunctionRef<std::unique_ptr<GeometryReadValue>()> compute_fn);

}  // namespace blender::nodes::geometry_cache
