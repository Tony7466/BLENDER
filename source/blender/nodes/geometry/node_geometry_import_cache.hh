/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <string>

#include "BKE_geometry_set.hh"
#include "BKE_report.hh"

#include "BLI_memory_cache.hh"
#include "BLI_memory_counter.hh"

namespace blender::nodes::geometry_import_cache {

enum class FileType : uint8_t { STL, OBJ, PLY };

class GeometryReadValue : public memory_cache::CachedValue {
 public:
  bke::GeometrySet geometry;
  ReportList reports;

  GeometryReadValue(bke::GeometrySet geometry, ReportList *reports) : geometry(geometry)
  {
    BKE_reports_init(&this->reports, RPT_STORE);
    BKE_reports_move_to_reports(&this->reports, reports);
  }

  ~GeometryReadValue()
  {
    BKE_reports_free(&reports);
  }

  void count_memory(MemoryCounter &memory) const override
  {
    geometry.count_memory(memory);
  }
};

void import_geometry_cache_clear_all();
std::shared_ptr<const GeometryReadValue> import_geometry_cached(
    const FileType file_type,
    const StringRef absolute_file_path,
    FunctionRef<std::unique_ptr<GeometryReadValue>()> compute_fn);

}  // namespace blender::nodes::geometry_import_cache
